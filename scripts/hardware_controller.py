import logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.types import RobotObservation, RobotAction

import numpy as np
import time
import lcm
from datetime import datetime, timedelta

import os
import sys
module_path = os.path.abspath(os.path.join('..'))
if module_path not in sys.path:
    sys.path.append(module_path)
from lcmdefs.messages.so101 import lcmt_so101_configuration


joint_names = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
follower_name = "my_awesome_follower_arm"
biases = np.load(f"../calibrations/{follower_name}_biases.npy")
config = SO101FollowerConfig(
    port="/dev/ttyACM1",
    id=follower_name,
    use_degrees=True
)
follower = SO101Follower(config)


def unpack_observation(observation: RobotObservation) -> np.ndarray:
    q_raw = np.array([observation[j] for j in joint_names], dtype=np.int16)
    return (q_raw - 2048) * (2 * np.pi / 4096)

def pack_action(q_desired: np.ndarray) -> RobotAction:
    q_raw = (q_desired * (4096 / (2 * np.pi))) + 2048
    return {j: int(q) for j, q in zip(joint_names, q_raw)}

def command_handler(channel, data):
    msg = lcmt_so101_configuration.decode(data)
    q_next = np.array(msg.q, dtype=np.float32)
    q_next_biased = q_next - biases
    action = pack_action(q_next_biased)
    follower.bus.sync_write("Goal_Position", action, normalize=False)


msg = lcmt_so101_configuration()
lc = lcm.LCM()
subscription = lc.subscribe("SO101_COMMAND", command_handler)
subscription.set_queue_capacity(1)


try:
    follower.connect(calibrate=False)
    ctr = 0

    while True:
        start_time = datetime.now()
        target_time = start_time + timedelta(milliseconds=5)

        obs_dict: RobotObservation = follower.bus.sync_read("Present_Position", normalize=False)
        q_current_biased = unpack_observation(obs_dict)
        q_current = q_current_biased + biases
        msg.q = q_current.tolist()
        lc.publish("SO101_STATUS", msg.encode())

        if ctr % 200 == 0:
            print(f'ctr: {ctr}, q_current: {q_current}')

        try:
            lc.handle_timeout(2)
        except OSError:
            break

        ctr += 1

        end_time = datetime.now()
        sleep_time = max(0, (target_time - end_time).total_seconds())
        time.sleep(sleep_time)

except KeyboardInterrupt:
    pass
finally:
    while True:
        try:
            follower.disconnect()
            break
        except KeyboardInterrupt:
            pass