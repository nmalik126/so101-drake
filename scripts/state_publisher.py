import logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.types import RobotObservation

import numpy as np
import time
import lcm

import os
import sys
module_path = os.path.abspath(os.path.join('..'))
if module_path not in sys.path:
    sys.path.append(module_path)
from lcmdefs.messages.so101 import lcmt_so101_configuration


def unpack_observation(observation: RobotObservation) -> np.ndarray:
    q_raw = np.array([
        observation['shoulder_pan'],
        observation['shoulder_lift'],
        observation['elbow_flex'],
        observation['wrist_flex'],
        observation['wrist_roll'],
        observation['gripper'],
    ], dtype=np.int16)
    return (q_raw - 2048) * (2 * np.pi / 4096)


follower_name = "my_awesome_follower_arm"

biases = np.load(f"../calibrations/{follower_name}_biases.npy")

config = SO101FollowerConfig(
    port="/dev/ttyACM1",
    id=follower_name,
    use_degrees=True
)
follower = SO101Follower(config)

msg = lcmt_so101_configuration()
lc = lcm.LCM()


try:
    follower.connect(calibrate=False)
    ctr = 0

    while True:
        obs_dict: RobotObservation = follower.bus.sync_read("Present_Position", normalize=False)
        q_raw = unpack_observation(obs_dict)

        q_current = q_raw + biases

        msg.q = q_current.tolist()
        lc.publish("SO101_STATUS", msg.encode())

        if ctr % 10 == 0:
            print(f'ctr: {ctr}, q: {q_current}')

        ctr += 1
        time.sleep(0.1)

except KeyboardInterrupt:
    pass
finally:
    follower.disconnect()