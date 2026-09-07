import logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

from pathlib import Path
import numpy as np
import time
import pickle
from concurrent.futures import ThreadPoolExecutor

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

from pydrake.all import CompositeTrajectory, BsplineTrajectory

import grpc
import pipeline_pb2
import pipeline_pb2_grpc


class TrajectoryExecutor:

    joint_names = [
        'shoulder_pan', 
        'shoulder_lift', 
        'elbow_flex', 
        'wrist_flex', 
        'wrist_roll', 
        'gripper'
    ]

    follower_name = "my_awesome_follower_arm"
    port = "/dev/ttyACM1"

    traj_exec_rate_hz = 200

    @classmethod
    def start(cls):
        try:
            logging.info("starting trajectory executor...")

            project_dir = Path("/home/noor/so101-drake")
            cls.biases = np.load(
                project_dir / "calibrations" / f"{cls.follower_name}_biases.npy"
            )

            config = SO101FollowerConfig(
                port=cls.port, id=cls.follower_name, use_degrees=True
            )
            cls.follower = SO101Follower(config)

            cls.follower.connect(calibrate=False)

            logging.info("trajectory executor started")

        except Exception:
            logging.error(f"trajectory executor start unsuccessful, stopping...")
            cls.stop()
            raise

    @classmethod
    def stop(cls):
        logging.info("stopping trajectory executor...")
        while True:
            try:
                cls.follower.disconnect()
                break
            except KeyboardInterrupt:
                pass
        logging.info("trajectory executor stopped")

    @classmethod
    def _write_robot_config(cls, q_next: np.ndarray):
        q_next_biased = q_next - cls.biases
        q_raw = (q_next_biased * (4096 / (2 * np.pi))) + 2048
        action = {j: int(q) for j, q in zip(cls.joint_names, q_raw)}
        cls.follower.bus.sync_write("Goal_Position", action, normalize=False)

    @classmethod
    def execute_trajectory(cls, traj: CompositeTrajectory):
        logging.info("executing trajectory...")

        time_step = 1.0 / cls.traj_exec_rate_hz
        for t in np.append(
            np.arange(traj.start_time(), traj.end_time(), time_step),
            traj.end_time(),
        ):
            q_next = traj.value(t).flatten()
            # logging.info(f"q_next: {q_next}")
            cls._write_robot_config(q_next)
            time.sleep(time_step)

        logging.info("trajectory executed")


class PipelineServicer(pipeline_pb2_grpc.PipelineServicer):

    def ExecutePlan(self, request: pipeline_pb2.TrajectoryRequest, context):
        TrajectoryExecutor.execute_trajectory(
            pickle.loads(request.traj.traj)
        )
        return pipeline_pb2.Response(
            id=request.request.id,
            success=True
        )

    
def serve():
    server = grpc.server(ThreadPoolExecutor(max_workers=10))
    pipeline_pb2_grpc.add_PipelineServicer_to_server(
        PipelineServicer(), server
    )
    server.add_insecure_port("127.0.0.1:50054")
    server.start()
    server.wait_for_termination()    


def main():
    try:
        TrajectoryExecutor.start()
        serve()
    except KeyboardInterrupt:
        logging.info("User keyboard interrupt")
    except Exception as e:
        logging.exception(f"An unexpected error occured: {e}")
    finally:
        TrajectoryExecutor.stop()


if __name__ == "__main__":
    main()
