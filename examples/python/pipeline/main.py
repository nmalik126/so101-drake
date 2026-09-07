import grpc
import pipeline_pb2
import pipeline_pb2_grpc
import uuid
import logging
import numpy as np
import matplotlib.pyplot as plt
import pickle

from pydrake.all import BsplineTrajectory


def get_image():
    with grpc.insecure_channel("127.0.0.1:50051") as channel:
        stub = pipeline_pb2_grpc.PipelineStub(channel)

        request = pipeline_pb2.Request(id=str(uuid.uuid4()))
        response: pipeline_pb2.ImagePairResponse = stub.GetImage(request)

        assert request.id == response.response.id
        if not response.response.success:
            return None

        return response.imagePair
        
        logging.info("image request successful")
        depth_image = np.frombuffer(
            response.imagePair.depth_data, dtype=np.float32
        ).reshape((480, 848))
        color_image = np.frombuffer(
            response.imagePair.color_data, dtype=np.uint8
        ).reshape((480, 848, 3))
        # logging.debug(f"depth image: {depth_image}")
        # logging.debug(f"color image: {color_image}")

        # fig, ax = plt.subplots(2, 1)
        # ax[0].imshow(color_image)
        # im = ax[1].imshow(depth_image)
        # fig.colorbar(im, ax=ax[1])
        # plt.show()


def get_grasp(imagePair: pipeline_pb2.ImagePair):
    with grpc.insecure_channel("127.0.0.1:50052") as channel:
        stub = pipeline_pb2_grpc.PipelineStub(channel)

        request = pipeline_pb2.ImagePairRequest(
            request=pipeline_pb2.Request(id=str(uuid.uuid4())),
            imagePair=imagePair
        )
        response: pipeline_pb2.GraspResponse = stub.GetGrasp(request)

        assert request.request.id == response.response.id
        if not response.response.success:
            return None

        return response.grasp

        grasp = np.array(
            response.grasp.grasp, dtype=np.float32
        ).reshape((4, 4))
        logging.debug(f"grasp: {grasp}")

def get_plan(grasp: pipeline_pb2.Grasp):
    with grpc.insecure_channel("127.0.0.1:50053") as channel:
        stub = pipeline_pb2_grpc.PipelineStub(channel)

        q_ready = np.array([0, 0, 0, 1.5, 0, np.pi/4])
        request = pipeline_pb2.GraspRequest(
            request=pipeline_pb2.Request(id=str(uuid.uuid4())),
            grasp=grasp,
            q_start=q_ready
        )
        response: pipeline_pb2.PlanTrajectoriesResponse = stub.GetPlan(request)

        assert request.request.id == response.response.id
        if not response.response.success:
            logging.warning("motion plan unsuccessful")
            return

        pick_traj: BsplineTrajectory = pickle.loads(
            response.planTrajectories.pick_traj.traj
        )
        place_traj: BsplineTrajectory = pickle.loads(
            response.planTrajectories.place_traj.traj
        )
        return_traj: BsplineTrajectory = pickle.loads(
            response.planTrajectories.return_traj.traj
        )

        logging.debug(f"pick traj start time: {pick_traj.start_time()}, end time: {pick_traj.end_time()}, start val: {pick_traj.InitialValue()}, end_val: {pick_traj.FinalValue()}")
        logging.debug(f"place traj start time: {place_traj.start_time()}, end time: {place_traj.end_time()}, start val: {place_traj.InitialValue()}, end_val: {place_traj.FinalValue()}")
        logging.debug(f"return traj start time: {return_traj.start_time()}, end time: {return_traj.end_time()}, start val: {return_traj.InitialValue()}, end_val: {return_traj.FinalValue()}")
    

def main():
    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    )

    try:
        imagePair = get_image()
        if imagePair is None:
            logging.warning("image request unsuccessful")
            return
        logging.info("image request successful")

        grasp = get_grasp(imagePair)
        if grasp is None:
            logging.warning("grasp request unsuccessful")
            return
        logging.info("grasp request successful")

        get_plan(grasp)
        
    except KeyboardInterrupt:
        logging.info("User keyboard interrupt")
    except Exception as e:
        logging.exception(f"An unexpected error occured: {e}")


if __name__ == "__main__":
    main()
