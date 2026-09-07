import grpc
import pipeline_pb2
import pipeline_pb2_grpc
import uuid
import logging
import numpy as np
import matplotlib.pyplot as plt
import pickle

from pydrake.all import (
    BsplineTrajectory, 
    CompositeTrajectory,
    PiecewisePolynomial
)


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

def get_plan(grasp: pipeline_pb2.Grasp, q_start: np.ndarray):
    with grpc.insecure_channel("127.0.0.1:50053") as channel:
        stub = pipeline_pb2_grpc.PipelineStub(channel)

        request = pipeline_pb2.GraspRequest(
            request=pipeline_pb2.Request(id=str(uuid.uuid4())),
            grasp=grasp,
            q_start=q_start
        )
        response: pipeline_pb2.PlanTrajectoriesResponse = stub.GetPlan(request)

        assert request.request.id == response.response.id
        if not response.response.success:
            return None

        pick_traj: BsplineTrajectory = pickle.loads(
            response.planTrajectories.pick_traj.traj
        )
        place_traj: BsplineTrajectory = pickle.loads(
            response.planTrajectories.place_traj.traj
        )
        return_traj: BsplineTrajectory = pickle.loads(
            response.planTrajectories.return_traj.traj
        )

        return pick_traj, place_traj, return_traj

        logging.debug(f"pick traj start time: {pick_traj.start_time()}, end time: {pick_traj.end_time()}, start val: {pick_traj.InitialValue()}, end_val: {pick_traj.FinalValue()}")
        logging.debug(f"place traj start time: {place_traj.start_time()}, end time: {place_traj.end_time()}, start val: {place_traj.InitialValue()}, end_val: {place_traj.FinalValue()}")
        logging.debug(f"return traj start time: {return_traj.start_time()}, end time: {return_traj.end_time()}, start val: {return_traj.InitialValue()}, end_val: {return_traj.FinalValue()}")
    

def execute_plan(traj: CompositeTrajectory):
    with grpc.insecure_channel("127.0.0.1:50054") as channel:
        stub = pipeline_pb2_grpc.PipelineStub(channel)

        request = pipeline_pb2.TrajectoryRequest(
            request=pipeline_pb2.Request(id=str(uuid.uuid4())),
            traj=pipeline_pb2.Trajectory(traj=pickle.dumps(traj))
        )
        response: pipeline_pb2.Response = stub.ExecutePlan(request)

        assert request.request.id == response.id
        if not response.success:
            logging.warning("plan execution unsuccessful")
            return

        logging.info("plan execution successful")


def main():
    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    )

    q_rest = np.array([0, -1.822, 1.55, 0.906, 0, 0])
    q_ready = np.array([0, 0, 0, 1.5, 0, np.pi/4])

    rest_to_ready_traj = PiecewisePolynomial.CubicShapePreserving(
        [0.0, 3.0], 
        np.vstack([q_rest, q_ready]).T,
        True
    )
    ready_to_rest_traj = PiecewisePolynomial.CubicShapePreserving(
        [0.0, 3.0], 
        np.vstack([q_ready, q_rest]).T,
        True
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

        plan = get_plan(grasp, q_ready)
        if plan is None:
            logging.warning("motion plan unsuccessful")
        logging.info("motion plan successful")

        pick_traj, place_traj, return_traj = plan

        q_pick_after = pick_traj.FinalValue().flatten()
        q_place = place_traj.FinalValue().flatten()
        # logging.info(f"q_pick_after: {q_pick_after}")
        # logging.info(f"q_place: {q_place}")

        q_pick_closed = q_pick_after.copy()
        q_pick_closed[5] = -0.1
        close_traj = PiecewisePolynomial.CubicShapePreserving(
            [0.0, 1.0],
            np.vstack([q_pick_after, q_pick_closed]).T,
            True
        )
        q_place_closed = q_place.copy()
        q_place_closed[5] = -0.1
        open_traj = PiecewisePolynomial.CubicShapePreserving(
            [0.0, 1.0],
            np.vstack([q_place_closed, q_place]).T,
            True
        )

        control_pts = [
            control_pt.copy() for control_pt in place_traj.control_points()
        ]
        for control_pt in control_pts:
            control_pt[5, 0] = -0.1
        place_traj = BsplineTrajectory(place_traj.basis(), control_pts)

        full_traj = CompositeTrajectory.AlignAndConcatenate([
            rest_to_ready_traj,
            pick_traj,
            close_traj,
            place_traj,
            open_traj,
            return_traj,
            ready_to_rest_traj
        ])

        execute_plan(full_traj)

        # from pathlib import Path
        # project_dir = Path("/home/noor/so101-drake")
        # with open(project_dir / "assets" / "example_full_traj.pkl", "wb") as f:
        #     pickle.dump(full_traj, f)

        # q_rest = np.array([0, -1.822, 1.55, 0.906, 0, 0])
        # q_open = np.array([0, -1.822, 1.55, 0.906, 0, np.pi/4])
        # traj = CompositeTrajectory.AlignAndConcatenate([
        #     PiecewisePolynomial.CubicShapePreserving(
        #         [0.0, 2.0],
        #         np.vstack([q_rest, q_open]).T,
        #         True
        #     ),
        #     PiecewisePolynomial.CubicShapePreserving(
        #         [0.0, 2.0],
        #         np.vstack([q_open, q_rest]).T,
        #         True
        #     ),
        # ])
        # execute_plan(traj)
        
    except KeyboardInterrupt:
        logging.info("User keyboard interrupt")
    except Exception as e:
        logging.exception(f"An unexpected error occured: {e}")


if __name__ == "__main__":
    main()
