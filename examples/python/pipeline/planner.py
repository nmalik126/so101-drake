from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import pickle
from concurrent.futures import ThreadPoolExecutor
import logging

import grpc
import pipeline_pb2
import pipeline_pb2_grpc

from pydrake.all import (
    RobotDiagramBuilder,
    Parser,
    RigidTransform,
    RotationMatrix,
    RobotDiagram,
    InverseKinematics,
    Solve,
    CollisionCheckerParams,
    SceneGraphCollisionChecker,
    MinimumDistanceLowerBoundConstraint,
    KinematicTrajectoryOptimization,
)


class MotionPlanner:

    def __init__(self):
        logging.info("initializing motion planner...")

        project_dir = Path("/home/noor/so101-drake")
        
        robot_builder = RobotDiagramBuilder()
        self.plant = robot_builder.plant()
    
        parser = Parser(self.plant)
    
        T_world_base = np.eye(4)
        rotvec = np.array([0, 0, 1]) * np.pi/2
        T_world_base[:3, :3] = Rotation.from_rotvec(rotvec).as_matrix()
        T_world_base[:3, 3] = [0, -0.1775, 0.0074]
    
        self.so101 = parser.AddModels(
            project_dir / "models" / "SO101" / "so101_new_calib_drake_hydro.urdf"
        )[0]
        self.plant.WeldFrames(
            self.plant.world_frame(),
            self.plant.GetFrameByName("base_link"),
            RigidTransform(
                RotationMatrix(T_world_base[:3, :3]), 
                T_world_base[:3, 3]
            )
        )
    
        parser.AddModels(
            project_dir / "models" / "objects" / "mat_raised.sdf"
        )
        self.plant.WeldFrames(
            self.plant.world_frame(), 
            self.plant.GetFrameByName("mat_link"),
            RigidTransform([0, 0.15, 0])
        )
    
        self.plant.Finalize()
        diagram: RobotDiagram = robot_builder.Build()

        self.T_gripper_tcp = np.eye(4)
        self.T_gripper_tcp[2, 3] = -0.11

        collision_checker_params = CollisionCheckerParams()
        collision_checker_params.model = diagram
        collision_checker_params.robot_model_instances = [self.so101]
        collision_checker_params.edge_step_size = 0.01
        collision_checker = SceneGraphCollisionChecker(collision_checker_params)
        self.collision_constraint = MinimumDistanceLowerBoundConstraint(
            collision_checker,
            1e-3,
            collision_checker.MakeStandaloneModelContext(),
            None,
            1e-2,
        )

        logging.info("motion planner initialized")

    def solve_ik(
        self, 
        T_world_gripper: np.ndarray, 
        rot_tol_radians: float,
        offset: float
    ):
        # init
        grasp = T_world_gripper @ self.T_gripper_tcp
        ik = InverseKinematics(self.plant)
        ik.get_mutable_prog().AddQuadraticErrorCost(1.0, np.zeros(6), ik.q())
        ik.get_mutable_prog().SetInitialGuess(ik.q(), np.zeros(6))

        # constraints
        ik.AddPositionConstraint(
            self.plant.GetFrameByName("gripper_link", self.so101),
            self.T_gripper_tcp[:3, 3] + np.array([0, 0, offset]),
            self.plant.world_frame(),
            grasp[:3, 3] - 1e-3,
            grasp[:3, 3] + 1e-3
        )
        ik.AddOrientationConstraint(
            self.plant.GetFrameByName("gripper_link", self.so101),
            RotationMatrix(),
            self.plant.world_frame(),
            RotationMatrix(grasp[:3, :3]),
            rot_tol_radians
        )
        ik.get_mutable_prog().AddBoundingBoxConstraint(
            np.pi/4, np.pi/4, ik.q()[5]
        )

        # solve
        result = Solve(ik.prog())
        if result.is_success():
            return result.GetSolution(ik.q())
        else:
            return None

    def solve_trajopt(
        self,
        q_start: np.ndarray,
        q_goal: np.ndarray,
        collision_begin: float,
        collision_end: float,
        waypoints: list[tuple[np.ndarray, float]] | None = None
    ):
        trajopt = KinematicTrajectoryOptimization(6, 10, spline_order=5)

        trajopt.AddDurationCost(0.2)
        trajopt.AddPathEnergyCost(1.0)
        trajopt.AddPositionBounds(
            self.plant.GetPositionLowerLimits(), 
            self.plant.GetPositionUpperLimits()
        )
        trajopt.AddVelocityBounds(
            0.3 * self.plant.GetVelocityLowerLimits(),
            0.3 * self.plant.GetVelocityUpperLimits()
        )
        trajopt.AddAccelerationBounds(
            -2.0 * np.ones(6),
            2.0 * np.ones(6)
        )
        trajopt.AddJerkBounds(
            -1.0 * np.ones(6),
            1.0 * np.ones(6)
        )
        trajopt.AddDurationConstraint(0.5, 5.0)

        trajopt.AddPathPositionConstraint(lb=q_start, ub=q_start, s=0)
        trajopt.AddPathPositionConstraint(lb=q_goal, ub=q_goal, s=1)
        if waypoints is not None:
            for waypoint, s in waypoints:
                trajopt.AddPathPositionConstraint(lb=waypoint, ub=waypoint, s=s)

        trajopt.AddPathVelocityConstraint(np.zeros((6, 1)), np.zeros((6, 1)), 0)
        trajopt.AddPathVelocityConstraint(np.zeros((6, 1)), np.zeros((6, 1)), 1)
        # trajopt.AddPathAccelerationConstraint(np.zeros((6, 1)), np.zeros((6, 1)), 0)
        # trajopt.AddPathAccelerationConstraint(np.zeros((6, 1)), np.zeros((6, 1)), 1)

        evaluate_at_s = np.linspace(collision_begin, collision_end, 25)
        for s in evaluate_at_s:
            trajopt.AddPathPositionConstraint(self.collision_constraint, s)

        prog = trajopt.get_mutable_prog()
        result = Solve(prog)
        if result.is_success():
            return trajopt.ReconstructTrajectory(result)
        else:
            return None


class PipelineServicer(pipeline_pb2_grpc.PipelineServicer):

    def __init__(self, motion_planner: MotionPlanner):
        super().__init__()
        self.motion_planner = motion_planner

    def GetPlan(self, request: pipeline_pb2.GraspRequest, context):
        place_grasp = np.eye(4)
        place_grasp[:3, 3] = [0.11, 0.0, 0.23]

        q_ready = np.array([0, 0, 0, 1.5, 0, np.pi/4])

        pick_grasp = np.array(
            request.grasp.grasp, dtype=np.float32
        ).reshape((4, 4))

        q_start = np.array(request.q_start, dtype=np.float32)
        assert len(q_start) == 6

        def gen_failed_response():
            return pipeline_pb2.PlanTrajectoriesResponse(
                response=pipeline_pb2.Response(
                    id=request.request.id,
                    success=False
                )
            )

        q_place = self.motion_planner.solve_ik(place_grasp, np.pi/8, 0)
        q_pick_before = self.motion_planner.solve_ik(pick_grasp, np.pi/16, -0.05)
        q_pick_after = self.motion_planner.solve_ik(pick_grasp, np.pi/16, 0)
        if any(q is None for q in [q_place, q_pick_before, q_pick_after]):
            logging.warning("IK failed")
            return gen_failed_response()

        pick_traj = self.motion_planner.solve_trajopt(
            q_start, q_pick_after, 0, 2/3, [(q_pick_before, 2/3)]
        )
        place_traj = self.motion_planner.solve_trajopt(
            q_pick_after, q_place, 1/3, 1, [(q_pick_before, 1/3)]
        )
        return_traj = self.motion_planner.solve_trajopt(
            q_place, q_ready, 0, 1
        )
        if any(traj is None for traj in [pick_traj, place_traj, return_traj]):
            logging.warning("TrajOpt failed")
            return gen_failed_response()
        
        logging.info("motion planning succeeded")
        return pipeline_pb2.PlanTrajectoriesResponse(
            response=pipeline_pb2.Response(
                id=request.request.id,
                success=True
            ),
            planTrajectories=pipeline_pb2.PlanTrajectories(
                pick_traj=pipeline_pb2.Trajectory(traj=pickle.dumps(pick_traj)),
                place_traj=pipeline_pb2.Trajectory(traj=pickle.dumps(place_traj)),
                return_traj=pipeline_pb2.Trajectory(traj=pickle.dumps(return_traj))
            )
        )


def serve(motion_planner: MotionPlanner):
    server = grpc.server(ThreadPoolExecutor(max_workers=10))
    pipeline_pb2_grpc.add_PipelineServicer_to_server(
        PipelineServicer(motion_planner), server
    )
    server.add_insecure_port("127.0.0.1:50053")
    server.start()
    server.wait_for_termination()


def main():
    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    )

    try:
        motion_planner = MotionPlanner()
        serve(motion_planner)
    except KeyboardInterrupt:
        logging.info("User keyboard interrupt")
    except Exception as e:
        logging.exception(f"An unexpected error occured: {e}")


if __name__ == "__main__":
    main()
