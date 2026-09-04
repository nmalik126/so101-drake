import numpy as np
from ompl import base as ob
from ompl import geometric as og
from pydrake.all import (
    RobotDiagram,
    ModelInstanceIndex,
    CollisionCheckerParams,
    SceneGraphCollisionChecker,
    MultibodyPlant,
    BsplineBasis,
    BsplineTrajectory,
    KinematicTrajectoryOptimization,
    MinimumDistanceLowerBoundConstraint,
    Solve,
)


class DrakeStateValidityChecker(ob.StateValidityChecker):
    def __init__(
        self, 
        si: ob.SpaceInformation,
        diagram: RobotDiagram,
        robot_model: ModelInstanceIndex,
        num_q: int,
        q_start: np.ndarray
    ):
        super().__init__(si)

        self.collision_checker_params = CollisionCheckerParams()
        self.collision_checker_params.model = diagram
        self.collision_checker_params.robot_model_instances = [robot_model]
        self.collision_checker_params.edge_step_size = 0.01
        self.collision_checker = SceneGraphCollisionChecker(self.collision_checker_params)

        self.context = self.collision_checker.MakeStandaloneModelContext()

        self.num_q = num_q
        self.influence_distance = 1e0
        self.q_start = q_start

    def _stateToVector(self, state: ob.State) -> np.ndarray[np.float64]:
        return np.array([state[i] for i in range(self.num_q)], dtype=np.float64)

    def isValid(self, state):
        q = self._stateToVector(state)
        if np.allclose(q, self.q_start):
            return True
        return self.collision_checker.CheckContextConfigCollisionFree(
            self.context, q
        )


class SO101SamplingPlanner:

    @classmethod
    def generate_path(
        cls,
        diagram: RobotDiagram,
        q_start: np.ndarray,
        q_goal: np.ndarray
    ):
        plant = diagram.plant()
        so101 = plant.GetModelInstanceByName("so101_new_calib")

        num_q = plant.num_positions()
        lower_bounds = plant.GetPositionLowerLimits()
        upper_bounds = plant.GetPositionUpperLimits()

        bounds = ob.RealVectorBounds(num_q)
        for i in range(num_q):
            bounds.setLow(i, lower_bounds[i])
            bounds.setHigh(i, upper_bounds[i])
        bounds.setLow(5, np.pi/4)
        bounds.setHigh(5, np.pi/4)

        space = ob.RealVectorStateSpace(num_q)
        space.setBounds(bounds)

        si = ob.SpaceInformation(space)
        validityChecker = DrakeStateValidityChecker(
            si, diagram, so101, num_q, q_start
        )
        si.setStateValidityChecker(validityChecker)
        si.setup()

        start = space.allocState()
        goal = space.allocState()
        for i in range(num_q):
            start[i] = q_start[i]
            goal[i] = q_goal[i]

        pdef = ob.ProblemDefinition(si)
        pdef.setStartAndGoalStates(start, goal)
    
        planner = og.RRTConnect(si, addIntermediateStates=True)
        planner.setProblemDefinition(pdef)
        planner.setup()

        solved = planner.solve(1.0)
        if solved:
            path = pdef.getSolutionPath()
            num_states = path.getStateCount()
            dim = space.getDimension()

            path_array = np.zeros((num_states, dim))
            for i in range(num_states):
                state = path.getState(i)
                for j in range(dim):
                    path_array[i, j] = state[j]

            waypoints = path_array.T
            return waypoints
        else:
            return None

    @classmethod
    def generate_trajectory(
        cls, 
        plant: MultibodyPlant,
        diagram: RobotDiagram,
        waypoints: np.ndarray,
        avoid_collisions = True
    ) -> BsplineTrajectory | None:
        num_q = plant.num_positions()
        basis = BsplineBasis(4, waypoints.shape[1])
        init_traj = BsplineTrajectory(basis, waypoints)
        trajopt = KinematicTrajectoryOptimization(num_q, waypoints.shape[1], 4)
        trajopt.SetInitialGuess(init_traj)
    
        trajopt.AddDurationCost(1.0)
        trajopt.AddPathLengthCost(1.0)
        trajopt.AddPositionBounds(
            plant.GetPositionLowerLimits(), 
            plant.GetPositionUpperLimits()
        )
        trajopt.AddVelocityBounds(
            0.3 * plant.GetVelocityLowerLimits(),
            0.3 * plant.GetVelocityUpperLimits()
        )
        trajopt.AddAccelerationBounds(
            -2.0 * np.ones(num_q),
                2.0 * np.ones(num_q)
        )
        trajopt.AddJerkBounds(
            -1.0 * np.ones(num_q),
                1.0 * np.ones(num_q)
        )
        trajopt.AddDurationConstraint(0.5, 5.0)
    
        q_start = waypoints[:, 0]
        q_goal = waypoints[:, -1]
        trajopt.AddPathPositionConstraint(lb=q_start, ub=q_start, s=0)
        trajopt.AddPathPositionConstraint(lb=q_goal, ub=q_goal, s=1)
    
        trajopt.AddPathVelocityConstraint(np.zeros((num_q, 1)), np.zeros((num_q, 1)), 0)
        trajopt.AddPathVelocityConstraint(np.zeros((num_q, 1)), np.zeros((num_q, 1)), 1)

        so101 = plant.GetModelInstanceByName("so101_new_calib")
        if avoid_collisions:
            collision_checker_params = CollisionCheckerParams()
            collision_checker_params.model = diagram
            collision_checker_params.robot_model_instances = [so101]
            collision_checker_params.edge_step_size = 0.01
            collision_checker = SceneGraphCollisionChecker(collision_checker_params)
            # collision_checker.SetPaddingAllRobotEnvironmentPairs(1e-3)
            collision_constraint = MinimumDistanceLowerBoundConstraint(
                collision_checker,
                5e-3,
                collision_checker.MakeStandaloneModelContext(),
                None,
                5e-2,
            )
            evaluate_at_s = np.linspace(0, 1, 25)
            for s in evaluate_at_s:
                trajopt.AddPathPositionConstraint(collision_constraint, s)
    
        prog = trajopt.get_mutable_prog()
        result = Solve(prog)
        if result.is_success():
            return trajopt.ReconstructTrajectory(result)
        else:
            return None
