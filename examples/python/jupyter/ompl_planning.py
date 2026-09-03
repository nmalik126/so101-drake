import numpy as np
from ompl import base as ob
from ompl import geometric as og
from pydrake.all import (
    RobotDiagram,
    ModelInstanceIndex,
    CollisionCheckerParams,
    SceneGraphCollisionChecker
)


class DrakeStateValidityChecker(ob.StateValidityChecker):
    def __init__(
        self, 
        si: ob.SpaceInformation,
        diagram: RobotDiagram,
        robot_model: ModelInstanceIndex,
        num_q: int,
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

    def _stateToVector(self, state: ob.State) -> np.ndarray[np.float64]:
        return np.array([state[i] for i in range(self.num_q)], dtype=np.float64)

    def isValid(self, state):
        q = self._stateToVector(state)
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
        print(num_q)

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
            si, diagram, so101, num_q
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
