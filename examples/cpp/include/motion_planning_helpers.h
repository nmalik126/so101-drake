#pragma once

#include "helpers.h"
#include "scenario_helpers.h"
#include "kinematics/inverse_kinematics.h"
#include "planning/ompl_planning.h"
#include "optimization/trajectory_optimization.h"
#include "constants.h"

#include <drake/common/trajectories/trajectory.h>
#include <drake/common/trajectories/bspline_trajectory.h>
#include <drake/common/trajectories/composite_trajectory.h>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/common/copyable_unique_ptr.h>
#include <drake/planning/scene_graph_collision_checker.h>
#include <drake/planning/collision_checker_params.h>
#include <drake/math/rigid_transform.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/basic_vector.h>

#include <Eigen/Dense>

#include <optional>
#include <memory>

using motion_planning::inverse_kinematics::SO101InverseKinematics;
using motion_planning::inverse_kinematics::SO101InverseKinematicsPick;
using motion_planning::inverse_kinematics::SO101InverseKinematicsPlace;
using motion_planning::ompl::SO101OMPL;
using motion_planning::trajectory_optimization::SO101TrajOpt;

using drake::trajectories::Trajectory;
using drake::trajectories::BsplineTrajectory;
using drake::trajectories::CompositeTrajectory;
using drake::trajectories::PiecewisePolynomial;
using drake::planning::SceneGraphCollisionChecker;
using drake::planning::CollisionCheckerParams;
using drake::math::RigidTransformd;
using drake::systems::LeafSystem;
using drake::systems::Context;
using drake::systems::BasicVector;

namespace motion_planning {

namespace detail {

std::optional<BsplineTrajectory<double>> GenerateMotionPlanImpl(
    SO101OMPL& sampling_planner,
    SO101TrajOpt& trajopt,
    const Eigen::VectorXd q_start,
    const Eigen::VectorXd q_goal,
    std::shared_ptr<SceneGraphCollisionChecker> checker
);

} // namespace detail

std::optional<BsplineTrajectory<double>> GenerateMotionPlan(
    SO101InverseKinematics& ik,
    SO101OMPL& sampling_planner,
    SO101TrajOpt& trajopt,
    const Eigen::VectorXd q_start,
    std::shared_ptr<SceneGraphCollisionChecker> checker
);

inline std::optional<BsplineTrajectory<double>> GenerateMotionPlan(
    SO101OMPL& sampling_planner,
    SO101TrajOpt& trajopt,
    const Eigen::VectorXd q_start,
    const Eigen::VectorXd q_goal,
    std::shared_ptr<SceneGraphCollisionChecker> checker
) {
    return detail::GenerateMotionPlanImpl(
        sampling_planner, trajopt, q_start, q_goal, checker
    );
}

std::optional<CompositeTrajectory<double>> GeneratePickPlaceMotionPlans();

std::optional<CompositeTrajectory<double>> GenerateBinpickMotionPlans(
    const RigidTransformd grasp_transform,
    const MultibodyPlant<double>& plan_plant,
    std::shared_ptr<RobotDiagram<double>> plan_diagram,
    std::shared_ptr<SceneGraphCollisionChecker> checker,
    const Eigen::VectorXd q_start,
    bool return_home = true
);

class MutableTrajectorySource final
    : public drake::systems::LeafSystem<double> {
public:
    explicit MutableTrajectorySource(int num_positions);

    void SetTrajectory(
        std::unique_ptr<Trajectory<double>> trajectory,
        double simulator_time);

private:
    void CalcOutput(
        const Context<double>& context,
        BasicVector<double>* output) const;

    const int num_positions_;

    std::unique_ptr<Trajectory<double>> trajectory_;

    double trajectory_start_time_{0.0};
};

} // namespace motion_planning
