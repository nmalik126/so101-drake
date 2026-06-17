#pragma once

#include "helpers.h"
#include "scenario_helpers.h"
#include "kinematics/inverse_kinematics.h"
#include "planning/ompl_planning.h"
#include "optimization/trajectory_optimization.h"
#include "constants.h"

#include <drake/common/trajectories/bspline_trajectory.h>
#include <drake/common/trajectories/composite_trajectory.h>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/common/copyable_unique_ptr.h>
#include <drake/planning/scene_graph_collision_checker.h>
#include <drake/planning/collision_checker_params.h>
#include <drake/math/rigid_transform.h>

#include <Eigen/Dense>

#include <optional>
#include <memory>

using motion_planning::inverse_kinematics::SO101InverseKinematics;
using motion_planning::inverse_kinematics::SO101InverseKinematicsPick;
using motion_planning::inverse_kinematics::SO101InverseKinematicsPlace;
using motion_planning::ompl::SO101OMPL;
using motion_planning::trajectory_optimization::SO101TrajOpt;

using drake::trajectories::BsplineTrajectory;
using drake::trajectories::CompositeTrajectory;
using drake::trajectories::PiecewisePolynomial;
using drake::planning::SceneGraphCollisionChecker;
using drake::planning::CollisionCheckerParams;
using drake::math::RigidTransformd;

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

} // namespace motion_planning
