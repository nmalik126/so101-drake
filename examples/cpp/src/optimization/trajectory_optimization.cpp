#include "optimization/trajectory_optimization.h"

#include "constants.h"

#include <drake/common/trajectories/bspline_trajectory.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/planning/robot_diagram.h>
#include <drake/math/bspline_basis.h>
#include <drake/math/matrix_util.h>
#include <drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h>
#include <drake/planning/collision_checker_params.h>
#include <drake/planning/scene_graph_collision_checker.h>
#include <drake/multibody/inverse_kinematics/minimum_distance_lower_bound_constraint.h>
#include <drake/solvers/solve.h>
#include <drake/solvers/minimum_value_constraint.h>
#include <drake/planning/collision_checker_context.h>

#include <Eigen/Dense>

#include <optional>
#include <memory>

using drake::trajectories::BsplineTrajectory;
using drake::multibody::MultibodyPlant;
using drake::planning::RobotDiagram;
using drake::math::BsplineBasis;
using drake::math::EigenToStdVector;
using drake::planning::trajectory_optimization::KinematicTrajectoryOptimization;
using drake::planning::CollisionCheckerParams;
using drake::planning::SceneGraphCollisionChecker;
using drake::multibody::MinimumDistanceLowerBoundConstraint;
using drake::solvers::MinimumValuePenaltyFunction;
using drake::solvers::Solve;
using drake::planning::CollisionCheckerContext;

namespace motion_planning {
namespace trajectory_optimization {

std::optional<BsplineTrajectory<double>> GenerateTrajectory(
    const MultibodyPlant<double>& plant,
    std::shared_ptr<RobotDiagram<double>> diagram,
    const Eigen::MatrixXd waypoints
) {
    // get n_waypoints
    const int n_waypoints { static_cast<int>(waypoints.cols()) };

    // init trajopt
    BsplineBasis<double> basis { 4, n_waypoints };
    BsplineTrajectory<double> init_traj { basis, EigenToStdVector<double>(waypoints) };
    KinematicTrajectoryOptimization trajopt { constants::SO101_NUM_Q, n_waypoints, 4 };
    trajopt.SetInitialGuess(init_traj);

    // add kinematic constraints
    trajopt.AddPositionBounds(
        plant.GetPositionLowerLimits(),
        plant.GetPositionUpperLimits()
    );
    trajopt.AddVelocityBounds(
        0.15 * plant.GetVelocityLowerLimits(),
        0.15 * plant.GetVelocityUpperLimits()
    );
    trajopt.AddAccelerationBounds(
        Eigen::VectorXd::Ones(constants::SO101_NUM_Q) * -1,
        Eigen::VectorXd::Ones(constants::SO101_NUM_Q) * 1
    );

    // add duration constraints
    trajopt.AddDurationCost(1e0);
    trajopt.AddDurationConstraint(0.5, 5.0);

    // add start/goal constraints
    const auto q_start { waypoints.col(0) };
    const auto q_goal { waypoints.col(n_waypoints - 1) };
    trajopt.AddPathPositionConstraint(q_start, q_start, 0);
    trajopt.AddPathPositionConstraint(q_goal, q_goal, 1);
    const auto zero_velocity { Eigen::VectorXd::Zero(constants::SO101_NUM_Q) };
    trajopt.AddPathVelocityConstraint(zero_velocity, zero_velocity, 0);
    trajopt.AddPathVelocityConstraint(zero_velocity, zero_velocity, 1);

    // add collision avoidance constraint
    auto so101 { plant.GetModelInstanceByName("so101_new_calib") };
    SceneGraphCollisionChecker checker { CollisionCheckerParams {
        .model { diagram },
        .robot_model_instances { { so101 } },
        .edge_step_size { 0.01 }
    } };
    checker.SetPaddingAllRobotEnvironmentPairs(1e-3);
    auto model_context { checker.MakeStandaloneModelContext() };
    auto collision_constraint {
        std::make_shared<MinimumDistanceLowerBoundConstraint>(
            &checker,
            1e-3,
            model_context.get(),
            MinimumValuePenaltyFunction {},
            1e-2
        )
    };
    for (const auto s : Eigen::VectorXd::LinSpaced(25, 0.0, 1.0))
        trajopt.AddPathPositionConstraint(collision_constraint, s);

    // solve trajopt
    auto& prog { trajopt.get_mutable_prog() };
    auto result { Solve(prog) };
    if (result.is_success())
        return trajopt.ReconstructTrajectory(result);
    else
        return std::nullopt;
}

SO101TrajOpt::SO101TrajOpt(
    std::shared_ptr<RobotDiagram<double>> diagram,
    std::shared_ptr<SceneGraphCollisionChecker> checker
) : diagram_ { diagram }
  , checker_ { checker }
  , context_ { checker->MakeStandaloneModelContext() }
{}

void SO101TrajOpt::set_waypoints(const Eigen::MatrixXd waypoints) {
    // init trajopt
    trajopt_ = std::make_unique<KinematicTrajectoryOptimization>(
        constants::SO101_NUM_Q,
        n_desired_waypoints_,
        bspline_basis_order_
    );

    const auto& plant { diagram_->plant() };

    // add kinematic constraints
    trajopt_->AddPositionBounds(
        plant.GetPositionLowerLimits(),
        plant.GetPositionUpperLimits()
    );
    trajopt_->AddVelocityBounds(
        0.15 * plant.GetVelocityLowerLimits(),
        0.15 * plant.GetVelocityUpperLimits()
    );
    trajopt_->AddAccelerationBounds(
        Eigen::VectorXd::Ones(constants::SO101_NUM_Q) * -1,
        Eigen::VectorXd::Ones(constants::SO101_NUM_Q) * 1
    );

    // add duration constraints
    trajopt_->AddDurationCost(1e0);
    trajopt_->AddDurationConstraint(0.5, 5.0);

    // add start/goal constraints
    const auto zero_velocity { Eigen::VectorXd::Zero(constants::SO101_NUM_Q) };
    trajopt_->AddPathVelocityConstraint(zero_velocity, zero_velocity, 0);
    trajopt_->AddPathVelocityConstraint(zero_velocity, zero_velocity, 1);

    // add collision avoidance constraint
    auto collision_constraint {
        std::make_shared<MinimumDistanceLowerBoundConstraint>(
            checker_.get(),
            1e-3,
            context_.get(),
            MinimumValuePenaltyFunction {},
            1e-2
        )
    };
    for (const auto s : Eigen::VectorXd::LinSpaced(25, 0.0, 1.0))
        trajopt_->AddPathPositionConstraint(collision_constraint, s);
        
    // set initial trajectory
    BsplineBasis<double> basis { bspline_basis_order_, n_desired_waypoints_ };
    BsplineTrajectory<double> init_traj {
        basis, EigenToStdVector<double>(waypoints)
    };
    trajopt_->SetInitialGuess(init_traj);

    // add start/goal constraints
    const auto q_start { waypoints.col(0) };
    const auto q_goal { waypoints.col(n_desired_waypoints_ - 1) };
    trajopt_->AddPathPositionConstraint(q_start, q_start, 0);
    trajopt_->AddPathPositionConstraint(q_goal, q_goal, 1);
}

std::optional<BsplineTrajectory<double>> SO101TrajOpt::solve() {
    auto& prog { trajopt_->get_mutable_prog() };
    auto result { Solve(prog) };
    if (result.is_success())
        return trajopt_->ReconstructTrajectory(result);
    else
        return std::nullopt;
}

} // namespace trajectory_optimization
} // namespace motion_planning
