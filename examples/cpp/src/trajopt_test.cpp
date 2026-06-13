#include "helpers.h"
#include "constants.h"

#include <drake/planning/robot_diagram_builder.h>
#include <drake/geometry/meshcat.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/math/bspline_basis.h>
#include <drake/math/matrix_util.h>
#include <drake/common/trajectories/bspline_trajectory.h>
#include <drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h>
#include <drake/planning/collision_checker_params.h>
#include <drake/planning/scene_graph_collision_checker.h>
#include <drake/multibody/inverse_kinematics/minimum_distance_lower_bound_constraint.h>
#include <drake/solvers/solve.h>
#include <drake/solvers/minimum_value_constraint.h>

#include <Eigen/Dense>

#include <iostream>
#include <vector>
#include <memory>

using drake::planning::RobotDiagramBuilder;
using drake::geometry::Meshcat;
using drake::geometry::MeshcatVisualizer;
using drake::math::BsplineBasis;
using drake::math::EigenToStdVector;
using drake::trajectories::BsplineTrajectory;
using drake::planning::trajectory_optimization::KinematicTrajectoryOptimization;
using drake::planning::CollisionCheckerParams;
using drake::planning::SceneGraphCollisionChecker;
using drake::multibody::MinimumDistanceLowerBoundConstraint;
using drake::solvers::MinimumValuePenaltyFunction;
using drake::solvers::Solve;

int main() {
    std::cout << "TrajOpt Test" << '\n';

    // read waypoints
    const Eigen::MatrixXd waypoints { helpers::load_matrix("waypoints.bin") };
    const int n_waypoints { static_cast<int>(waypoints.cols()) };
    std::cout << waypoints.transpose() << '\n';

    // init diagram
    RobotDiagramBuilder<double> builder {};
    auto& plant { builder.plant() };
    auto& scene_graph { builder.scene_graph() };
    auto& parser { builder.parser() };

    // init scenario
    std::cout << "parsing started..." << '\n';
    helpers::generate_so101_brick_welded(plant, scene_graph, parser);
    std::cout << "parsing finished." << '\n';

    // meshcat
    auto meshcat { std::make_shared<Meshcat>() };
    auto& visualizer { MeshcatVisualizer<double>::AddToBuilder(&(builder.builder()), scene_graph, meshcat) };

    // create context
    std::shared_ptr diagram { builder.Build() };
    auto context { diagram->CreateDefaultContext() };
    const auto& fixed_plant_context { plant.GetMyContextFromRoot(*context) };
    auto& mutable_plant_context { plant.GetMyMutableContextFromRoot(context.get()) };

    // init trajopt
    BsplineBasis<double> basis { 4, n_waypoints };
    BsplineTrajectory<double> init_traj { basis, EigenToStdVector<double>(waypoints) };
    KinematicTrajectoryOptimization trajopt { constants::SO101_NUM_Q, n_waypoints, 4 };
    trajopt.SetInitialGuess(init_traj);

    // add kinematic constraints
    std::cout << "configuring trajopt..." << '\n';
    trajopt.AddPositionBounds(
        plant.GetPositionLowerLimits(),
        plant.GetPositionUpperLimits()
    );
    trajopt.AddVelocityBounds(
        plant.GetVelocityLowerLimits(),
        plant.GetVelocityUpperLimits()
    );
    trajopt.AddAccelerationBounds(
        Eigen::VectorXd::Ones(constants::SO101_NUM_Q) * -2,
        Eigen::VectorXd::Ones(constants::SO101_NUM_Q) * 2
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
    std::cout << "trajopt configured." << '\n';

    // solve trajopt
    std::cout << "solving trajopt..." << '\n';
    auto& prog { trajopt.get_mutable_prog() };
    auto result { Solve(prog) };
    std::cout 
        << "trajopt result: "
        << (result.is_success() ? "success" : "failure") 
        << '\n';

    // visualize trajectory
    auto trajectory { trajopt.ReconstructTrajectory(result) };
    helpers::publish_position_trajectory(trajectory, *context, plant, visualizer);
    helpers::user_input_quit();

    return 0;
}
