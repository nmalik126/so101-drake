#include "helpers.h"
#include "kinematics/inverse_kinematics.h"
#include "planning/ompl_planning.h"
#include "optimization/trajectory_optimization.h"
#include "constants.h"

#include <drake/common/trajectories/bspline_trajectory.h>
#include <drake/planning/robot_diagram_builder.h>
#include <drake/geometry/meshcat.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/common/trajectories/composite_trajectory.h>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/common/copyable_unique_ptr.h>
#include <drake/planning/scene_graph_collision_checker.h>

#include <Eigen/Dense>

#include <optional>
#include <memory>

using motion_planning::inverse_kinematics::GenerateGoalConfig;
using motion_planning::ompl::GenerateWaypoints;
using motion_planning::trajectory_optimization::GenerateTrajectory;
using motion_planning::inverse_kinematics::GeneratePlaceConfig;
using motion_planning::inverse_kinematics::SO101InverseKinematics;
using motion_planning::ompl::SO101OMPL;
using motion_planning::trajectory_optimization::SO101TrajOpt;

using drake::trajectories::BsplineTrajectory;
using drake::planning::RobotDiagramBuilder;
using drake::geometry::Meshcat;
using drake::geometry::MeshcatVisualizer;
using drake::trajectories::CompositeTrajectory;
using drake::trajectories::PiecewisePolynomial;
using drake::planning::SceneGraphCollisionChecker;

namespace motion_planning {

inline std::optional<BsplineTrajectory<double>> ComputeGraspPlan() {
    // init builder
    RobotDiagramBuilder<double> builder {};
    auto& plant { builder.plant() };
    auto& scene_graph { builder.scene_graph() };
    auto& parser { builder.parser() };

    // init scenario
    helpers::generate_so101_brick_welded(plant, scene_graph, parser);

    // build diagram
    std::shared_ptr diagram { builder.Build() };

    // inverse kinematics
    auto ik_result { GenerateGoalConfig(plant, diagram) };
    if (!ik_result) {
        std::cout << "IK Failure" << '\n';
        return std::nullopt;
    }
    const auto q_goal { ik_result.value() };

    // sampling-based motion planning
    auto ompl_result { GenerateWaypoints(plant, diagram, q_goal) };
    if (!ompl_result) {
        std::cout << "OMPL Failure" << '\n';
        return std::nullopt;
    }
    const auto waypoints { ompl_result.value() };

    // trajectory optimization
    auto trajopt_result { GenerateTrajectory(plant, diagram, waypoints) };
    if (!trajopt_result) {
        std::cout << "TrajOpt Failure" << '\n';
        return std::nullopt;
    }
    return trajopt_result.value();
}

inline std::optional<BsplineTrajectory<double>> ComputePlacePlan(
    const Eigen::VectorXd q_grasp_closed
) {
    // init builder
    RobotDiagramBuilder<double> builder {};
    auto& plant { builder.plant() };
    auto& scene_graph { builder.scene_graph() };
    auto& parser { builder.parser() };

    // init scenario
    helpers::generate_so101_place(plant, scene_graph, parser, q_grasp_closed);

    // build diagram
    std::shared_ptr diagram { builder.Build() };

    // inverse kinematics
    auto ik_result { GeneratePlaceConfig(plant, diagram, q_grasp_closed) };
    if (!ik_result) {
        std::cout << "IK Failure" << '\n';
        return std::nullopt;
    }
    const auto q_goal { ik_result.value() };

    // sampling-based motion planning
    auto ompl_result { GenerateWaypoints(plant, diagram, q_grasp_closed, q_goal) };
    if (!ompl_result) {
        std::cout << "OMPL Failure" << '\n';
        return std::nullopt;
    }
    const auto waypoints { ompl_result.value() };

    // trajectory optimization
    auto trajopt_result { GenerateTrajectory(plant, diagram, waypoints) };
    if (!trajopt_result) {
        std::cout << "TrajOpt Failure" << '\n';
        return std::nullopt;
    }
    return trajopt_result.value();
}

inline std::optional<BsplineTrajectory<double>> ComputeRestPlan(
    const Eigen::VectorXd q_place_open
) {
    // init builder
    RobotDiagramBuilder<double> builder {};
    auto& plant { builder.plant() };
    auto& scene_graph { builder.scene_graph() };
    auto& parser { builder.parser() };

    // init scenario
    helpers::generate_so101_brick_welded(plant, scene_graph, parser, q_place_open);

    // build diagram
    std::shared_ptr diagram { builder.Build() };

    // sampling-based motion planning
    const Eigen::VectorXd q_init { 
        Eigen::VectorXd::Map(
            constants::SO101_Q_INIT, 
            constants::SO101_NUM_Q
        ) 
    };
    auto ompl_result { GenerateWaypoints(plant, diagram, q_place_open, q_init) };
    if (!ompl_result) {
        std::cout << "OMPL Failure" << '\n';
        return std::nullopt;
    }
    const auto waypoints { ompl_result.value() };

    // trajectory optimization
    auto trajopt_result { GenerateTrajectory(plant, diagram, waypoints) };
    if (!trajopt_result) {
        std::cout << "TrajOpt Failure" << '\n';
        return std::nullopt;
    }
    return trajopt_result.value();
}

inline std::optional<CompositeTrajectory<double>> ComputePickPlaceTrajectory() {
    // init builder
    RobotDiagramBuilder<double> builder {};
    auto& plant { builder.plant() };
    auto& scene_graph { builder.scene_graph() };
    auto& parser { builder.parser() };
    auto& diagram_builder { builder.builder() };

    // init non-welded simulation
    std::cout << "initializing scene..." << '\n';
    helpers::generate_so101_brick(plant, scene_graph, parser, diagram_builder);
    std::cout << "scene initialized." << '\n';

    // compute grasp plan
    std::cout << "computing grasp plan..." << '\n';
    const auto grasp_plan_result { ComputeGraspPlan() };
    if (!grasp_plan_result) {
        std::cout << "Grasp Plan Failure. Exiting..." << '\n';
        return std::nullopt;
    }
    std::cout << "grasp plan computed." << '\n';

    // compute place plan
    const auto grasp_trajectory { grasp_plan_result.value() };
    const Eigen::VectorXd q_grasp_open { grasp_trajectory.FinalValue() };
    Eigen::VectorXd q_grasp_closed { q_grasp_open };
    q_grasp_closed(constants::SO101_NUM_Q - 1) = -0.1;
    std::cout << "computing place plan..." << '\n';
    const auto place_plan_result { ComputePlacePlan(q_grasp_closed) };
    if (!place_plan_result) {
        std::cout << "Place Plan Failure. Exiting..." << '\n';
        return std::nullopt;
    }
    std::cout << "place plan computed." << '\n';

    // compute rest plan
    const auto place_trajectory { place_plan_result.value() };
    const Eigen::VectorXd q_place_closed { place_trajectory.FinalValue() };
    Eigen::VectorXd q_place_open { q_place_closed };
    q_place_open(constants::SO101_NUM_Q - 1) = 0.5;
    std::cout << "computing rest plan..." << '\n';
    const auto rest_plan_result { ComputeRestPlan(q_place_open) };
    if (!rest_plan_result) {
        std::cout << "Rest Plan Failure. Exiting..." << '\n';
        return std::nullopt;
    }
    std::cout << "rest plan computed." << '\n';

    // concat trajectories
    const auto gripper_close_trajectory {
        PiecewisePolynomial<double>::FirstOrderHold(
            { 0.0, 1.0 },
            { q_grasp_open, q_grasp_closed }
        )
    };
    const auto gripper_open_trajectory {
        PiecewisePolynomial<double>::FirstOrderHold(
            { 0.0, 1.0 },
            { q_place_closed, q_place_open }
        )
    };
    const auto rest_trajectory { rest_plan_result.value() };
    return CompositeTrajectory<double>::AlignAndConcatenate({
        drake::copyable_unique_ptr<Trajectory<double>> { grasp_trajectory }, 
        drake::copyable_unique_ptr<Trajectory<double>> { gripper_close_trajectory },
        drake::copyable_unique_ptr<Trajectory<double>> { place_trajectory }, 
        drake::copyable_unique_ptr<Trajectory<double>> { gripper_open_trajectory },
        drake::copyable_unique_ptr<Trajectory<double>> { rest_trajectory }
    });
}

namespace detail {

inline std::optional<BsplineTrajectory<double>> GenerateMotionPlanImpl(
    SO101OMPL& sampling_planner,
    SO101TrajOpt& trajopt,
    const Eigen::VectorXd q_start,
    const Eigen::VectorXd q_goal,
    std::shared_ptr<SceneGraphCollisionChecker> checker
) {
    // run sampling planner
    std::cout << "running sampling planner..." << std::endl;
    checker->SetPaddingAllRobotEnvironmentPairs(8e-3);
    sampling_planner.set_pdef(q_start, q_goal);
    auto sampling_planner_result { sampling_planner.solve() };
    if (!sampling_planner_result) {
        std::cout << "Sampling Planner Failure. Exiting..." << std::endl;
        return std::nullopt;
    }
    const auto waypoints { sampling_planner_result.value() };
    std::cout << "sampling planner success, waypoints:" << std::endl;
    std::cout << waypoints.transpose() << std::endl;

    // run trajopt
    std::cout << "running trajectory optimization..." << std::endl;
    checker->SetPaddingAllRobotEnvironmentPairs(1e-3);
    trajopt.set_waypoints(waypoints);
    auto trajopt_result { trajopt.solve() };
    if (!trajopt_result) {
        std::cout << "TrajOpt Failure. Exiting..." << std::endl;
        return std::nullopt;
    }
    return trajopt_result.value();
}

} // namespace detail

inline std::optional<BsplineTrajectory<double>> GenerateMotionPlan(
    SO101InverseKinematics& ik,
    SO101OMPL& sampling_planner,
    SO101TrajOpt& trajopt,
    const Eigen::VectorXd q_start,
    std::shared_ptr<SceneGraphCollisionChecker> checker
) {
    // inverse kinematics
    std::cout << "solving inverse kinematics..." << std::endl;
    ik.set_initial_guess(q_start);
    auto ik_result { ik.solve() };
    if (!ik_result) {
        std::cout << "IK Failure. Exiting..." << std::endl;
        return std::nullopt;
    }
    const auto q_goal { ik_result.value() };
    std::cout << "IK Success. Q Goal: " << q_goal.transpose() << std::endl;
    
    return detail::GenerateMotionPlanImpl(
        sampling_planner, trajopt, q_start, q_goal, checker
    );
}

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

} // namespace motion_planning
