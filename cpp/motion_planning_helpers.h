#include "helpers.h"
#include "inverse_kinematics.h"
#include "ompl_planning.h"
#include "trajectory_optimization.h"
#include "constants.h"

#include <drake/common/trajectories/bspline_trajectory.h>
#include <drake/planning/robot_diagram_builder.h>
#include <drake/geometry/meshcat.h>
#include <drake/geometry/meshcat_visualizer.h>

#include <Eigen/Dense>

#include <optional>
#include <memory>

using motion_planning::inverse_kinematics::GenerateGoalConfig;
using motion_planning::ompl::GenerateWaypoints;
using motion_planning::trajectory_optimization::GenerateTrajectory;
using motion_planning::inverse_kinematics::GeneratePlaceConfig;

using drake::trajectories::BsplineTrajectory;
using drake::planning::RobotDiagramBuilder;
using drake::geometry::Meshcat;
using drake::geometry::MeshcatVisualizer;

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
    if (!ik_result)
        return std::nullopt;
    const auto q_goal { ik_result.value() };

    // sampling-based motion planning
    auto ompl_result { GenerateWaypoints(plant, diagram, q_goal) };
    if (!ompl_result)
        return std::nullopt;
    const auto waypoints { ompl_result.value() };

    // trajectory optimization
    auto trajopt_result { GenerateTrajectory(plant, diagram, waypoints) };
    if (!trajopt_result)
        return std::nullopt;
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
    if (!ik_result)
        return std::nullopt;
    const auto q_goal { ik_result.value() };

    // sampling-based motion planning
    auto ompl_result { GenerateWaypoints(plant, diagram, q_grasp_closed, q_goal) };
    if (!ompl_result)
        return std::nullopt;
    const auto waypoints { ompl_result.value() };

    // trajectory optimization
    auto trajopt_result { GenerateTrajectory(plant, diagram, waypoints) };
    if (!trajopt_result)
        return std::nullopt;
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
    if (!ompl_result)
        return std::nullopt;
    const auto waypoints { ompl_result.value() };

    // trajectory optimization
    auto trajopt_result { GenerateTrajectory(plant, diagram, waypoints) };
    if (!trajopt_result)
        return std::nullopt;
    return trajopt_result.value();
}

}
