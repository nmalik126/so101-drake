#include "helpers.h"
#include "inverse_kinematics.h"
#include "ompl_planning.h"
#include "trajectory_optimization.h"

#include <drake/common/trajectories/bspline_trajectory.h>
#include <drake/planning/robot_diagram_builder.h>
#include <drake/geometry/meshcat.h>
#include <drake/geometry/meshcat_visualizer.h>

#include <optional>
#include <memory>

using motion_planning::inverse_kinematics::GenerateGoalConfig;
using motion_planning::ompl::GenerateWaypoints;
using motion_planning::trajectory_optimization::GenerateTrajectory;

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

}
