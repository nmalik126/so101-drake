#include "helpers.h"
#include "inverse_kinematics.h"
#include "ompl_planning.h"
#include "trajectory_optimization.h"

#include <drake/planning/robot_diagram_builder.h>
#include <drake/geometry/meshcat.h>
#include <drake/geometry/meshcat_visualizer.h>

#include <iostream>
#include <memory>

using motion_planning::inverse_kinematics::GenerateGoalConfig;
using motion_planning::ompl::GenerateWaypoints;
using motion_planning::trajectory_optimization::GenerateTrajectory;

using drake::planning::RobotDiagramBuilder;
using drake::geometry::Meshcat;
using drake::geometry::MeshcatVisualizer;

int main() {
    std::cout << "Classic Motion Planning" << '\n';

    // init builder
    RobotDiagramBuilder<double> builder {};
    auto& plant { builder.plant() };
    auto& scene_graph { builder.scene_graph() };
    auto& parser { builder.parser() };

    // init scenario
    std::cout << "parsing started..." << '\n';
    helpers::generate_so101_brick_welded(plant, scene_graph, parser);
    std::cout << "parsing finished." << '\n';

    // init meshcat
    auto meshcat { std::make_shared<Meshcat>() };
    auto& visualizer { MeshcatVisualizer<double>::AddToBuilder(
        &(builder.builder()), 
        scene_graph, 
        meshcat
    ) };

    // build diagram
    std::shared_ptr diagram { builder.Build() };

    // inverse kinematics
    std::cout << "running inverse kinematics..." << '\n';
    auto ik_result { GenerateGoalConfig(plant, diagram) };
    if (!ik_result) {
        std::cout << "IK Failure. Exiting..." << '\n';
        return 1;
    }
    const auto q_goal { ik_result.value() };
    std::cout << "IK Success. Q Goal: " << q_goal.transpose() << '\n';

    // sampling-based motion planning
    std::cout << "running sampling-based motion planning..." << '\n';
    auto ompl_result { GenerateWaypoints(plant, diagram, q_goal) };
    if (!ompl_result) {
        std::cout << "OMPL Failure. Exiting..." << '\n';
        return 1;
    }
    const auto waypoints { ompl_result.value() };
    std::cout << "OMPL Success. Waypoints: " << '\n';
    std::cout << waypoints.transpose() << '\n';

    // trajectory optimization
    std::cout << "running trajectory optimization..." << '\n';
    auto trajopt_result { GenerateTrajectory(plant, diagram, waypoints) };
    if (!trajopt_result) {
        std::cout << "TrajOpt Failure. Exiting..." << '\n';
        return 1;
    }
    const auto trajectory { trajopt_result.value() };
    auto context { diagram->CreateDefaultContext() };
    std::cout << "TrajOpt Success. Visualizing..." << '\n';
    helpers::publish_position_trajectory(
        trajectory, 
        *context, 
        plant, 
        visualizer
    );
    helpers::user_input_quit();

    return 0;
}
