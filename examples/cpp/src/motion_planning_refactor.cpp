#include "helpers.h"
#include "kinematics/inverse_kinematics.h"
#include "planning/ompl_planning.h"
#include "optimization/trajectory_optimization.h"

#include <drake/planning/robot_diagram_builder.h>
#include <drake/geometry/meshcat.h>
#include <drake/geometry/meshcat_visualizer.h>

#include <iostream>
#include <memory>

using motion_planning::inverse_kinematics::SO101InverseKinematicsPick;
using motion_planning::ompl::SO101OMPL;
using motion_planning::trajectory_optimization::SO101TrajOpt;

using drake::planning::RobotDiagramBuilder;
using drake::geometry::Meshcat;
using drake::geometry::MeshcatVisualizer;

int main() {
    std::cout << "motion planning refactor" << std::endl;

    // init builder
    RobotDiagramBuilder<double> builder {};
    auto& plant { builder.plant() };
    auto& scene_graph { builder.scene_graph() };
    auto& parser { builder.parser() };

    // init scenario
    std::cout << "parsing started..." << std::endl;
    helpers::generate_so101_brick_welded(plant, scene_graph, parser);
    std::cout << "parsing finished." << std::endl;

    // init meshcat
    auto meshcat { std::make_shared<Meshcat>() };
    auto& visualizer { MeshcatVisualizer<double>::AddToBuilder(
        &(builder.builder()), 
        scene_graph, 
        meshcat
    ) };

    // build diagram
    std::shared_ptr diagram { builder.Build() };

    // create context
    auto context { diagram->CreateDefaultContext() };
    const auto& fixed_plant_context { plant.GetMyContextFromRoot(*context) };
    
    // inverse kinematics
    std::cout << "running inverse kinematics..." << std::endl;
    SO101InverseKinematicsPick ik_pick { diagram };
    std::cout << "solving..." << std::endl;
    auto ik_pick_result { ik_pick.solve() };
    if (!ik_pick_result) {
        std::cout << "IK Failure. Exiting..." << std::endl;
        return 1;
    }
    const auto q_pick { ik_pick_result.value() };
    std::cout << "IK Success. Q Pick: " << q_pick.transpose() << std::endl;
    
    // sampling-based motion planning
    std::cout << "running sampling-based motion planning..." << std::endl;
    SO101OMPL sampling_planner { diagram };
    const auto q_start { plant.GetPositions(fixed_plant_context) };
    std::cout << "setting problem definition..." << std::endl;
    sampling_planner.set_pdef(q_start, q_pick);
    std::cout << "solving..." << std::endl;
    auto sampling_planner_result { sampling_planner.solve() };
    if (!sampling_planner_result) {
        std::cout << "Sampling Planner Failure. Exiting..." << std::endl;
        return 1;
    }
    const auto waypoints { sampling_planner_result.value() };
    std::cout << "OMPL Success. Waypoints: " << std::endl;
    std::cout << waypoints.transpose() << std::endl;

    // trajectory optimization
    std::cout << "running trajectory optimization..." << std::endl;
    SO101TrajOpt trajopt { diagram };
    std::cout << "setting waypoints..." << std::endl;
    trajopt.set_waypoints(waypoints);
    std::cout << "solving..." << std::endl;
    auto trajopt_result { trajopt.solve() };
    if (!trajopt_result) {
        std::cout << "TrajOpt Failure. Exiting..." << std::endl;
        return 1;
    }
    const auto trajectory { trajopt_result.value() };
    std::cout << "TrajOpt Success. Visualizing..." << std::endl;
    helpers::publish_position_trajectory(
        trajectory, 
        *context, 
        plant, 
        visualizer
    );
    helpers::user_input_quit();
    
    return 0;
}
