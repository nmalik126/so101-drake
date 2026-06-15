#include "helpers.h"
#include "kinematics/inverse_kinematics.h"

#include <drake/planning/robot_diagram_builder.h>
#include <drake/geometry/meshcat.h>
#include <drake/geometry/meshcat_visualizer.h>

#include <iostream>
#include <memory>

using motion_planning::inverse_kinematics::SO101InverseKinematicsPick;

using drake::planning::RobotDiagramBuilder;
using drake::geometry::Meshcat;
using drake::geometry::MeshcatVisualizer;

int main() {
    std::cout << "motion planning refactor" << '\n';

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
    std::cout << "running inverse kinematics..." << std::endl;
    SO101InverseKinematicsPick ik_pick { diagram };
    std::cout << "solving..." << std::endl;
    auto ik_pick_result { ik_pick.solve() };
    if (!ik_pick_result) {
        std::cout << "IK Failure. Exiting..." << '\n';
        return 1;
    }
    const auto q_pick { ik_pick_result.value() };
    std::cout << "IK Success. Q Pick: " << q_pick.transpose() << std::endl;
    
    return 0;
}
