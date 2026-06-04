#include "helpers.h"

#include <drake/planning/robot_diagram_builder.h>
#include <drake/geometry/meshcat.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/geometry/meshcat_visualizer_params.h>

#include <iostream>

using drake::planning::RobotDiagramBuilder;
using drake::geometry::Meshcat;
using drake::geometry::MeshcatVisualizer;
using drake::geometry::MeshcatVisualizerParams;

int main() {
    std::cout << "OMPL Test" << '\n';

    // init diagram
    RobotDiagramBuilder<double> builder {};
    auto& plant { builder.plant() };
    auto& scene_graph { builder.scene_graph() };
    auto& parser { builder.parser() };

    // init scenario
    std::cout << "parsing started..." << '\n';
    helpers::generate_so101_brick_welded(plant, scene_graph, parser);
    std::cout << "parsing finished." << '\n';

    // init visualizers
    auto meshcat { std::make_shared<Meshcat>() };
    auto& visualizer {
        MeshcatVisualizer<double>::AddToBuilder(
            &(builder.builder()),
            scene_graph,
            meshcat,
            MeshcatVisualizerParams {
                .role { drake::geometry::Role::kIllustration }
            }
        )
    };
    auto& collision_visualizer {
        MeshcatVisualizer<double>::AddToBuilder(
            &(builder.builder()),
            scene_graph,
            meshcat,
            MeshcatVisualizerParams {
                .role { drake::geometry::Role::kIllustration },
                .prefix { "collision" },
                .visible_by_default { false }
            }
        )
    };

    // create context
    auto diagram { builder.Build() };
    auto context { diagram->CreateDefaultContext() };
    const auto& fixed_plant_context { plant.GetMyContextFromRoot(*context) };
    auto q_start { plant.GetPositions(fixed_plant_context) };
    const Eigen::VectorXd q_goal { { -0.257417, 0.164283, 0.253324, 1.04113, 1.3605, 0.574355 } };
    std::cout << "q_start: \n" << q_start << '\n';
    std::cout << "q_goal: \n" << q_goal << '\n';

    return 0;
}
