#pragma once

#include "constants.h"

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/systems/framework/context.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/planning/robot_diagram_builder.h>
#include <drake/geometry/meshcat.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/planning/robot_diagram.h>

#include <Eigen/Dense>

#include <iostream>
#include <memory>

using drake::multibody::MultibodyPlant;
using drake::geometry::SceneGraph;
using drake::multibody::Parser;
using drake::systems::Context;
using drake::geometry::MeshcatVisualizerd;
using drake::planning::RobotDiagramBuilder;
using drake::geometry::Meshcat;
using drake::systems::DiagramBuilder;
using drake::planning::RobotDiagram;

namespace helpers {

namespace detail {

void generate_so101_brick_welded_impl(
    MultibodyPlant<double>& plant,
    SceneGraph<double>& scene_graph,
    Parser& parser,
    const Eigen::VectorXd q_init
);

} // namespace detail

void generate_so101_brick(
    MultibodyPlant<double>& plant,
    SceneGraph<double>& scene_graph,
    Parser& parser,
    DiagramBuilder<double>& diagram_builder
);

inline void generate_so101_brick_welded(
    MultibodyPlant<double>& plant,
    SceneGraph<double>& scene_graph
) {
    Parser parser{ &plant, &scene_graph };
    const Eigen::VectorXd q_init { 
        Eigen::VectorXd::Map(
            constants::SO101_Q_INIT, 
            constants::SO101_NUM_Q
        ) 
    };
    detail::generate_so101_brick_welded_impl(plant, scene_graph, parser, q_init);
}

inline void generate_so101_brick_welded(
    MultibodyPlant<double>& plant,
    SceneGraph<double>& scene_graph,
    Parser& parser
) {
    const Eigen::VectorXd q_init { 
        Eigen::VectorXd::Map(
            constants::SO101_Q_INIT, 
            constants::SO101_NUM_Q
        ) 
    };
    detail::generate_so101_brick_welded_impl(plant, scene_graph, parser, q_init);
}
inline void generate_so101_brick_welded(
    MultibodyPlant<double>& plant,
    SceneGraph<double>& scene_graph,
    Parser& parser,
    const Eigen::VectorXd q_init
) {
    detail::generate_so101_brick_welded_impl(plant, scene_graph, parser, q_init);
}

void generate_so101_binpick_welded(
    MultibodyPlant<double>& plant,
    SceneGraph<double>& scene_graph,
    Parser& parser
);

struct ScenarioAssets {
    std::unique_ptr<RobotDiagramBuilder<double>> builder {};
    MultibodyPlant<double>* plant {};
    std::shared_ptr<Meshcat> meshcat {};
    MeshcatVisualizerd* visualizer {};
    std::shared_ptr<RobotDiagram<double>> diagram {};
    DiagramBuilder<double>* diagram_builder {};
};

ScenarioAssets generate_so101_brick_diagram(
    bool welded,
    bool visualize,
    bool bin_pick = false
);
    
} // namespace helpers
