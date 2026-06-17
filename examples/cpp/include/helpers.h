#pragma once

#include "constants.h"

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/common/trajectories/trajectory.h>
#include <drake/systems/framework/context.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/planning/robot_diagram_builder.h>
#include <drake/geometry/meshcat.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/controllers/inverse_dynamics_controller.h>
#include <drake/planning/robot_diagram.h>

#include <Eigen/Dense>

#include <iostream>
#include <fstream>
#include <memory>

using drake::multibody::MultibodyPlant;
using drake::geometry::SceneGraph;
using drake::multibody::Parser;
using drake::math::RigidTransform;
using drake::math::RollPitchYaw;
using drake::trajectories::Trajectory;
using drake::systems::Context;
using drake::geometry::MeshcatVisualizerd;
using drake::planning::RobotDiagramBuilder;
using drake::geometry::Meshcat;
using drake::geometry::MeshcatVisualizer;
using drake::systems::DiagramBuilder;
using drake::systems::controllers::InverseDynamicsController;
using drake::planning::RobotDiagram;

namespace helpers {

inline void user_input_quit() {
    std::cout << "Press Enter to quit..." << std::endl;
    std::string line;
    std::getline(std::cin, line);        
}

inline void save_matrix(
    const Eigen::MatrixXd& matrix, 
    const std::string& filename
) {
    std::ofstream out { filename, std::ios::binary };

    Eigen::Index rows = matrix.rows();
    Eigen::Index cols = matrix.cols();

    out.write(reinterpret_cast<char*>(&rows), sizeof(rows));
    out.write(reinterpret_cast<char*>(&cols), sizeof(cols));

    out.write(
        reinterpret_cast<const char*>(matrix.data()),
        rows * cols * sizeof(double)
    );
}

inline Eigen::MatrixXd load_matrix(
    const std::string& filename
) {
    std::ifstream in { filename, std::ios::binary };

    Eigen::Index rows, cols;

    in.read(reinterpret_cast<char*>(&rows), sizeof(rows));
    in.read(reinterpret_cast<char*>(&cols), sizeof(cols));

    Eigen::MatrixXd matrix(rows, cols);

    in.read(
        reinterpret_cast<char*>(matrix.data()),
        rows * cols * sizeof(double)
    );

    return matrix;        
}

inline void publish_position_trajectory(
    const Trajectory<double>& trajectory,
    Context<double>& root_context,
    const MultibodyPlant<double>& plant,
    MeshcatVisualizerd& visualizer,
    const float time_step = 1.0 / 33.0
) {
    auto& plant_context { plant.GetMyMutableContextFromRoot(&root_context) };
    const auto& visualizer_context { visualizer.GetMyContextFromRoot(root_context) };
    auto so101 { plant.GetModelInstanceByName("so101_new_calib") };

    visualizer.StartRecording(false);

    for (double t = trajectory.start_time(); t <= trajectory.end_time(); t += time_step) {
        root_context.SetTime(t);
        plant.SetPositions(&plant_context, so101, trajectory.value(t));
        visualizer.ForcedPublish(visualizer_context);
    }

    visualizer.StopRecording();
    visualizer.PublishRecording();
}
    
} // namespace helpers
