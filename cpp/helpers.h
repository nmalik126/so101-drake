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

namespace helpers {

    namespace detail {

        template<typename T>
        void generate_so101_brick_welded_impl(
            MultibodyPlant<T>& plant,
            SceneGraph<T>& scene_graph,
            Parser& parser
        ) {
            // add mat
            auto mat = parser.AddModels(constants::model_paths::MAT)[0];
            plant.WeldFrames(
                plant.world_frame(),
                plant.GetFrameByName("mat_link", mat)
            );

            // add so101
            auto so101 = parser.AddModels(constants::model_paths::SO101)[0];
            const drake::Vector3<T> T_mat_so101 { constants::transforms::X_MAT_SO101::T };
            const drake::Vector3<T> rpy_mat_so101 { constants::transforms::X_MAT_SO101::R };
            const RollPitchYaw R_mat_so101 { rpy_mat_so101 };
            const RigidTransform X_mat_so101 { R_mat_so101, T_mat_so101 };
            plant.WeldFrames(
                plant.GetFrameByName("mat_link", mat),
                plant.GetFrameByName("base_link", so101),
                X_mat_so101
            );
            
            // add box
            auto box = parser.AddModels(constants::model_paths::BOX)[0];
            const drake::Vector3<T> T_mat_box { constants::transforms::X_MAT_BOX::T };
            const drake::Vector3<T> rpy_mat_box { constants::transforms::X_MAT_BOX::R };
            const RollPitchYaw R_mat_box { rpy_mat_box };
            const RigidTransform X_mat_box { R_mat_box, T_mat_box };
            plant.WeldFrames(
                plant.GetFrameByName("mat_link", mat),
                plant.GetFrameByName("box_link", box),
                X_mat_box
            );            

            // finalize plant
            plant.Finalize();

            // set so101 default positions
            const Eigen::VectorXd q_init { 
                Eigen::VectorXd::Map(
                    constants::SO101_Q_INIT, 
                    constants::SO101_NUM_Q
                ) 
            };
            plant.SetDefaultPositions(so101, q_init);
        }

    } // namespace detail

    template<typename T>
    void generate_so101_brick_welded(
        MultibodyPlant<T>& plant,
        SceneGraph<T>& scene_graph
    ) {
        Parser parser{ &plant, &scene_graph };
        detail::generate_so101_brick_welded_impl(plant, scene_graph, parser);
    }

    template<typename T>
    void generate_so101_brick_welded(
        MultibodyPlant<T>& plant,
        SceneGraph<T>& scene_graph,
        Parser& parser
    ) {
        detail::generate_so101_brick_welded_impl(plant, scene_graph, parser);
    }

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

        visualizer.StartRecording(false);

        for (double t = trajectory.start_time(); t <= trajectory.end_time(); t += time_step) {
            root_context.SetTime(t);
            plant.SetPositions(&plant_context, trajectory.value(t));
            visualizer.ForcedPublish(visualizer_context);
        }

        visualizer.StopRecording();
        visualizer.PublishRecording();
    }
    
} // namespace helpers
