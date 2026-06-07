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

namespace helpers {

    namespace detail {

        template<typename T>
        void generate_so101_brick_welded_impl(
            MultibodyPlant<T>& plant,
            SceneGraph<T>& scene_graph,
            Parser& parser,
            const Eigen::VectorXd q_init
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

            // add bin
            auto bin = parser.AddModels(constants::model_paths::BIN_CLR)[0];
            const drake::Vector3<T> T_mat_bin { constants::transforms::X_MAT_BIN::T };
            const drake::Vector3<T> rpy_mat_bin { constants::transforms::X_MAT_BIN::R };
            const RollPitchYaw R_mat_bin { rpy_mat_bin };
            const RigidTransform X_mat_bin { R_mat_bin, T_mat_bin };
            plant.WeldFrames(
                plant.GetFrameByName("mat_link", mat),
                plant.GetFrameByName("bin_link", bin),
                X_mat_bin
            );

            // finalize plant
            plant.Finalize();

            // set so101 default positions
            plant.SetDefaultPositions(so101, q_init);
        }

    } // namespace detail

    inline void generate_so101_brick(
        MultibodyPlant<double>& plant,
        SceneGraph<double>& scene_graph,
        Parser& parser,
        DiagramBuilder<double>& diagram_builder
    ) {
        // add control plant
        auto control_plant {
            std::make_unique<MultibodyPlant<double>>(plant.time_step())
        };
        Parser ctrl_parser { control_plant.get() };

        // add mat
        auto mat = parser.AddModels(constants::model_paths::MAT)[0];
        auto ctrl_mat = ctrl_parser.AddModels(constants::model_paths::MAT)[0];
        plant.WeldFrames(
            plant.world_frame(),
            plant.GetFrameByName("mat_link", mat)
        );
        control_plant->WeldFrames(
            control_plant->world_frame(),
            control_plant->GetFrameByName("mat_link", ctrl_mat)
        );

        // add so101
        auto so101 = parser.AddModels(constants::model_paths::SO101)[0];
        auto ctrl_so101 = ctrl_parser.AddModels(constants::model_paths::SO101)[0];
        const drake::Vector3<double> T_mat_so101 { constants::transforms::X_MAT_SO101::T };
        const drake::Vector3<double> rpy_mat_so101 { constants::transforms::X_MAT_SO101::R };
        const RollPitchYaw R_mat_so101 { rpy_mat_so101 };
        const RigidTransform X_mat_so101 { R_mat_so101, T_mat_so101 };
        plant.WeldFrames(
            plant.GetFrameByName("mat_link", mat),
            plant.GetFrameByName("base_link", so101),
            X_mat_so101
        );
        control_plant->WeldFrames(
            control_plant->GetFrameByName("mat_link", ctrl_mat),
            control_plant->GetFrameByName("base_link", ctrl_so101),
            X_mat_so101
        );

        // add box
        auto box = parser.AddModels(constants::model_paths::BOX)[0];

        // add bin
        auto bin = parser.AddModels(constants::model_paths::BIN)[0];
        const drake::Vector3<double> T_mat_bin { constants::transforms::X_MAT_BIN::T };
        const drake::Vector3<double> rpy_mat_bin { constants::transforms::X_MAT_BIN::R };
        const RollPitchYaw R_mat_bin { rpy_mat_bin };
        const RigidTransform X_mat_bin { R_mat_bin, T_mat_bin };
        plant.WeldFrames(
            plant.GetFrameByName("mat_link", mat),
            plant.GetFrameByName("bin_link", bin),
            X_mat_bin
        );

        // finalize plants
        plant.Finalize();
        control_plant->Finalize();

        // set so101 default positions
        const Eigen::VectorXd q_init { 
            Eigen::VectorXd::Map(
                constants::SO101_Q_INIT, 
                constants::SO101_NUM_Q
            ) 
        };
        plant.SetDefaultPositions(so101, q_init);
        control_plant->SetDefaultPositions(ctrl_so101, q_init);

        // set box default positions
        const drake::Vector3<double> T_mat_box { constants::transforms::X_MAT_BOX::T };
        const drake::Vector3<double> rpy_mat_box { constants::transforms::X_MAT_BOX::R };
        const RollPitchYaw R_mat_box { rpy_mat_box };
        const RigidTransform X_mat_box { R_mat_box, T_mat_box };
        const auto& box_body { plant.GetRigidBodyByName("box_link", box) };
        plant.SetDefaultFloatingBaseBodyPose(box_body, X_mat_box);

        // add so101 controller
        const auto kp { Eigen::VectorXd::Ones(constants::SO101_NUM_Q) * 100 };
        const auto ki { Eigen::VectorXd::Ones(constants::SO101_NUM_Q) * 1 };
        const auto kd { Eigen::VectorXd::Ones(constants::SO101_NUM_Q) * 20 };
        auto controller { 
            diagram_builder.AddSystem<InverseDynamicsController>(
                std::move(control_plant), kp, ki, kd, false
            )
        };
        controller->set_name("so101_controller");
        diagram_builder.Connect(
            plant.get_state_output_port(so101),
            controller->get_input_port_estimated_state()
        );
        diagram_builder.Connect(
            controller->get_output_port_control(),
            plant.get_actuation_input_port()
        );
    }

    inline void generate_so101_place(
        MultibodyPlant<double>& plant,
        SceneGraph<double>& scene_graph,
        Parser& parser,
        const Eigen::VectorXd q_grasp_closed
    ) {
        // add mat
        auto mat = parser.AddModels(constants::model_paths::MAT)[0];
        plant.WeldFrames(
            plant.world_frame(),
            plant.GetFrameByName("mat_link", mat)
        );

        // add so101
        auto so101 = parser.AddModels(constants::model_paths::SO101_OBJ)[0];
        const drake::Vector3<double> T_mat_so101 { constants::transforms::X_MAT_SO101::T };
        const drake::Vector3<double> rpy_mat_so101 { constants::transforms::X_MAT_SO101::R };
        const RollPitchYaw R_mat_so101 { rpy_mat_so101 };
        const RigidTransform X_mat_so101 { R_mat_so101, T_mat_so101 };
        plant.WeldFrames(
            plant.GetFrameByName("mat_link", mat),
            plant.GetFrameByName("base_link", so101),
            X_mat_so101
        );

        // add bin
        auto bin = parser.AddModels(constants::model_paths::BIN_CLR)[0];
        const drake::Vector3<double> T_mat_bin { constants::transforms::X_MAT_BIN::T };
        const drake::Vector3<double> rpy_mat_bin { constants::transforms::X_MAT_BIN::R };
        const RollPitchYaw R_mat_bin { rpy_mat_bin };
        const RigidTransform X_mat_bin { R_mat_bin, T_mat_bin };
        plant.WeldFrames(
            plant.GetFrameByName("mat_link", mat),
            plant.GetFrameByName("bin_link", bin),
            X_mat_bin
        );

        // finalize plant
        plant.Finalize();

        // set so101 default positions
        plant.SetDefaultPositions(so101, q_grasp_closed);
    }

    template<typename T>
    void generate_so101_brick_welded(
        MultibodyPlant<T>& plant,
        SceneGraph<T>& scene_graph
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

    template<typename T>
    void generate_so101_brick_welded(
        MultibodyPlant<T>& plant,
        SceneGraph<T>& scene_graph,
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
    template<typename T>
    void generate_so101_brick_welded(
        MultibodyPlant<T>& plant,
        SceneGraph<T>& scene_graph,
        Parser& parser,
        const Eigen::VectorXd q_init
    ) {
        detail::generate_so101_brick_welded_impl(plant, scene_graph, parser, q_init);
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
