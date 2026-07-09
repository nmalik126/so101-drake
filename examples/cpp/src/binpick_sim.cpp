#include "helpers.h"
#include "scenario_helpers.h"
#include "constants.h"
#include "motion_planning_helpers.h"

#include <drake/systems/framework/diagram_builder.h>
#include <drake/geometry/scene_graph.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/geometry/meshcat.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/lcm/lcm_interface_system.h>
#include <drake/systems/lcm/lcm_subscriber_system.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <drake/systems/rendering/multibody_position_to_geometry_pose.h>
#include <drake/systems/primitives/trajectory_source.h>
#include <drake/math/rotation_matrix.h>
#include <drake/common/trajectories/composite_trajectory.h>
#include <drake/systems/primitives/discrete_derivative.h>

#include <Eigen/Dense>

#include <iostream>

using motion_planning::GenerateBinpickMotionPlans;
using motion_planning::MutableTrajectorySource;

using drake::systems::DiagramBuilder;
using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::geometry::Meshcat;
using drake::geometry::MeshcatVisualizer;
using drake::systems::Simulator;
using drake::systems::rendering::MultibodyPositionToGeometryPose;
using drake::systems::TrajectorySource;
using drake::math::RotationMatrixd;
using drake::trajectories::CompositeTrajectory;
using drake::systems::StateInterpolatorWithDiscreteDerivative;

int main() {
    std::cout << "Bin Pick Demo" << '\n';

    // create sim scenario
    std::cout << "creating sim scenario..." << std::endl;
    auto sim_assets { helpers::generate_so101_brick_diagram(false, true, true) };
    auto sim_builder { std::move(sim_assets.builder) };
    auto& sim_plant { *(sim_assets.plant) };
    auto sim_meshcat { sim_assets.meshcat };
    auto& sim_diagram_builder { *(sim_assets.diagram_builder) };
    std::cout << "sim scenario created." << std::endl;

    // wire trajectory source
    auto q_source {
        sim_diagram_builder.AddSystem<MutableTrajectorySource>(
            constants::SO101_NUM_Q
        )
    };
    auto interpolator {
        sim_diagram_builder.AddSystem<StateInterpolatorWithDiscreteDerivative>(
            constants::SO101_NUM_Q,
            sim_plant.time_step()
        )
    };
    const auto& controller { 
        sim_diagram_builder.GetSubsystemByName("so101_controller")
    };
    sim_diagram_builder.Connect(
        q_source->get_output_port(),
        interpolator->get_input_port()
    );
    sim_diagram_builder.Connect(
        interpolator->get_output_port(),
        controller.GetInputPort("desired_state")
    );

    // build diagram
    auto diagram { sim_diagram_builder.Build() };
    Simulator<double> simulator { *diagram };
    simulator.set_target_realtime_rate(1.0);
    sim_meshcat->StartRecording();

    // create plan scenario
    std::cout << "creating plan scenario..." << std::endl;
    auto plan_assets { helpers::generate_so101_brick_diagram(true, false, true) };
    auto& plan_plant { *(plan_assets.plant) };
    auto plan_diagram { plan_assets.diagram };
    std::cout << "plan scenario created." << std::endl;

    // create collision checker
    std::cout << "creating collision checker..." << std::endl;
    auto checker { std::make_shared<SceneGraphCollisionChecker>(
        CollisionCheckerParams {
            .model { plan_diagram },
            .robot_model_instances { {
                plan_plant.GetModelInstanceByName("so101_new_calib")
            } },
            .edge_step_size { 0.01 }
        }
    ) };
    std::cout << "collision checker created." << std::endl;

    const Eigen::VectorXd q_init { 
        Eigen::VectorXd::Map(
            constants::SO101_Q_INIT, constants::SO101_NUM_Q
        ) 
    };
    Eigen::VectorXd q_start { q_init };
    
    constexpr int NUM_ITERS { 2 };
    for (int i { 0 }; i < NUM_ITERS; ++i) {
        const Eigen::Matrix3d grasp_rotation {
            {  0.71400319, -0.70000178,  0.01402814 },
            { -0.70009843, -0.7135907 ,  0.02550246 },
            { -0.0078414 , -0.02802991, -0.9995763  }
        };
        const Eigen::Vector3d grasp_translation { -0.09817977, 0.06973192, 0.12975439 };
        const RigidTransformd grasp_transform { 
            RotationMatrixd { grasp_rotation } * RotationMatrixd::MakeXRotation(pi), 
            grasp_translation 
        };
        const auto pick_place_result { GenerateBinpickMotionPlans(
            grasp_transform, plan_plant, plan_diagram, checker, q_start, i == NUM_ITERS - 1
        ) };
        if (!pick_place_result) {
            std::cout << "Bin Pick Computation Failed. Exiting..." << '\n';
            return 1;
        }
        auto trajectory {
            std::make_unique<CompositeTrajectory<double>>(
                pick_place_result.value()
            )
        };
        const double duration { trajectory->end_time() };
        q_start = trajectory->value(duration);
        q_source->SetTrajectory(
            std::move(trajectory),
            simulator.get_context().get_time()
        );
        simulator.ResetStatistics();
        std::cout << "simulating plan #" << i+1 << "..." << '\n';
        simulator.AdvanceTo(
            simulator.get_context().get_time() + duration
        );
    }

    sim_meshcat->StopRecording();
    sim_meshcat->PublishRecording();
    helpers::user_input_quit();

    return 0;
}
