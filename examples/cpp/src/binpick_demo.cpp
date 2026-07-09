#include "hardware/so101_lcm.h"
#include "helpers.h"
#include "scenario_helpers.h"
#include "constants.h"
#include "motion_planning_helpers.h"

#include "messages/so101/lcmt_so101_configuration.hpp"
#include "messages/so101/lcmt_so101_grasp.hpp"

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
#include <drake/lcm/drake_lcm_interface.h>

#include <Eigen/Dense>

// #include <lcm/lcm-cpp.hpp>

#include <iostream>
#include <fmt/format.h>
#include <fmt/ostream.h>

using motion_planning::hardware::SO101StatusReceiver;
using motion_planning::hardware::SO101CommandSender;
// using motion_planning::hardware::GraspPoseSubscriber;
using motion_planning::GenerateBinpickMotionPlans;
using motion_planning::MutableTrajectorySource;

using drake::systems::DiagramBuilder;
using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::geometry::Meshcat;
using drake::geometry::MeshcatVisualizer;
using drake::systems::Simulator;
using drake::systems::lcm::LcmInterfaceSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::rendering::MultibodyPositionToGeometryPose;
using drake::systems::TrajectorySource;
using drake::math::RotationMatrixd;
using drake::trajectories::CompositeTrajectory;

int main() {
    std::cout << "Bin Pick Demo" << '\n';

    // init diagram
    DiagramBuilder<double> diagram_builder {};
    auto* scene_graph {
        diagram_builder.AddSystem<SceneGraph>()
    };
    MultibodyPlant<double> plant { 1e-3 };
    plant.RegisterAsSourceForSceneGraph(scene_graph);
    Parser parser { &plant };

    // init scenario
    helpers::generate_so101_binpick_welded(plant, *scene_graph, parser);
    std::cout << "parsing finished." << '\n';

    // init meshcat
    auto meshcat { std::make_shared<Meshcat>() };
    auto& visualizer { MeshcatVisualizer<double>::AddToBuilder(
        &diagram_builder, *scene_graph, meshcat
    ) };

    // init lcm
    auto* lcm {
        diagram_builder.AddSystem<LcmInterfaceSystem>()
    };

    // create status subscriber and receiver
    auto* status_sub { diagram_builder.AddSystem(
        LcmSubscriberSystem::Make<so101::lcmt_so101_configuration>(
            "SO101_STATUS", lcm
        )
    ) };
    auto* status_receiver {
        diagram_builder.AddSystem<SO101StatusReceiver>()
    };
    diagram_builder.Connect(
        status_sub->get_output_port(),
        status_receiver->get_input_port(0)
    );

    // create command publisher and sender
    auto* command_pub { diagram_builder.AddSystem(
        LcmPublisherSystem::Make<so101::lcmt_so101_configuration>(
            "SO101_COMMAND", lcm, 0.005
        )
    ) };
    auto* command_sender {
        diagram_builder.AddSystem<SO101CommandSender>()
    };
    diagram_builder.Connect(
        command_sender->get_output_port(0),
        command_pub->get_input_port()
    );

    // wire to-pose
    auto* to_pose {
        diagram_builder.AddSystem<MultibodyPositionToGeometryPose>(plant)
    };
    diagram_builder.Connect(
        to_pose->get_output_port(),
        scene_graph->get_source_pose_port(
            plant.get_source_id().value()
        )
    );
    diagram_builder.Connect(
        status_receiver->get_output_port(),
        to_pose->get_input_port()
    );

    // wire trajectory source
    auto q_source {
        diagram_builder.AddSystem<MutableTrajectorySource>(
            constants::SO101_NUM_Q
        )
    };
    diagram_builder.Connect(
        q_source->get_output_port(),
        command_sender->get_input_port()
    );

    // build diagram
    auto diagram { diagram_builder.Build() };
    Simulator<double> simulator { *diagram };
    simulator.set_target_realtime_rate(1.0);
    meshcat->StartRecording();

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

    // define q_init and q_start
    const Eigen::VectorXd q_init { 
        Eigen::VectorXd::Map(
            constants::SO101_Q_INIT, constants::SO101_NUM_Q
        ) 
    };
    Eigen::VectorXd q_start { q_init };

    // start lcm

    // lcm::LCM lc;
    // if (!lc.good()) {
    //     std::cout << "Failed to start LCM. Exiting..." << std::endl;
    //     return 1;
    // }
    // GraspPoseSubscriber graspPoseSubscriber {};
    // auto* subscription { lc.subscribe(
    //     "SO101_GRASP",
    //     &GraspPoseSubscriber::handleMessage,
    //     &graspPoseSubscriber
    // ) };
    // subscription->setQueueCapacity(1);

    drake::lcm::Subscriber<so101::lcmt_so101_grasp> grasp_sub { 
        lcm, "SO101_GRASP"
    };
    
    constexpr int NUM_ITERS { 2 };
    for (int i { 0 }; i < NUM_ITERS; ++i) {
        // const Eigen::Matrix3d grasp_rotation {
        //     {  0.71400319, -0.70000178,  0.01402814 },
        //     { -0.70009843, -0.7135907 ,  0.02550246 },
        //     { -0.0078414 , -0.02802991, -0.9995763  }
        // };
        // const Eigen::Vector3d grasp_translation { -0.09817977, 0.06973192, 0.12975439 };
        // const RigidTransformd grasp_transform { 
        //     RotationMatrixd { grasp_rotation } * RotationMatrixd::MakeXRotation(pi), 
        //     grasp_translation 
        // };

        // lc.handle();
        // const auto grasp_transform { graspPoseSubscriber.get_grasp_pose() };
        
        lcm->HandleSubscriptions(0);
        grasp_sub.clear();
        LcmHandleSubscriptionsUntil(
            lcm, [&]() { return grasp_sub.count() > 0; }, 10
        );
        const RigidTransformd cgn_transform {
            Eigen::Matrix<double, 3, 4, Eigen::RowMajor> { 
                grasp_sub.message().grasp
            }
        };
        const RigidTransformd grasp_transform {
            cgn_transform.rotation() * RotationMatrixd::MakeXRotation(pi),
            cgn_transform.translation()
        };
        fmt::print("Grasp Transform:\n{}\n", grasp_transform);

        const auto pick_place_result { GenerateBinpickMotionPlans(
            // grasp_transform, plan_plant, plan_diagram, checker, q_start, i == NUM_ITERS - 1
            grasp_transform, plan_plant, plan_diagram, checker, q_init, true
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

    meshcat->StopRecording();
    meshcat->PublishRecording();
    helpers::user_input_quit();

    return 0;
}
