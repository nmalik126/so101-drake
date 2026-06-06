#include "helpers.h"
#include "motion_planning_helpers.h"

#include <drake/planning/robot_diagram_builder.h>
#include <drake/geometry/meshcat.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/primitives/trajectory_source.h>
#include <drake/systems/primitives/discrete_derivative.h>

#include <Eigen/Dense>

#include <iostream>
#include <optional>

using motion_planning::ComputeGraspPlan;

using drake::planning::RobotDiagramBuilder;
using drake::geometry::Meshcat;
using drake::geometry::MeshcatVisualizer;
using drake::systems::Simulator;
using drake::systems::TrajectorySource;
using drake::systems::StateInterpolatorWithDiscreteDerivative;

int main() {
    std::cout << "Simulation" << '\n';

    // init builder
    RobotDiagramBuilder<double> builder {};
    auto& plant { builder.plant() };
    auto& scene_graph { builder.scene_graph() };
    auto& parser { builder.parser() };
    auto& diagram_builder { builder.builder() };

    // init non-welded simulation
    std::cout << "initializing scene..." << '\n';
    helpers::generate_so101_brick(plant, scene_graph, parser, diagram_builder);
    std::cout << "scene initialized." << '\n';

    // init meshcat
    auto meshcat { std::make_shared<Meshcat>() };
    MeshcatVisualizer<double>::AddToBuilder(&diagram_builder, scene_graph, meshcat);

    // compute grasp plan
    std::cout << "computing grasp plan..." << '\n';
    auto trajectory { ComputeGraspPlan() };
    if (!trajectory) {
        std::cout << "Grasp Plan Failure. Exiting..." << '\n';
        return 1;
    }
    std::cout << "grasp plan computed." << '\n';

    // add trajectory source to diagram
    auto q_source {
        diagram_builder.AddSystem<TrajectorySource>(
            trajectory.value()
        )
    };
    auto interpolator {
        diagram_builder.AddSystem<StateInterpolatorWithDiscreteDerivative>(
            constants::SO101_NUM_Q,
            plant.time_step()
        )
    };
    const auto& controller { 
        diagram_builder.GetSubsystemByName("so101_controller")
    };
    diagram_builder.Connect(
        q_source->get_output_port(),
        interpolator->get_input_port()
    );
    diagram_builder.Connect(
        interpolator->get_output_port(),
        controller.GetInputPort("desired_state")
    );
    
    // build diagram
    auto diagram { builder.Build() };
    
    // set desired positions
    // const auto& controller { diagram->GetSubsystemByName("so101_controller") };
    // auto context { diagram->CreateDefaultContext() };
    // auto& controller_context { controller.GetMyMutableContextFromRoot(context.get()) };
    // const auto x0 { Eigen::VectorXd::Zero(constants::SO101_NUM_Q * 2) };
    // controller.GetInputPort("desired_state").FixValue(&controller_context, x0);

    // simulate
    std::cout << "simulating..." << '\n';
    auto context { diagram->CreateDefaultContext() };
    Simulator<double> simulator { *diagram, std::move(context) };
    simulator.set_target_realtime_rate(1.0);
    meshcat->StartRecording();
    simulator.AdvanceTo(5.0);
    meshcat->StopRecording();
    meshcat->PublishRecording();
    helpers::user_input_quit();

    return 0;
}
