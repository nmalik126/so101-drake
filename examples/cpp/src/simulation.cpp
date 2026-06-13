#include "helpers.h"
#include "motion_planning_helpers.h"
#include "constants.h"

#include <drake/planning/robot_diagram_builder.h>
#include <drake/geometry/meshcat.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/primitives/trajectory_source.h>
#include <drake/systems/primitives/discrete_derivative.h>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/common/trajectories/composite_trajectory.h>
#include <drake/common/copyable_unique_ptr.h>
#include <drake/common/trajectories/trajectory.h>

#include <Eigen/Dense>

#include <iostream>
#include <optional>

using motion_planning::ComputeGraspPlan;
using motion_planning::ComputePlacePlan;
using motion_planning::ComputeRestPlan;

using drake::planning::RobotDiagramBuilder;
using drake::geometry::Meshcat;
using drake::geometry::MeshcatVisualizer;
using drake::systems::Simulator;
using drake::systems::TrajectorySource;
using drake::systems::StateInterpolatorWithDiscreteDerivative;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::CompositeTrajectory;
using drake::trajectories::Trajectory;

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
    auto& visualizer { MeshcatVisualizer<double>::AddToBuilder(
        &diagram_builder, scene_graph, meshcat
    ) };

    // compute grasp plan
    std::cout << "computing grasp plan..." << '\n';
    const auto grasp_plan_result { ComputeGraspPlan() };
    if (!grasp_plan_result) {
        std::cout << "Grasp Plan Failure. Exiting..." << '\n';
        return 1;
    }
    std::cout << "grasp plan computed." << '\n';

    // compute place plan
    const auto grasp_trajectory { grasp_plan_result.value() };
    const Eigen::VectorXd q_grasp_open { grasp_trajectory.FinalValue() };
    Eigen::VectorXd q_grasp_closed { q_grasp_open };
    q_grasp_closed(constants::SO101_NUM_Q - 1) = -0.1;
    std::cout << "computing place plan..." << '\n';
    const auto place_plan_result { ComputePlacePlan(q_grasp_closed) };
    if (!place_plan_result) {
        std::cout << "Place Plan Failure. Exiting..." << '\n';
        return 1;
    }
    std::cout << "place plan computed." << '\n';

    // compute rest plan
    const auto place_trajectory { place_plan_result.value() };
    const Eigen::VectorXd q_place_closed { place_trajectory.FinalValue() };
    Eigen::VectorXd q_place_open { q_place_closed };
    q_place_open(constants::SO101_NUM_Q - 1) = 0.5;
    std::cout << "computing rest plan..." << '\n';
    const auto rest_plan_result { ComputeRestPlan(q_place_open) };
    if (!rest_plan_result) {
        std::cout << "Rest Plan Failure. Exiting..." << '\n';
        return 1;
    }
    std::cout << "rest plan computed." << '\n';

    // /*
    // concat trajectories
    const auto gripper_close_trajectory {
        PiecewisePolynomial<double>::FirstOrderHold(
            { 0.0, 1.0 },
            { q_grasp_open, q_grasp_closed }
        )
    };
    const auto gripper_open_trajectory {
        PiecewisePolynomial<double>::FirstOrderHold(
            { 0.0, 1.0 },
            { q_place_closed, q_place_open }
        )
    };
    const auto rest_trajectory { rest_plan_result.value() };
    const auto trajectory {
        CompositeTrajectory<double>::AlignAndConcatenate({
            drake::copyable_unique_ptr<Trajectory<double>> { grasp_trajectory }, 
            drake::copyable_unique_ptr<Trajectory<double>> { gripper_close_trajectory },
            drake::copyable_unique_ptr<Trajectory<double>> { place_trajectory }, 
            drake::copyable_unique_ptr<Trajectory<double>> { gripper_open_trajectory },
            drake::copyable_unique_ptr<Trajectory<double>> { rest_trajectory }
        })
    };
    
    // /*
    // add trajectory source to diagram
    auto q_source {
        diagram_builder.AddSystem<TrajectorySource>(trajectory)
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
    
    // simulate
    std::cout << "simulating..." << '\n';
    Simulator<double> simulator { *diagram };
    simulator.set_target_realtime_rate(1.0);
    meshcat->StartRecording();
    simulator.AdvanceTo(trajectory.end_time());
    meshcat->StopRecording();
    meshcat->PublishRecording();
    helpers::user_input_quit();
    // */
    
    return 0;
}
