#include "helpers.h"
#include "kinematics/inverse_kinematics.h"
#include "planning/ompl_planning.h"
#include "optimization/trajectory_optimization.h"
#include "motion_planning_helpers.h"

#include <drake/planning/scene_graph_collision_checker.h>
#include <drake/planning/collision_checker_params.h>
#include <drake/systems/primitives/trajectory_source.h>
#include <drake/systems/primitives/discrete_derivative.h>
#include <drake/systems/analysis/simulator.h>

#include <iostream>
#include <memory>

using motion_planning::inverse_kinematics::SO101InverseKinematicsPick;
using motion_planning::ompl::SO101OMPL;
using motion_planning::trajectory_optimization::SO101TrajOpt;

using drake::planning::SceneGraphCollisionChecker;
using drake::planning::CollisionCheckerParams;
using drake::systems::TrajectorySource;
using drake::systems::StateInterpolatorWithDiscreteDerivative;
using drake::systems::Simulator;

int main() {
    std::cout << "simulation refactor" << std::endl;

    // create sim scenario
    std::cout << "creating sim scenario..." << std::endl;
    auto sim_assets { helpers::generate_so101_brick_diagram(false, true) };
    auto sim_builder { std::move(sim_assets.builder) };
    auto& sim_plant { *(sim_assets.plant) };
    auto sim_meshcat { sim_assets.meshcat };
    auto& sim_diagram_builder { *(sim_assets.diagram_builder) };
    std::cout << "sim scenario created." << std::endl;

    // create plan scenario
    std::cout << "creating plan scenario..." << std::endl;
    auto plan_assets { helpers::generate_so101_brick_diagram(true, false) };
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

    // create motion planning objects
    SO101InverseKinematicsPick ik_pick { plan_diagram };
    SO101OMPL sampling_planner { plan_diagram, checker };
    SO101TrajOpt trajopt { plan_diagram, checker };
    const Eigen::VectorXd q_init { 
        Eigen::VectorXd::Map(
            constants::SO101_Q_INIT, constants::SO101_NUM_Q
        ) 
    };

    // solve motion planning problem
    auto pick_result { motion_planning::GenerateMotionPlan(
        ik_pick, sampling_planner, trajopt, q_init, checker
    ) };
    if (!pick_result) {
        std::cout << "Pick Plan Failed. Exiting..." << std::endl;
        return 1;
    }
    const auto trajectory { pick_result.value() };

    // add trajectory source to diagram
    auto q_source {
        sim_diagram_builder.AddSystem<TrajectorySource>(trajectory)
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
    auto sim_diagram { sim_builder->Build() };
    
    // simulate
    std::cout << "simulating..." << '\n';
    Simulator<double> simulator { *sim_diagram };
    simulator.set_target_realtime_rate(1.0);
    sim_meshcat->StartRecording();
    simulator.AdvanceTo(trajectory.end_time());
    sim_meshcat->StopRecording();
    sim_meshcat->PublishRecording();
    helpers::user_input_quit();

    return 0;
}
