#include "helpers.h"
#include "kinematics/inverse_kinematics.h"
#include "planning/ompl_planning.h"
#include "optimization/trajectory_optimization.h"
#include "motion_planning_helpers.h"
#include "constants.h"

#include <drake/planning/scene_graph_collision_checker.h>
#include <drake/planning/collision_checker_params.h>
#include <drake/systems/primitives/trajectory_source.h>
#include <drake/systems/primitives/discrete_derivative.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/common/trajectories/composite_trajectory.h>
#include <drake/common/copyable_unique_ptr.h>
#include <drake/common/trajectories/trajectory.h>
#include <drake/math/rigid_transform.h>

#include <Eigen/Dense>

#include <iostream>
#include <memory>

using motion_planning::inverse_kinematics::SO101InverseKinematicsPick;
using motion_planning::inverse_kinematics::SO101InverseKinematicsPlace;
using motion_planning::ompl::SO101OMPL;
using motion_planning::trajectory_optimization::SO101TrajOpt;

using drake::planning::SceneGraphCollisionChecker;
using drake::planning::CollisionCheckerParams;
using drake::systems::TrajectorySource;
using drake::systems::StateInterpolatorWithDiscreteDerivative;
using drake::systems::Simulator;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::CompositeTrajectory;
using drake::trajectories::Trajectory;
using drake::math::RigidTransformd;

int main() {
    std::cout << "Simulation Demo" << std::endl;

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
    SO101InverseKinematicsPlace ik_place { plan_diagram };
    SO101OMPL sampling_planner { plan_diagram, checker };
    SO101TrajOpt trajopt { plan_diagram, checker };
    const Eigen::VectorXd q_init { 
        Eigen::VectorXd::Map(
            constants::SO101_Q_INIT, constants::SO101_NUM_Q
        ) 
    };

    // solve pick plan
    std::cout << "solving pick plan..." << std::endl;
    auto pick_result { motion_planning::GenerateMotionPlan(
        ik_pick, sampling_planner, trajopt, q_init, checker
    ) };
    if (!pick_result) {
        std::cout << "Pick Plan Failed. Exiting..." << std::endl;
        return 1;
    }
    const auto pick_trajectory { pick_result.value() };

    // compute gripper close trajectory
    const Eigen::VectorXd q_pick_open { pick_trajectory.FinalValue() };
    Eigen::VectorXd q_pick_closed { q_pick_open };
    q_pick_closed(constants::SO101_NUM_Q - 1) = -0.1;
    const auto gripper_close_trajectory {
        PiecewisePolynomial<double>::FirstOrderHold(
            { 0.0, 1.0 },
            { q_pick_open, q_pick_closed }
        )
    };

    // filter collisions between welded box and gripper bodies
    const auto& box_body { plan_plant.GetBodyByName("box_link") };
    const auto& gripper_link_body { plan_plant.GetBodyByName("gripper_link") };
    const auto& moving_jaw_body { plan_plant.GetBodyByName("moving_jaw_so101_v1_link") };
    checker->SetCollisionFilteredBetween(box_body.index(), gripper_link_body.index(), true);
    checker->SetCollisionFilteredBetween(box_body.index(), moving_jaw_body.index(), true);

    // add box collision geometry to gripper body
    auto fk_context { plan_plant.CreateDefaultContext() };
    plan_plant.SetPositions(fk_context.get(), q_pick_closed);
    const RigidTransformd X_world_gripper { plan_plant.EvalBodyPoseInWorld(*fk_context, gripper_link_body) };
    const RigidTransformd X_world_box { plan_plant.EvalBodyPoseInWorld(*fk_context, box_body) };
    const RigidTransformd X_gripper_box { X_world_gripper.inverse() * X_world_box };
    checker->AddCollisionShapeToBody(
        "grasped_box",
        gripper_link_body,
        drake::geometry::Box(0.04, 0.03, 0.03),
        X_gripper_box
    );
    
    // solve place plan
    std::cout << "solving place plan..." << std::endl;
    auto place_result { motion_planning::GenerateMotionPlan(
        ik_place, sampling_planner, trajopt, q_pick_closed, checker
    ) };
    if (!place_result) {
        std::cout << "Place Plan Failed. Exiting..." << std::endl;
        return 1;
    }
    const auto place_trajectory { place_result.value() };

    // compute gripper open trajectory
    const Eigen::VectorXd q_place_closed { place_trajectory.FinalValue() };
    Eigen::VectorXd q_place_open { q_place_closed };
    q_place_open(constants::SO101_NUM_Q - 1) = 0.5;
    const auto gripper_open_trajectory {
        PiecewisePolynomial<double>::FirstOrderHold(
            { 0.0, 1.0 },
            { q_place_closed, q_place_open }
        )
    };

    // remove box collision geometry from gripper body
    checker->RemoveAllAddedCollisionShapes("grasped_box");

    // solve rest plan
    std::cout << "solving rest plan..." << std::endl;
    auto rest_result { motion_planning::GenerateMotionPlan(
        sampling_planner, trajopt, q_place_open, q_init, checker
    ) };
    if (!rest_result) {
        std::cout << "Rest Plan Failed. Exiting..." << std::endl;
        return 1;
    }
    const auto rest_trajectory { rest_result.value() };
    
    // construct composite trajectory
    const auto trajectory {
        CompositeTrajectory<double>::AlignAndConcatenate({
            drake::copyable_unique_ptr<Trajectory<double>> { pick_trajectory },
            drake::copyable_unique_ptr<Trajectory<double>> { gripper_close_trajectory },
            drake::copyable_unique_ptr<Trajectory<double>> { place_trajectory },
            drake::copyable_unique_ptr<Trajectory<double>> { gripper_open_trajectory },
            drake::copyable_unique_ptr<Trajectory<double>> { rest_trajectory },
        })
    };

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
    std::cout << "simulating..." << std::endl;
    Simulator<double> simulator { *sim_diagram };
    simulator.set_target_realtime_rate(1.0);
    sim_meshcat->StartRecording();
    simulator.AdvanceTo(trajectory.end_time());
    sim_meshcat->StopRecording();
    sim_meshcat->PublishRecording();
    helpers::user_input_quit();

    return 0;
}
