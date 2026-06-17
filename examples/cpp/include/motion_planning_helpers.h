#include "helpers.h"
#include "kinematics/inverse_kinematics.h"
#include "planning/ompl_planning.h"
#include "optimization/trajectory_optimization.h"
#include "constants.h"

#include <drake/common/trajectories/bspline_trajectory.h>
#include <drake/planning/robot_diagram_builder.h>
#include <drake/geometry/meshcat.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/common/trajectories/composite_trajectory.h>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/common/copyable_unique_ptr.h>
#include <drake/planning/scene_graph_collision_checker.h>
#include <drake/planning/collision_checker_params.h>
#include <drake/math/rigid_transform.h>

#include <Eigen/Dense>

#include <optional>
#include <memory>

using motion_planning::inverse_kinematics::GenerateGoalConfig;
using motion_planning::ompl::GenerateWaypoints;
using motion_planning::trajectory_optimization::GenerateTrajectory;
using motion_planning::inverse_kinematics::GeneratePlaceConfig;
using motion_planning::inverse_kinematics::SO101InverseKinematics;
using motion_planning::inverse_kinematics::SO101InverseKinematicsPick;
using motion_planning::inverse_kinematics::SO101InverseKinematicsPlace;
using motion_planning::ompl::SO101OMPL;
using motion_planning::trajectory_optimization::SO101TrajOpt;

using drake::trajectories::BsplineTrajectory;
using drake::planning::RobotDiagramBuilder;
using drake::geometry::Meshcat;
using drake::geometry::MeshcatVisualizer;
using drake::trajectories::CompositeTrajectory;
using drake::trajectories::PiecewisePolynomial;
using drake::planning::SceneGraphCollisionChecker;
using drake::planning::CollisionCheckerParams;
using drake::math::RigidTransformd;

namespace motion_planning {

inline std::optional<BsplineTrajectory<double>> ComputeGraspPlan() {
    // init builder
    RobotDiagramBuilder<double> builder {};
    auto& plant { builder.plant() };
    auto& scene_graph { builder.scene_graph() };
    auto& parser { builder.parser() };

    // init scenario
    helpers::generate_so101_brick_welded(plant, scene_graph, parser);

    // build diagram
    std::shared_ptr diagram { builder.Build() };

    // inverse kinematics
    auto ik_result { GenerateGoalConfig(plant, diagram) };
    if (!ik_result) {
        std::cout << "IK Failure" << '\n';
        return std::nullopt;
    }
    const auto q_goal { ik_result.value() };

    // sampling-based motion planning
    auto ompl_result { GenerateWaypoints(plant, diagram, q_goal) };
    if (!ompl_result) {
        std::cout << "OMPL Failure" << '\n';
        return std::nullopt;
    }
    const auto waypoints { ompl_result.value() };

    // trajectory optimization
    auto trajopt_result { GenerateTrajectory(plant, diagram, waypoints) };
    if (!trajopt_result) {
        std::cout << "TrajOpt Failure" << '\n';
        return std::nullopt;
    }
    return trajopt_result.value();
}

inline std::optional<BsplineTrajectory<double>> ComputePlacePlan(
    const Eigen::VectorXd q_grasp_closed
) {
    // init builder
    RobotDiagramBuilder<double> builder {};
    auto& plant { builder.plant() };
    auto& scene_graph { builder.scene_graph() };
    auto& parser { builder.parser() };

    // init scenario
    helpers::generate_so101_place(plant, scene_graph, parser, q_grasp_closed);

    // build diagram
    std::shared_ptr diagram { builder.Build() };

    // inverse kinematics
    auto ik_result { GeneratePlaceConfig(plant, diagram, q_grasp_closed) };
    if (!ik_result) {
        std::cout << "IK Failure" << '\n';
        return std::nullopt;
    }
    const auto q_goal { ik_result.value() };

    // sampling-based motion planning
    auto ompl_result { GenerateWaypoints(plant, diagram, q_grasp_closed, q_goal) };
    if (!ompl_result) {
        std::cout << "OMPL Failure" << '\n';
        return std::nullopt;
    }
    const auto waypoints { ompl_result.value() };

    // trajectory optimization
    auto trajopt_result { GenerateTrajectory(plant, diagram, waypoints) };
    if (!trajopt_result) {
        std::cout << "TrajOpt Failure" << '\n';
        return std::nullopt;
    }
    return trajopt_result.value();
}

inline std::optional<BsplineTrajectory<double>> ComputeRestPlan(
    const Eigen::VectorXd q_place_open
) {
    // init builder
    RobotDiagramBuilder<double> builder {};
    auto& plant { builder.plant() };
    auto& scene_graph { builder.scene_graph() };
    auto& parser { builder.parser() };

    // init scenario
    helpers::generate_so101_brick_welded(plant, scene_graph, parser, q_place_open);

    // build diagram
    std::shared_ptr diagram { builder.Build() };

    // sampling-based motion planning
    const Eigen::VectorXd q_init { 
        Eigen::VectorXd::Map(
            constants::SO101_Q_INIT, 
            constants::SO101_NUM_Q
        ) 
    };
    auto ompl_result { GenerateWaypoints(plant, diagram, q_place_open, q_init) };
    if (!ompl_result) {
        std::cout << "OMPL Failure" << '\n';
        return std::nullopt;
    }
    const auto waypoints { ompl_result.value() };

    // trajectory optimization
    auto trajopt_result { GenerateTrajectory(plant, diagram, waypoints) };
    if (!trajopt_result) {
        std::cout << "TrajOpt Failure" << '\n';
        return std::nullopt;
    }
    return trajopt_result.value();
}

inline std::optional<CompositeTrajectory<double>> ComputePickPlaceTrajectory() {
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

    // compute grasp plan
    std::cout << "computing grasp plan..." << '\n';
    const auto grasp_plan_result { ComputeGraspPlan() };
    if (!grasp_plan_result) {
        std::cout << "Grasp Plan Failure. Exiting..." << '\n';
        return std::nullopt;
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
        return std::nullopt;
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
        return std::nullopt;
    }
    std::cout << "rest plan computed." << '\n';

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
    return CompositeTrajectory<double>::AlignAndConcatenate({
        drake::copyable_unique_ptr<Trajectory<double>> { grasp_trajectory }, 
        drake::copyable_unique_ptr<Trajectory<double>> { gripper_close_trajectory },
        drake::copyable_unique_ptr<Trajectory<double>> { place_trajectory }, 
        drake::copyable_unique_ptr<Trajectory<double>> { gripper_open_trajectory },
        drake::copyable_unique_ptr<Trajectory<double>> { rest_trajectory }
    });
}

namespace detail {

inline std::optional<BsplineTrajectory<double>> GenerateMotionPlanImpl(
    SO101OMPL& sampling_planner,
    SO101TrajOpt& trajopt,
    const Eigen::VectorXd q_start,
    const Eigen::VectorXd q_goal,
    std::shared_ptr<SceneGraphCollisionChecker> checker
) {
    for (int i { 0 }; i < constants::TRAJOPT_N_RETRIES; ++i) {
        std::cout << "motion plan iteration: " << i << std::endl;
        // run sampling planner
        std::cout << "running sampling planner..." << std::endl;
        checker->SetPaddingAllRobotEnvironmentPairs(8e-3);
        sampling_planner.set_pdef(q_start, q_goal);
        auto sampling_planner_result { sampling_planner.solve() };
        if (!sampling_planner_result) {
            std::cout << "Sampling Planner Failure. Retrying..." << std::endl;
            continue;
        }
        const auto waypoints { sampling_planner_result.value() };
        std::cout << "sampling planner success, waypoints:" << std::endl;
        std::cout << waypoints.transpose() << std::endl;
    
        // run trajopt
        std::cout << "running trajectory optimization..." << std::endl;
        checker->SetPaddingAllRobotEnvironmentPairs(1e-3);
        trajopt.set_waypoints(waypoints);
        std::cout << "solving..." << std::endl;
        auto trajopt_result { trajopt.solve() };
        if (!trajopt_result) {
            std::cout << "TrajOpt Failure. Retrying..." << std::endl;
            continue;
        }
        std::cout << "trajopt success." << std::endl;
        return trajopt_result.value();
    }
    std::cout << "Unable to solve motion plan after " 
              << constants::TRAJOPT_N_RETRIES << " retries. Exiting" << std::endl;
    return std::nullopt;
}

} // namespace detail

inline std::optional<BsplineTrajectory<double>> GenerateMotionPlan(
    SO101InverseKinematics& ik,
    SO101OMPL& sampling_planner,
    SO101TrajOpt& trajopt,
    const Eigen::VectorXd q_start,
    std::shared_ptr<SceneGraphCollisionChecker> checker
) {
    // inverse kinematics
    std::cout << "solving inverse kinematics..." << std::endl;
    ik.set_initial_guess(q_start);
    auto ik_result { ik.solve() };
    if (!ik_result) {
        std::cout << "IK Failure. Exiting..." << std::endl;
        return std::nullopt;
    }
    const auto q_goal { ik_result.value() };
    std::cout << "IK Success. Q Goal: " << q_goal.transpose() << std::endl;
    
    return detail::GenerateMotionPlanImpl(
        sampling_planner, trajopt, q_start, q_goal, checker
    );
}

inline std::optional<BsplineTrajectory<double>> GenerateMotionPlan(
    SO101OMPL& sampling_planner,
    SO101TrajOpt& trajopt,
    const Eigen::VectorXd q_start,
    const Eigen::VectorXd q_goal,
    std::shared_ptr<SceneGraphCollisionChecker> checker
) {
    return detail::GenerateMotionPlanImpl(
        sampling_planner, trajopt, q_start, q_goal, checker
    );
}

inline std::optional<CompositeTrajectory<double>> GeneratePickPlaceMotionPlans() {
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
        return std::nullopt;
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
        return std::nullopt;
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
        return std::nullopt;
    }
    const auto rest_trajectory { rest_result.value() };
    
    // construct composite trajectory
    return CompositeTrajectory<double>::AlignAndConcatenate({
        drake::copyable_unique_ptr<Trajectory<double>> { pick_trajectory },
        drake::copyable_unique_ptr<Trajectory<double>> { gripper_close_trajectory },
        drake::copyable_unique_ptr<Trajectory<double>> { place_trajectory },
        drake::copyable_unique_ptr<Trajectory<double>> { gripper_open_trajectory },
        drake::copyable_unique_ptr<Trajectory<double>> { rest_trajectory },
    });
}

} // namespace motion_planning
