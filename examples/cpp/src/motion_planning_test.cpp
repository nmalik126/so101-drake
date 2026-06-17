#include "helpers.h"
#include "kinematics/inverse_kinematics.h"
#include "planning/ompl_planning.h"
#include "optimization/trajectory_optimization.h"

#include <drake/planning/scene_graph_collision_checker.h>
#include <drake/planning/collision_checker_params.h>

#include <iostream>
#include <memory>

using motion_planning::inverse_kinematics::SO101InverseKinematicsPick;
using motion_planning::ompl::SO101OMPL;
using motion_planning::trajectory_optimization::SO101TrajOpt;

using drake::planning::SceneGraphCollisionChecker;
using drake::planning::CollisionCheckerParams;

int main() {
    std::cout << "Motion Planning Test" << std::endl;

    // create scenario
    std::cout << "creating scenario..." << std::endl;
    auto assets { helpers::generate_so101_brick_diagram(true, true) };
    auto& plant { *(assets.plant) };
    auto& visualizer { *(assets.visualizer) };
    auto diagram { assets.diagram };
    std::cout << "scenario created." << std::endl;

    // create context
    auto context { diagram->CreateDefaultContext() };
    const auto& fixed_plant_context { plant.GetMyContextFromRoot(*context) };

    // create collision checker
    std::cout << "creating collision checker..." << std::endl;
    auto so101 { plant.GetModelInstanceByName("so101_new_calib") };
    auto checker { std::make_shared<SceneGraphCollisionChecker>(
        CollisionCheckerParams {
            .model { diagram },
            .robot_model_instances { { so101 } },
            .edge_step_size { 0.01 }
        }
    ) };
    std::cout << "collision checker created." << std::endl;

    // inverse kinematics
    std::cout << "running inverse kinematics..." << std::endl;
    SO101InverseKinematicsPick ik_pick { diagram };
    std::cout << "solving..." << std::endl;
    auto ik_pick_result { ik_pick.solve() };
    if (!ik_pick_result) {
        std::cout << "IK Failure. Exiting..." << std::endl;
        return 1;
    }
    const auto q_pick { ik_pick_result.value() };
    std::cout << "IK Success. Q Pick: " << q_pick.transpose() << std::endl;
    
    // init sampling planner
    SO101OMPL sampling_planner { diagram, checker };
    const auto q_start { plant.GetPositions(fixed_plant_context) };
    std::cout << "setting problem definition..." << std::endl;
    sampling_planner.set_pdef(q_start, q_pick);

    // init trajopt
    SO101TrajOpt trajopt { diagram, checker };

    for (int i { 0 }; i < 10; i++) {
        std::cout << "iteration: " << i << std::endl;
        // run sampling planner
        std::cout << "running sampling planner..." << std::endl;
        checker->SetPaddingAllRobotEnvironmentPairs(8e-3);
        auto sampling_planner_result { sampling_planner.solve() };
        if (!sampling_planner_result) {
            std::cout << "sampling planner failure." << std::endl;
            continue;
        }
        const auto waypoints { sampling_planner_result.value() };
        std::cout << "sampling planner success, waypoints:" << std::endl;
        std::cout << waypoints.transpose() << std::endl;

        // run trajopt
        std::cout << "running trajectory optimization..." << std::endl;
        checker->SetPaddingAllRobotEnvironmentPairs(6e-3);
        trajopt.set_waypoints(waypoints);
        auto trajopt_result { trajopt.solve() };
        if (!trajopt_result) {
            std::cout << "trajopt failure." << std::endl;
            continue;
        }
        const auto trajectory { trajopt_result.value() };
        std::cout << "trajopt success, visualizing..." << std::endl;
        helpers::publish_position_trajectory(
            trajectory, 
            *context, 
            plant, 
            visualizer
        );
        helpers::user_input_quit();
        return 0;
    }
    
    std::cout << "Failed to solve motion planning problem. Exiting..." << std::endl;
    return 1;
}
