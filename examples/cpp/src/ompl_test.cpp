#include "helpers.h"
#include "scenario_helpers.h"
#include "planning/ompl_planning.h"
#include "constants.h"

#include <drake/planning/scene_graph_collision_checker.h>
#include <drake/planning/collision_checker_params.h>
#include <drake/common/trajectories/piecewise_polynomial.h>

#include <Eigen/Dense>

#include <iostream>
#include <memory>

using motion_planning::ompl::SO101OMPL;

using drake::planning::SceneGraphCollisionChecker;
using drake::planning::CollisionCheckerParams;
using drake::trajectories::PiecewisePolynomial;

int main() {
    std::cout << "OMPL Test" << std::endl;

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
    
    // init sampling planner
    const Eigen::VectorXd q_pick { {
        -0.263936, 0.194553, 0.20977, 1.06962, 1.30411, 0.534514
    } };
    SO101OMPL sampling_planner { diagram, checker };
    const auto q_start { plant.GetPositions(fixed_plant_context) };
    std::cout << "setting problem definition..." << std::endl;
    sampling_planner.set_pdef(q_start, q_pick);

    // run sampling planner
    std::cout << "running sampling planner..." << std::endl;
    checker->SetPaddingAllRobotEnvironmentPairs(8e-3);
    auto sampling_planner_result { sampling_planner.solve() };
    if (!sampling_planner_result) {
        std::cout << "sampling planner failure." << std::endl;
        return 1;
    }
    const auto waypoints { sampling_planner_result.value() };
    std::cout << "sampling planner success, waypoints:" << std::endl;
    std::cout << waypoints.transpose() << std::endl;

    // publish position trajectory
    Eigen::VectorXd times = Eigen::VectorXd::LinSpaced(constants::TRAJOPT_N_WAYPOINTS, 0.0, 5.0);
    PiecewisePolynomial ompl_traj { PiecewisePolynomial<double>::FirstOrderHold(times, waypoints) };
    helpers::publish_position_trajectory(ompl_traj, *context, plant, visualizer);
    helpers::user_input_quit();

    return 0;
}
