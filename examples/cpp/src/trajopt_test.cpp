#include "helpers.h"
#include "scenario_helpers.h"
#include "optimization/trajectory_optimization.h"

#include <drake/planning/scene_graph_collision_checker.h>
#include <drake/planning/collision_checker_params.h>

#include <Eigen/Dense>

#include <iostream>
#include <memory>

using motion_planning::trajectory_optimization::SO101TrajOpt;

using drake::planning::SceneGraphCollisionChecker;
using drake::planning::CollisionCheckerParams;

int main() {
    std::cout << "TrajOpt Test" << std::endl;

    // create scenario
    std::cout << "creating scenario..." << std::endl;
    auto assets { helpers::generate_so101_brick_diagram(true, true) };
    auto& plant { *(assets.plant) };
    auto& visualizer { *(assets.visualizer) };
    auto diagram { assets.diagram };
    std::cout << "scenario created." << std::endl;

    // create context
    auto context { diagram->CreateDefaultContext() };

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

    // define waypoints
    const Eigen::MatrixXd waypoints_transpose {
        {         0,     -1.822,       1.55,      0.906,          0,          0},
        {-0.0100888,   -1.79968,     1.5166,   0.888432,  0.0012219, 0.00449613},
        {-0.0201775,   -1.77736,    1.48321,   0.870864, 0.00244381, 0.00899226},
        { -0.184314,    0.11317,   0.157648,    0.95813,    1.21599,   0.527864},
        {  -0.20422,   0.133516,   0.170679,   0.986003,    1.23802,   0.529527},
        { -0.224125,   0.153862,   0.183709,    1.01388,    1.26005,   0.531189},
        { -0.244031,   0.174207,    0.19674,    1.04175,    1.28208,   0.532852},
        { -0.263936,   0.194553,    0.20977,    1.06962,    1.30411,   0.534514},
    };
    const Eigen::MatrixXd waypoints { waypoints_transpose.transpose() };

    // run trajopt
    std::cout << "running trajectory optimization..." << std::endl;
    SO101TrajOpt trajopt { diagram, checker };
    checker->SetPaddingAllRobotEnvironmentPairs(6e-3);
    trajopt.set_waypoints(waypoints);
    auto trajopt_result { trajopt.solve() };
    if (!trajopt_result) {
        std::cout << "trajopt failure." << std::endl;
        return 1;
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
