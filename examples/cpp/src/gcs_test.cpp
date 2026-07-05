#include "helpers.h"
#include "scenario_helpers.h"

#include <drake/planning/scene_graph_collision_checker.h>
#include <drake/planning/collision_checker_params.h>
#include <drake/geometry/optimization/iris.h>
#include <drake/geometry/optimization/hpolyhedron.h>
#include <drake/geometry/optimization/convex_set.h>
#include <drake/planning/trajectory_optimization/gcs_trajectory_optimization.h>
#include <drake/common/copyable_unique_ptr.h>
#include <drake/geometry/optimization/point.h>

#include <Eigen/Dense>

#include <iostream>

using drake::planning::SceneGraphCollisionChecker;
using drake::planning::CollisionCheckerParams;
using drake::geometry::optimization::IrisOptions;
using drake::geometry::optimization::IrisNp;
using drake::geometry::optimization::HPolyhedron;
using drake::geometry::optimization::ConvexSets;
using drake::planning::trajectory_optimization::GcsTrajectoryOptimization;
using drake::geometry::optimization::Point;

int main() {
    std::cout << "GCS Test" << std::endl;

    // create scenario
    std::cout << "creating scenario..." << std::endl;
    const auto assets { helpers::generate_so101_brick_diagram(true, true) };
    const auto& plant { *(assets.plant) };
    auto& visualizer { *(assets.visualizer) };
    auto diagram { assets.diagram };
    std::cout << "scenario created." << std::endl;

    // create context
    auto context { diagram->CreateDefaultContext() };
    const auto& fixed_plant_context { plant.GetMyContextFromRoot(*context) };
    auto& mutable_plant_context { plant.GetMyMutableContextFromRoot(context.get()) };

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

    // define seed points
    const Eigen::MatrixXd seed_points {
        {         0,     -1.822,       1.55,      0.906,          0,          0},
        {         0,          0,          0,        1.5,          0,          0},
        { -0.263936,   0.194553,    0.20977,    1.06962,    1.30411,   0.534514},
    };

    // define iris options
    const IrisOptions options {
        .require_sample_point_is_contained { true },
        .num_collision_infeasible_samples { 3 },
        .random_seed { 1235 },
    };

    // generate iris regions
    std::cout << "generating iris regions" << std::endl;
    ConvexSets regions { static_cast<size_t>(seed_points.rows()) };
    for (int i { 0 }; i < seed_points.rows(); ++i) {
        std::cout << "generating region #" << i << std::endl;
        plant.SetPositions(&mutable_plant_context, so101, seed_points.row(i));
        regions[i] = drake::copyable_unique_ptr<HPolyhedron> {
            IrisNp(plant, fixed_plant_context, options)
        };
    }
    std::cout << "iris regions generated" << std::endl;

    // perform gcs trajectory optimization
    std::cout << "performing gcs trajopt" << std::endl;

    const int order { 5 };
    const int continuity_order { 4 };

    GcsTrajectoryOptimization trajopt { plant.num_positions() };
    
    auto& gcs_regions { trajopt.AddRegions(regions, order) };
    
    ConvexSets start_region { 1 };
    ConvexSets goal_region { 1 };
    start_region[0] = drake::copyable_unique_ptr {
        Point { seed_points.row(0) }
    };
    goal_region[0] = drake::copyable_unique_ptr {
        Point { seed_points.row(seed_points.rows() - 1) }
    };
    auto& source { trajopt.AddRegions(start_region, 0) };
    auto& target { trajopt.AddRegions(goal_region, 0) };

    trajopt.AddEdges(source, gcs_regions);
    trajopt.AddEdges(gcs_regions, target);

    trajopt.AddTimeCost();
    trajopt.AddVelocityBounds(
        plant.GetVelocityLowerLimits(),
        plant.GetVelocityUpperLimits()
    );

    for (int i { 1 }; i <= continuity_order; ++i)
        trajopt.AddContinuityConstraints(i);

    auto [traj, result] { trajopt.SolvePath(source, target) };

    if (!result.is_success()) {
        std::cout << "gcs trajopt failed" << std::endl;
        return 1;   
    }

    std::cout << "gcs trajopt success, visualizing..." << std::endl;
    helpers::publish_position_trajectory(
        traj, 
        *context, 
        plant, 
        visualizer
    );
    helpers::user_input_quit();

    return 0;
}
