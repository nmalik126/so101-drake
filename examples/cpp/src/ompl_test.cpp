#include "helpers.h"
#include "constants.h"
#include "planning/ompl_validity_checker.h"
#include "planning/ompl_helpers.h"

#include <drake/planning/robot_diagram_builder.h>
#include <drake/geometry/meshcat.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/geometry/meshcat_visualizer_params.h>
#include <drake/planning/robot_diagram.h>
#include <drake/common/trajectories/piecewise_polynomial.h>

#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/Planner.h>
#include <ompl/base/PlannerStatus.h>

#include <Eigen/Dense>

#include <iostream>
#include <memory>
#include <limits>
#include <vector>

using drake::planning::RobotDiagramBuilder;
using drake::geometry::Meshcat;
using drake::geometry::MeshcatVisualizer;
using drake::geometry::MeshcatVisualizerParams;
using drake::trajectories::PiecewisePolynomial;

namespace ob = ompl::base;
namespace og = ompl::geometric;

int main() {
    std::cout << "OMPL Test" << '\n';
    std::cout << '\n';

    // init diagram
    RobotDiagramBuilder<double> builder {};
    auto& plant { builder.plant() };
    auto& scene_graph { builder.scene_graph() };
    auto& parser { builder.parser() };

    // init scenario
    std::cout << "parsing started..." << '\n';
    helpers::generate_so101_brick_welded(plant, scene_graph, parser);
    std::cout << "parsing finished." << '\n';
    std::cout << '\n';

    // init visualizers
    auto meshcat { std::make_shared<Meshcat>() };
    auto& visualizer {
        MeshcatVisualizer<double>::AddToBuilder(
            &(builder.builder()),
            scene_graph,
            meshcat,
            MeshcatVisualizerParams {
                .role { drake::geometry::Role::kIllustration }
            }
        )
    };
    auto& collision_visualizer {
        MeshcatVisualizer<double>::AddToBuilder(
            &(builder.builder()),
            scene_graph,
            meshcat,
            MeshcatVisualizerParams {
                .role { drake::geometry::Role::kIllustration },
                .prefix { "collision" },
                .visible_by_default { false }
            }
        )
    };

    // create context
    std::shared_ptr diagram { builder.Build() };
    auto context { diagram->CreateDefaultContext() };
    const auto& fixed_plant_context { plant.GetMyContextFromRoot(*context) };
    auto q_start { plant.GetPositions(fixed_plant_context) };
    const Eigen::VectorXd q_goal { { -0.257417, 0.164283, 0.253324, 1.04113, 1.3605, 0.574355 } };
    std::cout << "q_start: \n" << q_start << '\n';
    std::cout << '\n';
    std::cout << "q_goal: \n" << q_goal << '\n';
    std::cout << '\n';

    // create space
    std::cout << "creating space..." << '\n';
    Eigen::VectorXd lower_bounds { plant.GetPositionLowerLimits() };
    Eigen::VectorXd upper_bounds { plant.GetPositionUpperLimits() };
    ob::RealVectorBounds bounds { constants::SO101_NUM_Q };
    for (int i { 0 }; i < constants::SO101_NUM_Q; ++i) {
        bounds.setLow(i, lower_bounds(i));
        bounds.setHigh(i, upper_bounds(i));
    }
    auto space { std::make_shared<ob::RealVectorStateSpace>(constants::SO101_NUM_Q) };
    space->setBounds(bounds);
    std::cout << "space created." << '\n';

    // create space information
    std::cout << "creating space info..." << '\n';
    auto si { std::make_shared<ob::SpaceInformation>(space) };
    si->setStateValidityChecker(
        std::make_shared<DrakeSO101ValidityChecker>(si, diagram)
    );
    si->setup();
    std::cout << "space info created." << '\n';

    // create problem definition
    std::cout << "creating problem definition..." << '\n';
    ob::ScopedState start { space };
    ob::ScopedState goal { space };
    for (int i { 0 }; i < constants::SO101_NUM_Q; ++i) {
        start->as<ob::RealVectorStateSpace::StateType>()->values[i] = q_start(i);
        goal->as<ob::RealVectorStateSpace::StateType>()->values[i] = q_goal(i);
    }
    auto pdef { std::make_shared<ob::ProblemDefinition>(si) };
    pdef->setStartAndGoalStates(start, goal);
    pdef->setOptimizationObjective(
        std::make_shared<ob::PathLengthOptimizationObjective>(si)
    );
    std::cout << "problem definition created." << '\n';

    // create planner
    std::cout << "creating planner..." << '\n';
    ob::PlannerPtr planner { std::make_shared<og::RRTConnect>(si, true) };
    planner->setProblemDefinition(pdef);
    planner->setup();
    std::cout << "planner created." << '\n';

    constexpr int max_iters = { 10 };
    constexpr int min_waypoints { 6 };
    constexpr int max_waypoints { 10 };
    Eigen::MatrixXd waypoints_mat {};
    int num_waypoints {};
    bool waypoints_found { false };
    for (int iters { 0 }; iters < max_iters; ++iters) {
        std::cout << "iteration: " << iters << '\n';
        // clear previous data
        planner->clear();
        pdef->clearSolutionPaths();
        
        // solve planning problem
        std::cout << "solving planning problem..." << '\n';
        ob::PlannerStatus solved { planner->solve(1.0) };
        std::cout << "planner status: " << solved.asString() << '\n';
        std::cout << "success: " << (solved==ob::PlannerStatus::EXACT_SOLUTION ? "y" : "n") << '\n';
        if (!solved) {
            std::cout << "planning failure." << '\n';
            // return 1;
            continue;
        }
        auto* path { pdef->getSolutionPath()->as<og::PathGeometric>() };
        std::cout << "planning success, result:" << '\n';
        // path->printAsMatrix(std::cout);
        
        // prune path
        std::cout << "num points original: " << path->getStateCount() << '\n';
        // path->interpolate(path->getStateCount() * 4);
        path->subdivide();
        std::cout << "num points after interpolation: " << path->getStateCount() << '\n';
        constexpr double low_clearance { 1e-2 };
        std::vector<Eigen::VectorXd> waypoints {
            helpers::state_to_vector(path->getState(0))
        };
        num_waypoints = 1;
        for (int i { 1 }; i < path->getStateCount() - 1; ++i) {
            auto s { path->getState(i) };
            bool valid { si->getStateValidityChecker()->isValid(s) };
            double clearance { si->getStateValidityChecker()->clearance(s) };
            if (valid && (clearance < low_clearance)) {
                waypoints.push_back(helpers::state_to_vector(s));
                ++num_waypoints;
            }
        }
        waypoints.push_back(
            helpers::state_to_vector(path->getState(path->getStateCount() - 1))
        );
        ++num_waypoints;
        std::cout << "num points after pruning: " << num_waypoints << '\n';
        if ((num_waypoints >= min_waypoints) && (num_waypoints <= max_waypoints)) {
            waypoints_mat = Eigen::MatrixXd { constants::SO101_NUM_Q, num_waypoints };
            for (int i { 0 }; i < num_waypoints; ++i) {
                waypoints_mat.col(i) = waypoints[i];
            }
            waypoints_found = true;
            // path->printAsMatrix(std::cout);
            break;
        }
    }
    
    if (!waypoints_found) {
        std::cout << "valid waypoints not found" << '\n';
        return 1;
    }
    
    std::cout << waypoints_mat.transpose() << '\n';
    helpers::save_matrix(waypoints_mat, "waypoints.bin");

    // publish position trajectory
    Eigen::VectorXd times = Eigen::VectorXd::LinSpaced(num_waypoints, 0.0, 5.0);
    // std::cout << times << '\n';
    PiecewisePolynomial ompl_traj { PiecewisePolynomial<double>::FirstOrderHold(times, waypoints_mat) };
    helpers::publish_position_trajectory(ompl_traj, *context, plant, visualizer);
    helpers::user_input_quit();

    return 0;
}
