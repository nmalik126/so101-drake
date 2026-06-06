#include "ompl_planning.h"

#include "constants.h"
#include "ompl_validity_checker.h"
#include "ompl_helpers.h"

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/planning/robot_diagram.h>

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

#include <optional>
#include <memory>

using drake::multibody::MultibodyPlant;
using drake::planning::RobotDiagram;

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace motion_planning {
namespace ompl {

std::optional<Eigen::MatrixXd> GenerateWaypoints(
    const MultibodyPlant<double>& plant,
    std::shared_ptr<RobotDiagram<double>> diagram,
    const Eigen::VectorXd q_goal
) {
    // create context
    auto context { diagram->CreateDefaultContext() };
    const auto& fixed_plant_context { plant.GetMyContextFromRoot(*context) };
    auto q_start { plant.GetPositions(fixed_plant_context) };

    // create space
    Eigen::VectorXd lower_bounds { plant.GetPositionLowerLimits() };
    Eigen::VectorXd upper_bounds { plant.GetPositionUpperLimits() };
    ob::RealVectorBounds bounds { constants::SO101_NUM_Q };
    for (int i { 0 }; i < constants::SO101_NUM_Q; ++i) {
        bounds.setLow(i, lower_bounds(i));
        bounds.setHigh(i, upper_bounds(i));
    }
    auto space { std::make_shared<ob::RealVectorStateSpace>(constants::SO101_NUM_Q) };
    space->setBounds(bounds);

    // create space information
    auto si { std::make_shared<ob::SpaceInformation>(space) };
    si->setStateValidityChecker(
        std::make_shared<DrakeSO101ValidityChecker>(si, diagram)
    );
    si->setup();

    // create problem definition
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

    // create planner
    ob::PlannerPtr planner { std::make_shared<og::RRTConnect>(si, true) };
    planner->setProblemDefinition(pdef);
    planner->setup();

    // init constants
    constexpr int max_iters = { 10 };
    constexpr int min_waypoints { 5 };
    constexpr int max_waypoints { 8 };
    constexpr double low_clearance { 1e-2 };
    
    // generate waypoints
    Eigen::MatrixXd waypoints_mat {};
    int num_waypoints {};
    bool waypoints_found { false };
    for (int iters { 0 }; iters < max_iters; ++iters) {
        // clear previous data
        planner->clear();
        pdef->clearSolutionPaths();
        
        // solve planning problem
        ob::PlannerStatus solved { planner->solve(1.0) };
        if (!solved) {
            continue;
        }
        auto* path { pdef->getSolutionPath()->as<og::PathGeometric>() };
        
        // prune path
        path->subdivide();
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
        if ((num_waypoints >= min_waypoints) && (num_waypoints <= max_waypoints)) {
            waypoints_mat = Eigen::MatrixXd { constants::SO101_NUM_Q, num_waypoints };
            for (int i { 0 }; i < num_waypoints; ++i) {
                waypoints_mat.col(i) = waypoints[i];
            }
            waypoints_found = true;
            break;
        }
    }

    if (waypoints_found)
        return waypoints_mat;
    else
        return std::nullopt;
}

} // namespace ompl
} // namespace motion_planning
