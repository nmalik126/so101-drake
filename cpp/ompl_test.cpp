#include "helpers.h"
#include "constants.h"
#include "ompl_validity_checker.h"

#include <drake/planning/robot_diagram_builder.h>
#include <drake/geometry/meshcat.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/geometry/meshcat_visualizer_params.h>
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

#include <iostream>
#include <memory>
#include <limits>

using drake::planning::RobotDiagramBuilder;
using drake::geometry::Meshcat;
using drake::geometry::MeshcatVisualizer;
using drake::geometry::MeshcatVisualizerParams;
using drake::planning::RobotDiagram;

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

    // solve planning problem
    std::cout << "solving planning problem..." << '\n';
    ob::PlannerStatus solved { planner->solve(1.0) };
    if (solved) {
        std::cout << "planning success, result:" << '\n';
        std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath())->printAsMatrix(std::cout);
    }
    else {
        std::cout << "planning failure." << '\n';
    }

    return 0;
}
