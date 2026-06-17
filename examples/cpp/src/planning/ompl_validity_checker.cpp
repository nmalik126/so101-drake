#include "planning/ompl_validity_checker.h"

#include "planning/ompl_helpers.h"

#include <drake/planning/robot_diagram.h>
#include <drake/planning/collision_checker_params.h>
#include <drake/planning/scene_graph_collision_checker.h>
#include <drake/planning/collision_checker_context.h>
#include <drake/planning/robot_clearance.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>

#include <Eigen/Dense>

#include <memory>
#include <limits>

using drake::planning::RobotDiagram;
using drake::planning::CollisionCheckerParams;
using drake::planning::SceneGraphCollisionChecker;
using drake::planning::CollisionCheckerContext;
using drake::planning::RobotCollisionType;

namespace ob = ompl::base;

DrakeSO101ValidityChecker::DrakeSO101ValidityChecker(
    const ob::SpaceInformationPtr& si,
    std::shared_ptr<RobotDiagram<double>> diagram,
    std::shared_ptr<SceneGraphCollisionChecker> checker,
    const double influence_distance
) : ob::StateValidityChecker(si)
  , gripper_link_index_ { diagram->plant().GetBodyByName("gripper_link").index() }
  , moving_jaw_index_ { diagram->plant().GetBodyByName("moving_jaw_so101_v1_link").index() }
  , checker_ { checker }
  , context_ { checker->MakeStandaloneModelContext() }
  , influence_distance_ { influence_distance }
{}

bool DrakeSO101ValidityChecker::isValid(const ob::State *state) const {
    if (si_->distance(state, start_state_) < 1e-5)
        return true;
    return checker_->CheckContextConfigCollisionFree(
        context_.get(),
        helpers::state_to_vector(state)
    );
}

double DrakeSO101ValidityChecker::clearance(const ob::State *state) const {
    auto robot_clearance {
        checker_->CalcContextRobotClearance(
            context_.get(),
            helpers::state_to_vector(state),
            influence_distance_
        )
    };
    auto distances { robot_clearance.distances() };

    double min_dist { std::numeric_limits<double>::max() };
    auto collision_types { robot_clearance.collision_types() };
    auto robot_indices { robot_clearance.robot_indices() };
    for (int i { 0 }; i < distances.size(); ++i) {
        double dist { distances(i) };
        RobotCollisionType t { collision_types[i] };
        auto idx { robot_indices[i] };
        if (
            (t == RobotCollisionType::kEnvironmentCollision) &&
            ((idx == gripper_link_index_) || (idx == moving_jaw_index_)) &&
            (dist < min_dist)
        )
            min_dist = dist;
    }

    return min_dist;
}
