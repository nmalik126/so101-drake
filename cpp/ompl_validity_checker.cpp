#include "ompl_validity_checker.h"

#include "ompl_helpers.h"

#include <drake/planning/robot_diagram.h>
#include <drake/planning/collision_checker_params.h>
#include <drake/planning/scene_graph_collision_checker.h>
#include <drake/planning/collision_checker_context.h>
#include <drake/planning/robot_clearance.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/State.h>

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
    const double influence_distance
) : ob::StateValidityChecker(si)
  , gripper_link_index_ { diagram->plant().GetBodyByName("gripper_link").index() }
  , moving_jaw_index_ { diagram->plant().GetBodyByName("moving_jaw_so101_v1_link").index() }
  , influence_distance_ { influence_distance }
{
    auto so101 { diagram->plant().GetModelInstanceByName("so101_new_calib") };
    CollisionCheckerParams params {
        .model { diagram },
        .robot_model_instances { { so101 } },
        .edge_step_size { 0.01 }
    };

    checker_ = std::make_unique<SceneGraphCollisionChecker>(params);
    checker_->SetPaddingAllRobotEnvironmentPairs(8e-3);

    context_ = checker_->MakeStandaloneModelContext();
}

bool DrakeSO101ValidityChecker::isValid(const ob::State *state) const {
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
