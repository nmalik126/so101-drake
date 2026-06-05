#include "ompl_validity_checker.h"

#include <drake/planning/robot_diagram.h>
#include <drake/planning/collision_checker_params.h>
#include <drake/planning/scene_graph_collision_checker.h>
#include <drake/planning/collision_checker_context.h>

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

namespace ob = ompl::base;

DrakeSO101ValidityChecker::DrakeSO101ValidityChecker(
    const ob::SpaceInformationPtr& si,
    std::shared_ptr<RobotDiagram<double>> diagram,
    const double influence_distance
) : ob::StateValidityChecker(si)
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
        state_to_vector_(state)
    );
}

double DrakeSO101ValidityChecker::clearance(const ob::State *state) const {
    auto distances {
        checker_->CalcContextRobotClearance(
            context_.get(),
            state_to_vector_(state),
            influence_distance_
        ).distances()             
    };
    // TODO: filter distances to kEnvironmentCollision and gripper
    if (distances.size() > 0)
        return distances.minCoeff();
    else
        return std::numeric_limits<double>::max();
}

Eigen::VectorXd DrakeSO101ValidityChecker::state_to_vector_(const ob::State *state) const {
    const auto* s = state->as<ob::RealVectorStateSpace::StateType>();
    Eigen::VectorXd v { num_q_ };
    for (int i { 0 }; i < num_q_; ++i) {
        v(i) = (*s)[i];
    }
    return v;
}
