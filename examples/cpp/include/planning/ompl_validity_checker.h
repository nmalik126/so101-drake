#pragma once

#include "constants.h"

#include <drake/planning/robot_diagram.h>
#include <drake/planning/scene_graph_collision_checker.h>
#include <drake/planning/collision_checker_context.h>
#include <drake/multibody/tree/multibody_tree_indexes.h>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/State.h>

#include <Eigen/Dense>

#include <memory>

using drake::planning::RobotDiagram;
using drake::planning::SceneGraphCollisionChecker;
using drake::planning::CollisionCheckerContext;
using drake::multibody::BodyIndex;

namespace ob = ompl::base;

class DrakeSO101ValidityChecker final : public ob::StateValidityChecker
{
public:

    explicit DrakeSO101ValidityChecker(
        const ob::SpaceInformationPtr& si,
        std::shared_ptr<RobotDiagram<double>> diagram,
        std::shared_ptr<SceneGraphCollisionChecker> checker,
        const double influence_distance = 5e0
    );

    bool isValid(const ob::State *state) const override;

    double clearance(const ob::State *state) const override;

    void set_start_state(ob::State* start_state) { start_state_ = start_state; }

private:

    std::shared_ptr<SceneGraphCollisionChecker> checker_;
    std::shared_ptr<CollisionCheckerContext> context_;
    const BodyIndex gripper_link_index_;
    const BodyIndex moving_jaw_index_;
    const double influence_distance_;
    ob::State* start_state_;
    static constexpr int num_q_ { constants::SO101_NUM_Q };
    
};
