#pragma once

#include "constants.h"
#include "planning/ompl_validity_checker.h"

#include <drake/planning/robot_diagram.h>
#include <drake/planning/scene_graph_collision_checker.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/Planner.h>
#include <ompl/base/ProblemDefinition.h>

#include <Eigen/Dense>

#include <optional>
#include <memory>

using drake::planning::RobotDiagram;
using drake::planning::SceneGraphCollisionChecker;

namespace ob = ompl::base;

namespace motion_planning {
namespace ompl {

class SO101OMPL final {
public:

    explicit SO101OMPL(
        std::shared_ptr<RobotDiagram<double>> diagram,
        std::shared_ptr<SceneGraphCollisionChecker> checker
    );

    SO101OMPL(const SO101OMPL&) = delete;
    SO101OMPL& operator=(const SO101OMPL&) = delete;
    SO101OMPL(SO101OMPL&&) = delete;
    SO101OMPL& operator=(SO101OMPL&&) = delete;
    
    void set_pdef(
        const Eigen::VectorXd q_start,
        const Eigen::VectorXd q_goal
    );

    std::optional<Eigen::MatrixXd> solve();
    
private:

    std::shared_ptr<RobotDiagram<double>> diagram_;
    std::shared_ptr<ob::RealVectorStateSpace> space_;
    std::shared_ptr<ob::SpaceInformation> si_;
    std::shared_ptr<ob::Planner> planner_;
    std::shared_ptr<ob::ProblemDefinition> pdef_;
    std::shared_ptr<DrakeSO101ValidityChecker> validity_checker_;
    static constexpr int n_desired_waypoints_ { constants::TRAJOPT_N_WAYPOINTS };

};

} // namespace ompl
} // namespace motion_planning
