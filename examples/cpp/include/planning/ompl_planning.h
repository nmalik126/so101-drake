#pragma once

#include "constants.h"

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/planning/robot_diagram.h>
#include <drake/planning/scene_graph_collision_checker.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/Planner.h>
#include <ompl/base/ProblemDefinition.h>

#include <Eigen/Dense>

#include <optional>
#include <memory>

using drake::multibody::MultibodyPlant;
using drake::planning::RobotDiagram;
using drake::planning::SceneGraphCollisionChecker;

namespace ob = ompl::base;

namespace motion_planning {
namespace ompl {

namespace detail {

    std::optional<Eigen::MatrixXd> GenerateWaypointsImpl(
        const MultibodyPlant<double>& plant,
        std::shared_ptr<RobotDiagram<double>> diagram,
        const Eigen::VectorXd q_start,
        const Eigen::VectorXd q_goal
    );

} // namespace detail

inline std::optional<Eigen::MatrixXd> GenerateWaypoints(
    const MultibodyPlant<double>& plant,
    std::shared_ptr<RobotDiagram<double>> diagram,
    const Eigen::VectorXd q_goal
) {
    auto context { diagram->CreateDefaultContext() };
    const auto& fixed_plant_context { plant.GetMyContextFromRoot(*context) };
    const auto q_start { plant.GetPositions(fixed_plant_context) };

    return detail::GenerateWaypointsImpl(plant, diagram, q_start, q_goal);
}

inline std::optional<Eigen::MatrixXd> GenerateWaypoints(
    const MultibodyPlant<double>& plant,
    std::shared_ptr<RobotDiagram<double>> diagram,
    const Eigen::VectorXd q_start,
    const Eigen::VectorXd q_goal
) {
    return detail::GenerateWaypointsImpl(plant, diagram, q_start, q_goal);
}

class SO101OMPL final {
public:

    explicit SO101OMPL(
        std::shared_ptr<RobotDiagram<double>> diagram,
        std::shared_ptr<SceneGraphCollisionChecker> checker = nullptr
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
    static constexpr int n_desired_waypoints_ { constants::TRAJOPT_N_WAYPOINTS };

};

} // namespace ompl
} // namespace motion_planning
