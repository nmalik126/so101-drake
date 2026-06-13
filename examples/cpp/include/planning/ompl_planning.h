#pragma once

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/planning/robot_diagram.h>

#include <Eigen/Dense>

#include <optional>
#include <memory>

using drake::multibody::MultibodyPlant;
using drake::planning::RobotDiagram;

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

} // namespace ompl
} // namespace motion_planning
