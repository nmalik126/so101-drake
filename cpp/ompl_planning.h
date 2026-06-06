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

std::optional<Eigen::MatrixXd> GenerateWaypoints(
    const MultibodyPlant<double>& plant,
    std::shared_ptr<RobotDiagram<double>> diagram,
    const Eigen::VectorXd q_goal
);

} // namespace ompl
} // namespace motion_planning
