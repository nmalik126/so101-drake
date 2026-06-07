#pragma once

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/planning/robot_diagram.h>

#include <Eigen/Dense>

#include <optional>
#include <memory>

using drake::multibody::MultibodyPlant;
using drake::planning::RobotDiagram;

namespace motion_planning {
namespace inverse_kinematics {

std::optional<Eigen::VectorXd> GenerateGoalConfig(
    const MultibodyPlant<double>& plant,
    std::shared_ptr<RobotDiagram<double>> diagram
);

std::optional<Eigen::VectorXd> GeneratePlaceConfig(
    const MultibodyPlant<double>& plant,
    std::shared_ptr<RobotDiagram<double>> diagram,
    const Eigen::VectorXd q_grasp_closed
);
    
} // namespace inverse_kinematics
} // namespace motion_planning
