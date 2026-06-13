#pragma once

#include <drake/common/trajectories/bspline_trajectory.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/planning/robot_diagram.h>

#include <Eigen/Dense>

#include <optional>

using drake::trajectories::BsplineTrajectory;
using drake::multibody::MultibodyPlant;
using drake::planning::RobotDiagram;

namespace motion_planning {
namespace trajectory_optimization {

std::optional<BsplineTrajectory<double>> GenerateTrajectory(
    const MultibodyPlant<double>& plant,
    std::shared_ptr<RobotDiagram<double>> diagram,
    const Eigen::MatrixXd waypoints
);

} // namespace trajectory_optimization
} // namespace motion_planning
