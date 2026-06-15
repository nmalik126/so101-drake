#pragma once

#include "constants.h"

#include <drake/common/trajectories/bspline_trajectory.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/planning/robot_diagram.h>
#include <drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h>
#include <drake/planning/collision_checker_params.h>
#include <drake/planning/scene_graph_collision_checker.h>
#include <drake/planning/collision_checker_context.h>

#include <Eigen/Dense>

#include <optional>
#include <memory>

using drake::trajectories::BsplineTrajectory;
using drake::multibody::MultibodyPlant;
using drake::planning::RobotDiagram;
using drake::planning::trajectory_optimization::KinematicTrajectoryOptimization;
using drake::planning::CollisionCheckerParams;
using drake::planning::SceneGraphCollisionChecker;
using drake::planning::CollisionCheckerContext;

namespace motion_planning {
namespace trajectory_optimization {

std::optional<BsplineTrajectory<double>> GenerateTrajectory(
    const MultibodyPlant<double>& plant,
    std::shared_ptr<RobotDiagram<double>> diagram,
    const Eigen::MatrixXd waypoints
);

class SO101TrajOpt final {
public:

    explicit SO101TrajOpt(
        std::shared_ptr<RobotDiagram<double>> diagram
    );

    SO101TrajOpt(const SO101TrajOpt&) = delete;
    SO101TrajOpt& operator=(const SO101TrajOpt&) = delete;
    SO101TrajOpt(SO101TrajOpt&&) = delete;
    SO101TrajOpt& operator=(SO101TrajOpt&&) = delete;

    void set_waypoints(const Eigen::MatrixXd waypoints);

    std::optional<BsplineTrajectory<double>> solve();

private:

    std::shared_ptr<RobotDiagram<double>> diagram_;
    std::unique_ptr<KinematicTrajectoryOptimization> trajopt_;
    std::unique_ptr<SceneGraphCollisionChecker> checker_;
    std::shared_ptr<CollisionCheckerContext> context_;
    static constexpr int n_desired_waypoints_ { constants::TRAJOPT_N_WAYPOINTS };
    static constexpr int bspline_basis_order_ { 4 };

};

} // namespace trajectory_optimization
} // namespace motion_planning
