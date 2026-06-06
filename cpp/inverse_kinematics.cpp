#include "inverse_kinematics.h"

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/planning/robot_diagram.h>
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/solvers/solve.h>

#include <Eigen/Dense>

#include <memory>
#include <numbers>
#include <optional>

using drake::multibody::MultibodyPlant;
using drake::planning::RobotDiagram;
using drake::multibody::InverseKinematics;
using drake::solvers::Solve;

using std::numbers::pi;

namespace motion_planning {
namespace inverse_kinematics {

std::optional<Eigen::VectorXd> GenerateGoalConfig(
    const MultibodyPlant<double>& plant,
    std::shared_ptr<RobotDiagram<double>> diagram
) {
    // create context
    auto context { diagram->CreateDefaultContext() };
    const auto& fixed_plant_context { plant.GetMyContextFromRoot(*context) };
    auto& mutable_plant_context { plant.GetMyMutableContextFromRoot(context.get()) };

    // get frames
    auto q0 { plant.GetPositions(fixed_plant_context) };
    auto so101 { plant.GetModelInstanceByName("so101_new_calib") };
    const auto& gripper_frame { plant.GetFrameByName("gripper_link", so101) };
    auto box { plant.GetModelInstanceByName("box") };
    const auto& box_frame { plant.GetFrameByName("box_link", box) };

    // configure inverse kinematics
    InverseKinematics ik { plant, &mutable_plant_context, true };
    constexpr double grasp_tolerance { 1e-3 };
    ik.AddPositionConstraint(
        gripper_frame,
        Eigen::Vector3d { 0.02, 0.0, -0.1 },
        box_frame,
        Eigen::Vector3d { -grasp_tolerance, 0.0, -grasp_tolerance },
        Eigen::Vector3d { grasp_tolerance, std::numeric_limits<double>::max(), grasp_tolerance }
    );
    ik.AddPositionConstraint(
        gripper_frame,
        Eigen::Vector3d { -0.02, 0.0, -0.1 },
        box_frame,
        Eigen::Vector3d { -grasp_tolerance, std::numeric_limits<double>::lowest(), -grasp_tolerance },
        Eigen::Vector3d { grasp_tolerance, 0.0, grasp_tolerance }
    );
    ik.AddMinimumDistanceLowerBoundConstraint(8e-3, 1e-1);
    auto* prog { ik.get_mutable_prog() };
    const auto& q { ik.q() };
    prog->AddQuadraticErrorCost(1.0, q0, q);
    prog->AddBoundingBoxConstraint(pi/8, pi/2, q(5, 0));
    prog->SetInitialGuess(q, q0);

    // solve inverse kinematics
    auto result { Solve(ik.prog()) };
    if (result.is_success())
        return result.GetSolution(q);
    else
        return std::nullopt;
}
    
} // namespace inverse_kinematics
} // namespace motion_planning
