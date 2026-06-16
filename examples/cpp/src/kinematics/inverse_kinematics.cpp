#include "kinematics/inverse_kinematics.h"

#include "constants.h"

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/planning/robot_diagram.h>
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/solvers/solve.h>
#include <drake/math/rigid_transform.h>
#include <drake/math/roll_pitch_yaw.h>

#include <Eigen/Dense>

#include <memory>
#include <numbers>
#include <optional>

using drake::multibody::MultibodyPlant;
using drake::planning::RobotDiagram;
using drake::multibody::InverseKinematics;
using drake::solvers::Solve;
using drake::math::RigidTransform;
using drake::math::RollPitchYaw;

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

std::optional<Eigen::VectorXd> GeneratePlaceConfig(
    const MultibodyPlant<double>& plant,
    std::shared_ptr<RobotDiagram<double>> diagram,
    const Eigen::VectorXd q_grasp_closed
) {
    // create context
    auto context { diagram->CreateDefaultContext() };
    const auto& fixed_plant_context { plant.GetMyContextFromRoot(*context) };
    auto& mutable_plant_context { plant.GetMyMutableContextFromRoot(context.get()) };

    // get frames
    auto so101 { plant.GetModelInstanceByName("so101_new_calib") };
    const auto& gripper_frame { plant.GetFrameByName("gripper_link", so101) };

    // configure inverse kinematics
    InverseKinematics ik { plant, &mutable_plant_context, true };
    const drake::Vector3<double> T_mat_boxgoal { constants::transforms::X_MAT_BOXGOAL::T };
    const drake::Vector3<double> rpy_mat_boxgoal { constants::transforms::X_MAT_BOXGOAL::R };
    const RollPitchYaw R_mat_boxgoal { rpy_mat_boxgoal };
    const RigidTransform X_mat_boxgoal { R_mat_boxgoal, T_mat_boxgoal };
    ik.AddPositionConstraint(
        gripper_frame,
        // Eigen::Vector3d { { 0.015, 0, -0.12 } },
        Eigen::Vector3d { { 0, 0, -0.12 } },
        plant.world_frame(),
        T_mat_boxgoal,
        T_mat_boxgoal
    );
    ik.AddOrientationConstraint(
        gripper_frame,
        {},
        plant.world_frame(),
        {},
        0.0
    );
    ik.AddMinimumDistanceLowerBoundConstraint(8e-3, 1e-1);
    auto* prog { ik.get_mutable_prog() };
    const auto& q { ik.q() };
    prog->AddQuadraticErrorCost(1.0, q_grasp_closed, q);
    prog->SetInitialGuess(q, q_grasp_closed);

    // solve inverse kinematics
    auto result { Solve(ik.prog()) };
    if (result.is_success())
        return result.GetSolution(q);
    else
        return std::nullopt;
}

SO101InverseKinematics::SO101InverseKinematics(
    std::shared_ptr<RobotDiagram<double>> diagram,
    const Eigen::VectorXd q_init
) : diagram_ { diagram }
  , context_ { diagram->CreateDefaultContext() }
{
    const auto& plant { diagram->plant() };
    auto& mutable_plant_context { plant.GetMyMutableContextFromRoot(context_.get()) };
    ik_ = std::make_unique<InverseKinematics>(plant, &mutable_plant_context, true);
    set_initial_guess(q_init);
}

void SO101InverseKinematics::set_initial_guess(const Eigen::VectorXd q_init) {
    auto* prog { ik_->get_mutable_prog() };
    const auto& q { ik_->q() };
    prog->AddQuadraticErrorCost(1.0, q_init, q);
    prog->SetInitialGuess(q, q_init);
}

std::optional<Eigen::VectorXd> SO101InverseKinematics::solve() const {
    const auto& q { ik_->q() };
    auto result { Solve(ik_->prog()) };
    if (result.is_success())
        return result.GetSolution(q);
    else
        return std::nullopt;
}

void SO101InverseKinematicsPick::add_constraints() {
    // get frames
    const auto& plant { diagram_->plant() };
    auto so101 { plant.GetModelInstanceByName("so101_new_calib") };
    const auto& gripper_frame { plant.GetFrameByName("gripper_link", so101) };
    auto box { plant.GetModelInstanceByName("box") };
    const auto& box_frame { plant.GetFrameByName("box_link", box) };
    
    // set constraints
    constexpr double grasp_tolerance { 1e-3 };
    ik_->AddPositionConstraint(
        gripper_frame,
        Eigen::Vector3d { 0.02, 0.0, -0.1 },
        box_frame,
        Eigen::Vector3d { -grasp_tolerance, 0.0, -grasp_tolerance },
        Eigen::Vector3d { grasp_tolerance, std::numeric_limits<double>::max(), grasp_tolerance }
    );
    ik_->AddPositionConstraint(
        gripper_frame,
        Eigen::Vector3d { -0.02, 0.0, -0.1 },
        box_frame,
        Eigen::Vector3d { -grasp_tolerance, std::numeric_limits<double>::lowest(), -grasp_tolerance },
        Eigen::Vector3d { grasp_tolerance, 0.0, grasp_tolerance }
    );
    ik_->AddMinimumDistanceLowerBoundConstraint(8e-3, 1e-1);
    auto* prog { ik_->get_mutable_prog() };
    const auto& q { ik_->q() };
    prog->AddBoundingBoxConstraint(pi/8, pi/2, q(5, 0));
}

void SO101InverseKinematicsPlace::add_constraints() {
    // get frames
    const auto& plant { diagram_->plant() };
    auto so101 { plant.GetModelInstanceByName("so101_new_calib") };
    const auto& gripper_frame { plant.GetFrameByName("gripper_link", so101) };

    // set constraints
    const drake::Vector3<double> T_mat_boxgoal { constants::transforms::X_MAT_BOXGOAL::T };
    const drake::Vector3<double> rpy_mat_boxgoal { constants::transforms::X_MAT_BOXGOAL::R };
    const RollPitchYaw R_mat_boxgoal { rpy_mat_boxgoal };
    const RigidTransform X_mat_boxgoal { R_mat_boxgoal, T_mat_boxgoal };
    ik_->AddPositionConstraint(
        gripper_frame,
        Eigen::Vector3d { { 0.015, 0, -0.12 } },
        // Eigen::Vector3d { { 0, 0, -0.15 } },
        plant.world_frame(),
        T_mat_boxgoal,
        T_mat_boxgoal
    );
    ik_->AddOrientationConstraint(
        gripper_frame,
        {},
        plant.world_frame(),
        {},
        0.0
    );
    ik_->AddMinimumDistanceLowerBoundConstraint(8e-3, 1e-1);
}

} // namespace inverse_kinematics
} // namespace motion_planning
