#pragma once

#include "constants.h"

#include <drake/planning/robot_diagram.h>
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/systems/framework/context.h>
#include <drake/math/rigid_transform.h>

#include <Eigen/Dense>

#include <optional>
#include <memory>

using drake::planning::RobotDiagram;
using drake::multibody::InverseKinematics;
using drake::systems::Context;
using drake::math::RigidTransformd;

namespace motion_planning {
namespace inverse_kinematics {

class SO101InverseKinematics {
public:

    explicit SO101InverseKinematics(
        std::shared_ptr<RobotDiagram<double>> diagram,
        const Eigen::VectorXd q_init = Eigen::VectorXd::Zero(constants::SO101_NUM_Q)
    );

    SO101InverseKinematics(const SO101InverseKinematics&) = delete;
    SO101InverseKinematics& operator=(const SO101InverseKinematics&) = delete;
    SO101InverseKinematics(SO101InverseKinematics&&) = delete;
    SO101InverseKinematics& operator=(SO101InverseKinematics&&) = delete;

    virtual ~SO101InverseKinematics() = default;

    void set_initial_guess(const Eigen::VectorXd q_init);

    std::optional<Eigen::VectorXd> solve() const;

protected:

    virtual void add_constraints() = 0;

    std::shared_ptr<RobotDiagram<double>> diagram_;
    std::unique_ptr<InverseKinematics> ik_;

private:

    std::unique_ptr<Context<double>> context_;

};
    
class SO101InverseKinematicsPick final : public SO101InverseKinematics {
public:

    explicit SO101InverseKinematicsPick(
        std::shared_ptr<RobotDiagram<double>> diagram
    ) : SO101InverseKinematics { diagram }
    {
        add_constraints();
    }

private:

    void add_constraints() override;

};

class SO101InverseKinematicsPlace final : public SO101InverseKinematics {
public:

    explicit SO101InverseKinematicsPlace(
        std::shared_ptr<RobotDiagram<double>> diagram
    ) : SO101InverseKinematics { diagram }
    {
        add_constraints();
    }

private:

    void add_constraints() override;

};

class SO101InverseKinematicsRandPick final : public SO101InverseKinematics {
public:

    explicit SO101InverseKinematicsRandPick(
        std::shared_ptr<RobotDiagram<double>> diagram,
        const RigidTransformd grasp_transform
    ) : SO101InverseKinematics { diagram }
      , grasp_transform { grasp_transform }
    {
        add_constraints();
    }

private:

    void add_constraints() override;

    const RigidTransformd grasp_transform;

};
    
} // namespace inverse_kinematics
} // namespace motion_planning
