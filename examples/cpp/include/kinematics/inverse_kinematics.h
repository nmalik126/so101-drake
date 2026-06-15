#pragma once

#include "constants.h"

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/planning/robot_diagram.h>
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/systems/framework/context.h>

#include <Eigen/Dense>

#include <optional>
#include <memory>

using drake::multibody::MultibodyPlant;
using drake::planning::RobotDiagram;
using drake::multibody::InverseKinematics;
using drake::systems::Context;

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

protected:

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

protected:

    void add_constraints() override;

};
    
} // namespace inverse_kinematics
} // namespace motion_planning
