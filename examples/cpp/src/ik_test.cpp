#include "helpers.h"
#include "kinematics/inverse_kinematics.h"

#include <iostream>
#include <memory>

using motion_planning::inverse_kinematics::SO101InverseKinematicsPick;

int main() {
    std::cout << "IK Test" << std::endl;

    // create scenario
    std::cout << "creating scenario..." << std::endl;
    auto assets { helpers::generate_so101_brick_diagram(true, true) };
    auto& plant { *(assets.plant) };
    auto meshcat { assets.meshcat };
    auto diagram { assets.diagram };
    std::cout << "scenario created." << std::endl;

    // create context
    auto context { diagram->CreateDefaultContext() };
    auto& mutable_plant_context { plant.GetMyMutableContextFromRoot(context.get()) };

    // inverse kinematics
    std::cout << "running inverse kinematics..." << std::endl;
    SO101InverseKinematicsPick ik_pick { diagram };
    std::cout << "solving..." << std::endl;
    auto ik_pick_result { ik_pick.solve() };
    if (!ik_pick_result) {
        std::cout << "IK Failure. Exiting..." << std::endl;
        return 1;
    }
    const auto q_pick { ik_pick_result.value() };
    std::cout << "IK Success. Q Pick: " << q_pick.transpose() << std::endl;

    // publish result
    meshcat->Flush();
    plant.SetPositions(&mutable_plant_context, q_pick);
    diagram->ForcedPublish(*context);
    helpers::user_input_quit();

    return 0;
}
