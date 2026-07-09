#include "helpers.h"

#include <drake/systems/framework/diagram_builder.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/math/rigid_transform.h>
#include <drake/math/rotation_matrix.h>

#include <iostream>

using drake::systems::DiagramBuilder;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::Parser;
using drake::math::RigidTransform;
using drake::math::RigidTransformd;
using drake::math::RotationMatrix;
using drake::math::RotationMatrixd;

int main() {
    std::cout << "Gripper Visualizer" << std::endl;

    DiagramBuilder<double> builder {};
    auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
    Parser parser { &plant, &scene_graph };
    parser.SetAutoRenaming(true);

    //[[ 0.73272478 -0.67711918  0.06799961 -0.10030148]
    // [-0.68033879 -0.73119597  0.04991503  0.06230986]
    // [ 0.01592264 -0.08283675 -0.99643584  0.12030827]
    // [ 0.          0.          0.          1.        ]]

    // const Eigen::Matrix3d grasp_rotation {
    //     {  0.99084952, -0.09741316,  0.09342336 },
    //     { -0.08173476, -0.98388558, -0.15902411 },
    //     { 0.10740898,   0.14993302, -0.98284456 }
    // };
    // const Eigen::Vector3d grasp_translation { -0.1035339, 0.08832988, 0.0781016 + 0.04 };

    const Eigen::Matrix3d grasp_rotation {
        {  0.73272478, -0.67711918,  0.06799961 },
        { -0.68033879, -0.73119597,  0.04991503 },
        {  0.01592264, -0.08283675, -0.99643584 }
    };
    const Eigen::Vector3d grasp_translation { -0.10030148, 0.06230986, 0.12030827 };
    
    const RigidTransform grasp_transform { 
        RotationMatrix { grasp_rotation } * RotationMatrix<double>::MakeXRotation(pi), 
        grasp_translation 
    };

    const RigidTransformd box_transform {
        RotationMatrixd::MakeZRotation(pi/2),
        Eigen::Vector3d { 0.015, 0, -0.1 }
    };

    auto mat = parser.AddModels(constants::model_paths::MAT)[0];
    auto box = parser.AddModels(constants::model_paths::BOX)[0];
    auto gripper = parser.AddModels(constants::model_paths::SO101_GRIPPER)[0];
    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("gripper_link", gripper),
        grasp_transform
    );
    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("box_link", box),
        grasp_transform * box_transform
    );

    plant.Finalize();
    
    auto meshcat { std::make_shared<Meshcat>() };
    auto& visualizer { MeshcatVisualizer<double>::AddToBuilder(
        &builder, 
        scene_graph, 
        meshcat
    ) };

    auto diagram { builder.Build() };
    auto context { diagram->CreateDefaultContext() };
    auto& mutable_plant_context { plant.GetMyMutableContextFromRoot(context.get()) };

    meshcat->Flush();
    plant.SetPositions(&mutable_plant_context, gripper, Eigen::VectorXd { { 1.5 } });
    diagram->ForcedPublish(*context);
    helpers::user_input_quit();

    return 0;
}
