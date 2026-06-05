#include "helpers.h"

#include <drake/systems/framework/diagram_builder.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/geometry/meshcat.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/geometry/shape_specification.h>
#include <drake/geometry/rgba.h>
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/solvers/solve.h>

#include <Eigen/Dense>

#include <iostream>
#include <memory>
#include <limits>
#include <numbers>

using drake::systems::DiagramBuilder;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::geometry::Meshcat;
using drake::geometry::MeshcatVisualizer;
using drake::geometry::Sphere;
using drake::geometry::Rgba;
using drake::multibody::InverseKinematics;
using drake::solvers::Solve;

using std::numbers::pi;

int main() {
    std::cout << "IK Test" << '\n';

    // init builder
    DiagramBuilder<double> builder {};
    
    // create plant, scene graph, and parser
    auto [plant, scene_graph] { AddMultibodyPlantSceneGraph(&builder, 1e-4) };

    // init scenario
    std::cout << "parsing started..." << '\n';
    helpers::generate_so101_brick_welded(plant, scene_graph);
    std::cout << "parsing finished." << '\n';

    // meshcat
    auto meshcat { std::make_shared<Meshcat>() };
    MeshcatVisualizer<double>::AddToBuilder(&builder, scene_graph, meshcat);

    // create context
    auto diagram { builder.Build() };
    auto context { diagram->CreateDefaultContext() };
    const auto& fixed_plant_context { plant.GetMyContextFromRoot(*context) };
    auto& mutable_plant_context { plant.GetMyMutableContextFromRoot(context.get()) };

    // draw sphere around box
    auto box { plant.GetModelInstanceByName("box") };
    const auto& box_body { plant.GetRigidBodyByName("box_link", box) };
    auto X_WGoal { plant.EvalBodyPoseInWorld(fixed_plant_context, box_body) };
    meshcat->SetObject("goal", Sphere { 0.02 }, Rgba { 0.1, 0.9, 0.1, 1 } );
    meshcat->SetTransform("goal", X_WGoal);

    // get frames
    auto q0 { plant.GetPositions(fixed_plant_context) };
    auto so101 { plant.GetModelInstanceByName("so101_new_calib") };
    const auto& gripper_frame { plant.GetFrameByName("gripper_link", so101) };
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
    std::cout << "solving IK..." << '\n';
    auto result { Solve(ik.prog()) };
    if (result.is_success())
        std::cout << "IK Success, q: \n" << result.GetSolution(q) << '\n';
    else
        std::cout << "IK Failure" << '\n';

    // publish result
    meshcat->Flush();
    diagram->ForcedPublish(*context);

    // wait for user to stop program
    helpers::user_input_quit();

    return 0;
}
