#include <drake/systems/framework/diagram_builder.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/geometry/meshcat.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/systems/analysis/simulator.h>

#include <Eigen/Dense>

#include <iostream>

using drake::systems::DiagramBuilder;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::Parser;
using drake::geometry::Meshcat;
using drake::geometry::MeshcatVisualizer;
using drake::systems::Simulator;

int main() {
    std::cout << "begin program" << '\n';

    // builder
    DiagramBuilder<double> builder {};
    std::cout << "diagram empty: " << (builder.empty() ? "true" : "false") << '\n';

    // plant and scene graph
    double time_step{ 1e-4 };
    auto [plant, scene_graph] { AddMultibodyPlantSceneGraph(&builder, time_step) };

    // parser
    Parser parser{ &plant, &scene_graph };
    std::string model_path = "/home/noor/SO-ARM100/Simulation/SO101/so101_new_calib_urdf_drake_hydro.urdf";
    std::cout << "parsing started..." << '\n';
    auto so101 = parser.AddModels(model_path)[0];
    std::cout << "parsing finished." << '\n';

    // weld frames
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link"));

    // finalize
    plant.Finalize();

    // meshcat
    auto meshcat{ std::make_shared<Meshcat>() };
    MeshcatVisualizer<double>::AddToBuilder(&builder, scene_graph, meshcat);

    // build
    auto diagram{ builder.Build() };

    // set positions
    auto context{ diagram->CreateDefaultContext() };
    auto& plant_context{ plant.GetMyMutableContextFromRoot(context.get()) };
    const Eigen::VectorXd q{{0, -1.75, 1.5, 1.0, 0, 0}};
    plant.SetPositions(&plant_context, q);
    plant.get_actuation_input_port().FixValue(
        &plant_context, 
        Eigen::VectorXd::Zero(plant.num_positions())
    );

    // simulator
    Simulator<double> simulator{ *diagram, std::move(context) };
    simulator.set_target_realtime_rate(1.0);
    std::cout << "simulation starting..." << '\n';
    meshcat->StartRecording();
    simulator.AdvanceTo(1.0);
    meshcat->StopRecording();
    meshcat->PublishRecording();
    std::cout << "simulation ended." << '\n';

    // wait for user to stop program
    std::cout << "Press Enter to quit..." << std::endl;
    std::string line;
    std::getline(std::cin, line);

    return 0;
}
