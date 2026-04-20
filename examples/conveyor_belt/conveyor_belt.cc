#include <memory>
#include <string>
#include <utility>

#include <gflags/gflags.h>

#include "drake/geometry/meshcat.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/visualization/visualization_config.h"
#include "drake/visualization/visualization_config_functions.h"

namespace drake {
namespace examples {
namespace conveyor_belt {
namespace {

DEFINE_double(max_time_step, 1e-3, "Simulation time step used for integrator.");

int do_main_continous_plant() {
  systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] =
      multibody::AddMultibodyPlantSceneGraph(&builder, FLAGS_max_time_step);
  std::string conveyor_belt_url =
      "package://drake/examples/conveyor_belt/conveyor_belt.sdf";
  multibody::Parser(&builder).AddModelsFromUrl(conveyor_belt_url);
  plant.Finalize();

  // Set up visualization
  auto meshcat = std::make_shared<geometry::Meshcat>();
  visualization::ApplyVisualizationConfig(
      visualization::VisualizationConfig{
          .default_proximity_color = geometry::Rgba{1, 0, 0, 0.25},
          .enable_alpha_sliders = true,
      },
      &builder, nullptr, nullptr, nullptr, meshcat);

  // Set up context
  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();
  std::unique_ptr<systems::Context<double>> context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(context.get());

  // Force visualization
  diagram->ForcedPublish(*context);

  // Set up simulator
  systems::Simulator<double> simulator(*diagram);
  simulator.set_target_realtime_rate(1.0);
  simulator.Initialize();
  meshcat->StartRecording();
  simulator.AdvanceTo(20.0);
  meshcat->PublishRecording();

  return 0;
}

}  // namespace
}  // namespace conveyor_belt
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  // Initialize gflags.
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::examples::conveyor_belt::do_main_continous_plant();
  return 0;
}
