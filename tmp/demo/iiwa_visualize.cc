#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace tmp {

using namespace drake::geometry;
using namespace drake::systems;
using namespace drake::multibody;

int DoMain(int argc, char** argv) {
  (void)argc; (void)argv;
  const std::string model_path =
      "drake/manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf";
  DiagramBuilder<double> builder;
  MultibodyPlant<double>* plant{};
  SceneGraph<double>* scene_graph{};
  std::tie(plant, scene_graph) = AddMultibodyPlantSceneGraph(&builder);
  Parser(plant).AddModelFromFile(FindResourceOrThrow(model_path));
  plant->Finalize();
  ConnectDrakeVisualizer(&builder, *scene_graph);
  auto diagram = builder.Build();
  Simulator<double>(*diagram).Initialize();
  return 0;
}

}  // namespace tmp
}  // namespace drake

int main(int argc, char** argv) {
  return drake::tmp::DoMain(argc, argv);
}
