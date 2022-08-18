#include <cstdio>
#include <fstream>
#include <iostream>
#include <thread>

#include "drake/common/find_resource.h"
#include "drake/common/temp_directory.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/meshcat_animation.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/rgba.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/systems/analysis/simulator.h"

/* To test, you must manually run `bazel run //geometry:meshcat_manual_test`,
then follow the instructions on your console. */

namespace drake {
namespace geometry {
namespace {

using Eigen::Vector3d;
using math::RigidTransformd;
using math::RotationMatrixd;

int do_main() {
  drake::systems::DiagramBuilder<double> builder;
  drake::multibody::AddMultibodyPlantSceneGraphResult<double> result =
      drake::multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);
  auto& plant = result.plant;
  auto& scene_graph {result.scene_graph};
  auto parser {drake::multibody::Parser(&plant, &scene_graph)};
  drake::multibody::ModelInstanceIndex model_instance {
    parser.AddModelFromFile(
        FindResourceOrThrow("drake/manipulation/models/iiwa_description/urdf/"
                            "iiwa14_spheres_collision.urdf"))};
  plant.Finalize();
  auto meshcat = std::make_shared<drake::geometry::Meshcat>();
  drake::geometry::MeshcatVisualizer<double>::AddToBuilder(
      &builder, scene_graph, meshcat);
  const auto diagram = builder.Build();
  std::unique_ptr<drake::systems::Context<double>> diagram_context {
    diagram->CreateDefaultContext()};
  diagram->Publish(*diagram_context);
  std::string button_name {"Button X"};
  meshcat->AddButton(button_name);
  int number_of_clicks {0};
  while (true){
    number_of_clicks = meshcat->GetButtonClicks(button_name);
    drake::log()->info("number_of_clicks: {}", number_of_clicks);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}

}  // namespace
}  // namespace geometry
}  // namespace drake

int main() { return drake::geometry::do_main(); }
