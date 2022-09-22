/**
@file
Parses and visualizes YCB objects.

An example of showing all objects:

    bazel run //manipulation/models/ycb:parse_test -- --pause

Showing the first object:

    bazel run //manipulation/models/ycb:parse_test -- \
        --pause --gtest_filter='*0'
*/

#include <chrono>
#include <string>
#include <thread>

#include <fmt/format.h>
#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/scope_exit.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/test_utilities/meshcat_environment.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

DEFINE_bool(pause, false, "Show each object until the user clicks.");

namespace drake {
namespace manipulation {
namespace {

using geometry::GetTestEnvironmentMeshcat;
using geometry::Meshcat;
using geometry::MeshcatVisualizerd;
using geometry::SceneGraph;
using multibody::AddMultibodyPlantSceneGraph;
using multibody::MultibodyPlant;
using multibody::ModelInstanceIndex;
using multibody::Parser;
using systems::DiagramBuilder;
using systems::Simulator;

class ParseTest : public testing::TestWithParam<std::string> {};

void WaitForNextButtonClick() {
  std::shared_ptr<Meshcat> meshcat = GetTestEnvironmentMeshcat();
  constexpr char kButtonName[] = "Show Next Model";
  meshcat->AddButton(kButtonName);
  ScopeExit guard([&meshcat, kButtonName]() {
    meshcat->DeleteButton(kButtonName);
  });
  drake::log()->info(
      "Pausing until '{}' is clicked in the Meshcat control panel...",
      kButtonName);
  while (meshcat->GetButtonClicks(kButtonName) == 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

TEST_P(ParseTest, Quantities) {
  const std::string object_name = GetParam();
  const std::string filename = FindResourceOrThrow(fmt::format(
      "drake/manipulation/models/ycb/sdf/{}.sdf", object_name));

  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
  Parser(&plant).AddModels(filename);
  const auto& visualizer = MeshcatVisualizerd::AddToBuilder(
      &builder, scene_graph, GetTestEnvironmentMeshcat());
  plant.Finalize();
  auto diagram = builder.Build();
  ScopeExit guard([&visualizer]() {
    visualizer.Delete();
  });

  // MultibodyPlant always creates at least two model instances, one for the
  // world and one for a default model instance for unspecified modeling
  // elements. Finally, there is a model instance for each YCB sdf file.
  EXPECT_EQ(plant.num_model_instances(), 3);

  // Each object has two bodies, the world body and the object body.
  EXPECT_EQ(plant.num_bodies(), 2);

  // Display the object; optionally wait for user input.
  drake::log()->info("Visualize: {}", object_name);
  auto context = diagram->CreateDefaultContext();
  diagram->ForcedPublish(*context);
  if (FLAGS_pause) {
    WaitForNextButtonClick();
  }
}

INSTANTIATE_TEST_SUITE_P(Both, ParseTest, testing::Values(
    "003_cracker_box",
    "004_sugar_box",
    "005_tomato_soup_can",
    "006_mustard_bottle",
    "009_gelatin_box",
    "010_potted_meat_can"));

}  // namespace
}  // namespace manipulation
}  // namespace drake
