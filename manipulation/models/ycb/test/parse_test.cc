/**
@file
Parses and visualizes YCB objects.

An example of showing all objects at a 5s interval:

    bazel run //manipulation/models/ycb:parse_test -- --visualize_sec=5

Showing the first object for 1s:

    bazel run //manipulation/models/ycb:parse_test -- \
        --visualize_sec=1 --gtest_filter='*0'
*/

#include <chrono>
#include <string>
#include <thread>

#include <fmt/format.h>
#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

// N.B. We choose to pause because using stdin w/ `bazel run` is painful.
DEFINE_double(
    visualize_sec, 0., "Pause and visualize to Drake Visualizer.");

namespace drake {
namespace manipulation {
namespace {

using geometry::DrakeVisualizerd;
using geometry::SceneGraph;
using multibody::AddMultibodyPlantSceneGraph;
using multibody::MultibodyPlant;
using multibody::ModelInstanceIndex;
using multibody::Parser;
using systems::DiagramBuilder;
using systems::Simulator;

class ParseTest : public testing::TestWithParam<std::string> {};

TEST_P(ParseTest, Quantities) {
  const std::string object_name = GetParam();
  const std::string filename = FindResourceOrThrow(fmt::format(
      "drake/manipulation/models/ycb/sdf/{}.sdf", object_name));

  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
  Parser(&plant).AddModelFromFile(filename);
  DrakeVisualizerd::AddToBuilder(&builder, scene_graph);
  plant.Finalize();
  auto diagram = builder.Build();

  // MultibodyPlant always creates at least two model instances, one for the
  // world and one for a default model instance for unspecified modeling
  // elements. Finally, there is a model instance for each YCB sdf file.
  EXPECT_EQ(plant.num_model_instances(), 3);

  // Each object has two bodies, the world body and the object body.
  EXPECT_EQ(plant.num_bodies(), 2);

  if (FLAGS_visualize_sec > 0.) {
    drake::log()->info("Visualize: {}", object_name);
    Simulator<double>(*diagram).Initialize();
    auto context = diagram->CreateDefaultContext();
    diagram->Publish(*context);
    drake::log()->info("Pausing for {} sec...", FLAGS_visualize_sec);
    std::this_thread::sleep_for(
        std::chrono::milliseconds(
            static_cast<int>(1000 * FLAGS_visualize_sec)));
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
