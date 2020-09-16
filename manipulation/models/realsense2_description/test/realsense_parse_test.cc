/**
@file
Parses and visualizes Realsense d415.

An example of showing a Realsense d415 at a 5s interval:

 $ bazel run //manipulation/models/realsense2_description:realsense_parse_test -- --visualize_sec=5

*/

#include <chrono>
#include <string>
#include <thread>

#include <fmt/format.h>
#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

DEFINE_double(
    visualize_sec, 0., "Pause and visualize to Drake Visualizer.");

namespace drake {
namespace manipulation {
namespace {

using geometry::ConnectDrakeVisualizer;
using geometry::SceneGraph;
using multibody::AddMultibodyPlantSceneGraph;
using multibody::Parser;
using systems::DiagramBuilder;
using systems::Simulator;

class ParseTest : public testing::TestWithParam<std::string> {};

TEST_P(ParseTest, Quantities) {
  const std::string object_name = GetParam();
  const std::string filename = FindResourceOrThrow(fmt::format(
      "drake/manipulation/models/realsense2_description/urdf/{}.urdf",
      object_name));

  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);

  // Check to ensure URDF is parsable
  EXPECT_NO_THROW(Parser(&plant).AddModelFromFile(filename));

  ConnectDrakeVisualizer(&builder, scene_graph);
  plant.Finalize();
  auto diagram = builder.Build();

  // Expect three model instances: world, modeling elements, and
  // the realsense model itself
  EXPECT_EQ(plant.num_model_instances(), 3);

  // Expect the world body, and three realsense link bodies
  EXPECT_EQ(plant.num_bodies(), 4);

  // Visualize via publishing, if requested
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

INSTANTIATE_TEST_SUITE_P(Both, ParseTest, testing::Values("test_d415_camera"));

}  // namespace
}  // namespace manipulation
}  // namespace drake
