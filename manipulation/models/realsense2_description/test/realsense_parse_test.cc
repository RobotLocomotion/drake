/*
@file
Parses and visualizes Realsense d415.

If you wish to visualize this mesh, in it's own terminal run:

 $ bazel-bin/tools/drake_visualizer

In a new terminal, run example of showing a Realsense d415:

 $ bazel run \
   //manipulation/models/realsense2_description:realsense_parse_test -- \
   --visualize=true

*/

#include <string>

#include <fmt/format.h>
#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

DEFINE_bool(
    visualize, false, "Publish model to Drake Visualizer.");

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

TEST_P(ParseTest, ParsesURDFAndVisualizes) {
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

  // MultibodyPlant always creates at least two model instances, one for the
  // world and one for a default model instance for unspecified modeling
  // elements. Finally, there is one model instance for the realsense model
  EXPECT_EQ(plant.num_model_instances(), 3);

  // Visualize via publishing, if requested
  if (FLAGS_visualize) {
    drake::log()->info("Visualize: {}", object_name);
    Simulator<double>(*diagram).Initialize();
    auto context = diagram->CreateDefaultContext();
    diagram->Publish(*context);
  }
}

INSTANTIATE_TEST_SUITE_P(RealsenseD415, ParseTest, testing::Values("d415"));

}  // namespace
}  // namespace manipulation
}  // namespace drake
