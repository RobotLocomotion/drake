/*
@file
Parses and visualizes Realsense d415.

If you wish to visualize this mesh, in its own terminal run:

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
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

DEFINE_bool(
    visualize, false, "Publish model to Drake Visualizer.");

namespace drake {
namespace manipulation {
namespace {

using geometry::DrakeVisualizerd;
using geometry::SceneGraph;
using multibody::AddMultibodyPlantSceneGraph;
using multibody::Parser;
using systems::DiagramBuilder;
using systems::Simulator;

class ParseTest : public testing::TestWithParam<std::string> {};

TEST_P(ParseTest, ParsesUrdfAndVisualizes) {
  const std::string object_name = GetParam();
  const std::string filename = FindResourceOrThrow(fmt::format(
      "drake/manipulation/models/realsense2_description/urdf/{}.urdf",
      object_name));

  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
  // Record the default number of models before a new model is added.
  const int default_num_models = plant.num_model_instances();

  // Check to ensure URDF is parsable.
  EXPECT_NO_THROW(Parser(&plant).AddModelFromFile(filename));

  // Ensure there was exactly one model instance added for the new model.
  EXPECT_EQ(plant.num_model_instances() - default_num_models, 1);

  // Visualize via publishing, if requested.
  if (FLAGS_visualize) {
    DrakeVisualizerd::AddToBuilder(&builder, scene_graph);
    plant.Finalize();
    auto diagram = builder.Build();
    drake::log()->info("Visualize: {}", object_name);
    Simulator<double> simulator(*diagram);
    simulator.Initialize();
    diagram->Publish(simulator.get_context());
  }
}

// Note: We use a test suite here, even if currently for only one value, to
// allow for easily adding more models under test in the future without
// rewriting the test code.
INSTANTIATE_TEST_SUITE_P(RealsenseD415, ParseTest, testing::Values("d415"));

}  // namespace
}  // namespace manipulation
}  // namespace drake
