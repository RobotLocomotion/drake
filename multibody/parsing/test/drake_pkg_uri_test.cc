/*
@file
Parses and visualizes sdf/urdf.

If you wish to visualize this mesh, in its own terminal run:

 $ bazel-bin/tools/drake_visualizer

In a new terminal, run example to visualize model:

 $ bazel run \
   //multibody/parsing/test:drake_pkg_uri_test -- \
   --visualize=true --visualize_sec=1

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
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

DEFINE_bool(
    visualize, false, "Publish model to Drake Visualizer.");
DEFINE_double(
    visualize_sec, 0., "Amount of time to pause between each model to "
    "visualize in Drake Visualizer.");

namespace drake {
namespace multibody {
namespace {

using geometry::DrakeVisualizer;
using geometry::SceneGraph;
using multibody::AddMultibodyPlantSceneGraph;
using multibody::Parser;
using systems::DiagramBuilder;
using systems::Simulator;

class ParseTest : public testing::TestWithParam<std::string> {};

TEST_P(ParseTest, ParsesSdfAndVisualizes) {
  const std::string object_name = GetParam();
  const std::string filename = FindResourceOrThrow(fmt::format(
      "drake/multibody/parsing/test/drake_pkg_uri_test/{}",
      object_name));

  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
  // Record the default number of models before a new model is added.
  const int default_num_models = plant.num_model_instances();

  // Check to ensure model is parsable.
  EXPECT_NO_THROW(Parser(&plant).AddModelFromFile(filename));

  // Ensure there was exactly one model instance added for the new model.
  EXPECT_EQ(plant.num_model_instances() - default_num_models, 1);

  // Visualize via publishing, if requested.
  if (FLAGS_visualize || FLAGS_visualize_sec > 0.0) {
    DrakeVisualizer::AddToBuilder(&builder, scene_graph);
    plant.Finalize();
    auto diagram = builder.Build();
    drake::log()->info("Visualize: {}", object_name);
    Simulator<double> simulator(*diagram);
    simulator.Initialize();
    diagram->Publish(simulator.get_context());
    if (FLAGS_visualize_sec > 0.0) {
      drake::log()->info("Pausing for {} sec...", FLAGS_visualize_sec);
      std::this_thread::sleep_for(
          std::chrono::milliseconds(
              static_cast<int>(1000 * FLAGS_visualize_sec)));
    }
  }
}


INSTANTIATE_TEST_SUITE_P(DRAKE_URI_MODEL, ParseTest,
  testing::Values("cube_visual.sdf", "cube_visual.urdf"));

}  // namespace
}  // namespace multibody
}  // namespace drake
