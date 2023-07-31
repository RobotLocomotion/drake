#include <string>

#include <fmt/format.h>
#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace manipulation {
namespace {

using geometry::SceneGraph;
using multibody::AddMultibodyPlantSceneGraph;
using multibody::Parser;
using systems::DiagramBuilder;
using systems::Simulator;

class ParseTest : public testing::TestWithParam<std::string> {};

TEST_P(ParseTest, ParsesUrdf) {
  const std::string object_name = GetParam();
  const std::string filename = FindResourceOrThrow(fmt::format(
      "drake/manipulation/models/realsense2_description/urdf/{}.urdf",
      object_name));

  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
  // Record the default number of models before a new model is added.
  const int default_num_models = plant.num_model_instances();

  // Check to ensure URDF is parsable.
  Parser parser(&plant);
  EXPECT_NO_THROW(parser.AddModels(filename));

  // Ensure there was exactly one model instance added for the new model.
  EXPECT_EQ(plant.num_model_instances() - default_num_models, 1);
}

// Note: We use a test suite here, even if currently for only one value, to
// allow for easily adding more models under test in the future without
// rewriting the test code.
INSTANTIATE_TEST_SUITE_P(RealsenseD415, ParseTest, testing::Values("d415"));

}  // namespace
}  // namespace manipulation
}  // namespace drake
