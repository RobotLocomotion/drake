/*
@file
Parses and resolves manifest-relative mesh URIs in sdf/urdf.
*/

#include <string>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {
namespace {

using multibody::AddMultibodyPlantSceneGraph;
using multibody::Parser;
using systems::DiagramBuilder;

class DrakeManifestResolutionTest : public testing::TestWithParam<std::string> {
};

// Verify that an SDF model containing mesh URIs rooted at the top-level Drake
// package manifest are parsed and resolved successfully.
TEST_P(DrakeManifestResolutionTest, ResolvesMeshUris) {
  const std::string object_name = GetParam();
  const std::string filename = FindResourceOrThrow(fmt::format(
      "drake/multibody/parsing/test/drake_manifest_resolution_test/{}",
      object_name));

  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
  // Record the default number of models before a new model is added.
  const int default_num_models = plant.num_model_instances();
  const auto& inspector = scene_graph.model_inspector();
  const int default_num_geometries = inspector.num_geometries();

  // Check to ensure model is parsable.
  // The call to AddModelFromFile should throw an exception if any of the
  // mesh URIs in the SDF model could not be resolved or loaded.
  EXPECT_NO_THROW(Parser(&plant).AddModelFromFile(filename));

  // Sanity checks to ensure there was exactly one model instance and one
  // geometry added for the new model.
  EXPECT_EQ(plant.num_model_instances() - default_num_models, 1);
  EXPECT_EQ(inspector.num_geometries() - default_num_geometries, 1);
}

INSTANTIATE_TEST_SUITE_P(DrakeUriModel, DrakeManifestResolutionTest,
  testing::Values("cube_visual.sdf", "cube_visual.urdf"));

}  // namespace
}  // namespace multibody
}  // namespace drake
