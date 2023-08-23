#include "drake/geometry/scene_graph_config.h"

#include <gtest/gtest.h>

#include "drake/common/yaml/yaml_io.h"

namespace drake {
namespace geometry {
namespace {

using yaml::LoadYamlString;

const char* const kExampleConfig = R"""(
hydroelastic:
  enabled: true
  minimum_primitive_size: 1
  default_hydroelastic_modulus: 2
  default_mesh_resolution_hint: 3
  default_slab_thickness: 4
)""";

GTEST_TEST(SceneGraphConfigTest, YamlTest) {
  const auto config = LoadYamlString<SceneGraphConfig>(kExampleConfig);
  EXPECT_EQ(config.hydroelastic.enabled, true);
  EXPECT_EQ(config.hydroelastic.minimum_primitive_size, 1);
  EXPECT_EQ(config.hydroelastic.default_hydroelastic_modulus, 2);
  EXPECT_EQ(config.hydroelastic.default_mesh_resolution_hint, 3);
  EXPECT_EQ(config.hydroelastic.default_slab_thickness, 4);
}

}  // namespace
}  // namespace geometry
}  // namespace drake


