#include "drake/geometry/scene_graph_config.h"

#include <gtest/gtest.h>

#include "drake/common/yaml/yaml_io.h"

namespace drake {
namespace geometry {
namespace {

using yaml::LoadYamlString;

const char* const kExampleConfig = R"""(
hydroelastication:
  enabled: true
  minimum_primitive_size: 1
  default_hydroelastic_modulus: 2
  default_mesh_resolution_hint: 3
  default_slab_thickness: 4
  default_hunt_crossley_dissipation: 5
  default_dynamic_friction: 6
  default_static_friction: 7
)""";

GTEST_TEST(SceneGraphConfigTest, YamlTest) {
  const auto config = LoadYamlString<SceneGraphConfig>(kExampleConfig);
  EXPECT_EQ(config.hydroelastication.enabled, true);
  EXPECT_EQ(config.hydroelastication.minimum_primitive_size, 1);
  EXPECT_EQ(config.hydroelastication.default_hydroelastic_modulus, 2);
  EXPECT_EQ(config.hydroelastication.default_mesh_resolution_hint, 3);
  EXPECT_EQ(config.hydroelastication.default_slab_thickness, 4);
  EXPECT_EQ(config.hydroelastication.default_hunt_crossley_dissipation, 5);
  EXPECT_EQ(config.hydroelastication.default_dynamic_friction, 6);
  EXPECT_EQ(config.hydroelastication.default_static_friction, 7);
}

}  // namespace
}  // namespace geometry
}  // namespace drake


