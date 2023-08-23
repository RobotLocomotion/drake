#include "drake/geometry/scene_graph_config.h"

#include <gtest/gtest.h>

#include "drake/common/yaml/yaml_io.h"

namespace drake {
namespace geometry {
namespace {

using yaml::LoadYamlString;
using yaml::SaveYamlString;

const char* const kExampleConfig = R"""(
default_proximity_properties:
  compliance_type: compliant
  compliance_type_rigid_fallback: false
  hydroelastic_modulus: 2.0
  mesh_resolution_hint: 3.0
  slab_thickness: 4.0
  hunt_crossley_dissipation: 5.0
  dynamic_friction: 6.0
  static_friction: 7.0
)""";

GTEST_TEST(SceneGraphConfigTest, YamlTest) {
  const auto config = LoadYamlString<SceneGraphConfig>(kExampleConfig);
  const auto& props = config.default_proximity_properties;
  EXPECT_EQ(props.compliance_type, "compliant");
  EXPECT_FALSE(props.compliance_type_rigid_fallback);
  EXPECT_EQ(props.hydroelastic_modulus, 2);
  EXPECT_EQ(props.mesh_resolution_hint, 3);
  EXPECT_EQ(props.slab_thickness, 4);
  EXPECT_EQ(props.hunt_crossley_dissipation, 5);
  EXPECT_EQ(props.dynamic_friction, 6);
  EXPECT_EQ(props.static_friction, 7);
  EXPECT_EQ("\n" + SaveYamlString(config), kExampleConfig);
}

}  // namespace
}  // namespace geometry
}  // namespace drake
