#include "drake/geometry/scene_graph_config.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/yaml/yaml_io.h"

namespace drake {
namespace geometry {
namespace {

using yaml::LoadYamlString;
using yaml::SaveYamlString;

const char* const kExampleConfig = R"""(
default_proximity_properties:
  compliance_type: compliant
  hydroelastic_modulus: 2.0
  resolution_hint: 3.0
  slab_thickness: 4.0
  dynamic_friction: 5.0
  static_friction: 6.0
  hunt_crossley_dissipation: 7.0
  relaxation_time: 8.0
  point_stiffness: 9.0
)""";

GTEST_TEST(SceneGraphConfigTest, YamlTest) {
  const auto config = LoadYamlString<SceneGraphConfig>(kExampleConfig);
  const auto& props = config.default_proximity_properties;
  EXPECT_EQ(props.compliance_type, "compliant");
  EXPECT_EQ(props.hydroelastic_modulus, 2);
  EXPECT_EQ(props.resolution_hint, 3);
  EXPECT_EQ(props.slab_thickness, 4);
  EXPECT_EQ(props.dynamic_friction, 5);
  EXPECT_EQ(props.static_friction, 6);
  EXPECT_EQ(props.hunt_crossley_dissipation, 7);
  EXPECT_EQ(props.relaxation_time, 8);
  EXPECT_EQ(props.point_stiffness, 9);
  EXPECT_EQ("\n" + SaveYamlString(config), kExampleConfig);
}

GTEST_TEST(SceneGraphConfigTest, ValidDefault) {
  const SceneGraphConfig kDefault;
  EXPECT_NO_THROW(kDefault.ValidateOrThrow());
}

GTEST_TEST(SceneGraphConfigTest, ValidateCompliance) {
  SceneGraphConfig config;
  auto& props = config.default_proximity_properties;
  props.compliance_type = "nope";
  DRAKE_EXPECT_THROWS_MESSAGE(config.ValidateOrThrow(),
                              "Unknown hydroelastic_type: 'nope'");
}

GTEST_TEST(SceneGraphConfigTest, ValidateModulus) {
  SceneGraphConfig config;
  auto& props = config.default_proximity_properties;
  props.hydroelastic_modulus = 0;
  DRAKE_EXPECT_THROWS_MESSAGE(
      config.ValidateOrThrow(),
      "Invalid scene graph configuration:"
      " 'hydroelastic_modulus' \\(0\\) must be a positive value.");
  props.hydroelastic_modulus = std::numeric_limits<double>::quiet_NaN();
  // TODO(#21167) document a disposition for NaN.
}

GTEST_TEST(SceneGraphConfigTest, ValidateRezHint) {
  SceneGraphConfig config;
  auto& props = config.default_proximity_properties;
  props.resolution_hint = 0;
  DRAKE_EXPECT_THROWS_MESSAGE(
      config.ValidateOrThrow(),
      "Invalid scene graph configuration:"
      " 'resolution_hint' \\(0\\) must be a positive, finite value.");
  props.resolution_hint = std::numeric_limits<double>::quiet_NaN();
  DRAKE_EXPECT_THROWS_MESSAGE(
      config.ValidateOrThrow(),
      "Invalid scene graph configuration:"
      " 'resolution_hint' \\(nan\\) must be a positive, finite value.");
}

GTEST_TEST(SceneGraphConfigTest, ValidateSlabThickness) {
  SceneGraphConfig config;
  auto& props = config.default_proximity_properties;
  props.slab_thickness = 0;
  DRAKE_EXPECT_THROWS_MESSAGE(
      config.ValidateOrThrow(),
      "Invalid scene graph configuration:"
      " 'slab_thickness' \\(0\\) must be a positive, finite value.");
  props.slab_thickness = std::numeric_limits<double>::quiet_NaN();
  DRAKE_EXPECT_THROWS_MESSAGE(
      config.ValidateOrThrow(),
      "Invalid scene graph configuration:"
      " 'slab_thickness' \\(nan\\) must be a positive, finite value.");
}

GTEST_TEST(SceneGraphConfigTest, ValidateDynamicFriction) {
  SceneGraphConfig config;
  auto& props = config.default_proximity_properties;
  props.dynamic_friction = -1;
  DRAKE_EXPECT_THROWS_MESSAGE(
      config.ValidateOrThrow(),
      "Invalid scene graph configuration:"
      " 'dynamic_friction' \\(-1\\) must be a non-negative value.");
  props.dynamic_friction = std::numeric_limits<double>::quiet_NaN();
  // TODO(#21167) document a disposition for NaN.
}

GTEST_TEST(SceneGraphConfigTest, ValidateStaticFriction) {
  SceneGraphConfig config;
  auto& props = config.default_proximity_properties;
  props.static_friction = -1;
  DRAKE_EXPECT_THROWS_MESSAGE(
      config.ValidateOrThrow(),
      "Invalid scene graph configuration:"
      " 'static_friction' \\(-1\\) must be a non-negative value.");
  props.static_friction = std::numeric_limits<double>::quiet_NaN();
  // TODO(#21167) document a disposition for NaN.
}

GTEST_TEST(SceneGraphConfigTest, ValidateHuntCrossley) {
  SceneGraphConfig config;
  auto& props = config.default_proximity_properties;
  props.hunt_crossley_dissipation = -1;
  DRAKE_EXPECT_THROWS_MESSAGE(
      config.ValidateOrThrow(),
      "Invalid scene graph configuration:"
      " 'hunt_crossley_dissipation' \\(-1\\) must be a non-negative"
      " value.");
  props.hunt_crossley_dissipation = std::numeric_limits<double>::quiet_NaN();
  // TODO(#21167) document a disposition for NaN.
}

GTEST_TEST(SceneGraphConfigTest, ValidateRelaxationTime) {
  SceneGraphConfig config;
  auto& props = config.default_proximity_properties;
  props.relaxation_time = -1;
  DRAKE_EXPECT_THROWS_MESSAGE(
      config.ValidateOrThrow(),
      "Invalid scene graph configuration:"
      " 'relaxation_time' \\(-1\\) must be a non-negative, finite value.");
  props.relaxation_time = std::numeric_limits<double>::quiet_NaN();
  DRAKE_EXPECT_THROWS_MESSAGE(
      config.ValidateOrThrow(),
      "Invalid scene graph configuration:"
      " 'relaxation_time' \\(nan\\) must be a non-negative, finite value.");
}

GTEST_TEST(SceneGraphConfigTest, ValidatePointStiffness) {
  SceneGraphConfig config;
  auto& props = config.default_proximity_properties;
  props.point_stiffness = -1;
  DRAKE_EXPECT_THROWS_MESSAGE(
      config.ValidateOrThrow(),
      "Invalid scene graph configuration:"
      " 'point_stiffness' \\(-1\\) must be a positive value.");
  props.point_stiffness = std::numeric_limits<double>::quiet_NaN();
  // TODO(#21167) document a disposition for NaN.
}

GTEST_TEST(SceneGraphConfigTest, ValidateCoulombFriction) {
  SceneGraphConfig config;
  auto& props = config.default_proximity_properties;

  // This configuration fails a pre-condition of CoulombFriction. We do not
  // test them all here; it has its own tests.
  props.static_friction = 0;
  DRAKE_EXPECT_THROWS_MESSAGE(
      config.ValidateOrThrow(),
      "The given dynamic friction \\(0.5\\) is greater than"
      " the given static friction \\(0\\); dynamic friction must be"
      " less than or equal to static friction.");

  // This configuration fails a pre-condition of DefaultProximityProperties.
  props.static_friction.reset();
  DRAKE_EXPECT_THROWS_MESSAGE(
      config.ValidateOrThrow(),
      "Invalid scene graph configuration:"
      " either both 'static_friction' \\(nullopt\\) and"
      " 'dynamic_friction' \\(0.5\\) must have a value, or neither.");
}

}  // namespace
}  // namespace geometry
}  // namespace drake
