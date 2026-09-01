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

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

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
      "Invalid scene graph configuration: 'hydroelastic_modulus' \\(0\\) is "
      "invalid. The hydroelastic modulus must be positive; given 0");
  props.hydroelastic_modulus = -1;
  DRAKE_EXPECT_THROWS_MESSAGE(
      config.ValidateOrThrow(),
      "Invalid scene graph configuration: 'hydroelastic_modulus' \\(-1\\) is "
      "invalid. The hydroelastic modulus must be positive; given -1");
  props.hydroelastic_modulus = kNan;
  DRAKE_EXPECT_THROWS_MESSAGE(
      config.ValidateOrThrow(),
      "Invalid scene graph configuration: 'hydroelastic_modulus' \\(nan\\) is "
      "invalid. The hydroelastic modulus must be positive; given nan");
  // +∞ is intentionally allowed for hydroelastic modulus.
  props.hydroelastic_modulus = kInf;
  EXPECT_NO_THROW(config.ValidateOrThrow());
}

GTEST_TEST(SceneGraphConfigTest, ValidateRezHint) {
  SceneGraphConfig config;
  auto& props = config.default_proximity_properties;
  props.resolution_hint = 0;
  DRAKE_EXPECT_THROWS_MESSAGE(
      config.ValidateOrThrow(),
      "Invalid scene graph configuration: 'resolution_hint' \\(0\\) is "
      "invalid. The resolution_hint must be positive and finite; given 0");
  props.resolution_hint = kNan;
  DRAKE_EXPECT_THROWS_MESSAGE(
      config.ValidateOrThrow(),
      "Invalid scene graph configuration: 'resolution_hint' \\(nan\\) is "
      "invalid. The resolution_hint must be positive and finite; given nan");
  props.resolution_hint = kInf;
  DRAKE_EXPECT_THROWS_MESSAGE(
      config.ValidateOrThrow(),
      "Invalid scene graph configuration: 'resolution_hint' \\(inf\\) is "
      "invalid. The resolution_hint must be positive and finite; given inf");
}

GTEST_TEST(SceneGraphConfigTest, ValidateSlabThickness) {
  SceneGraphConfig config;
  auto& props = config.default_proximity_properties;
  props.slab_thickness = 0;
  DRAKE_EXPECT_THROWS_MESSAGE(
      config.ValidateOrThrow(),
      "Invalid scene graph configuration: 'slab_thickness' \\(0\\) is "
      "invalid. The slab_thickness must be positive and finite; given 0");
  props.slab_thickness = kNan;
  DRAKE_EXPECT_THROWS_MESSAGE(
      config.ValidateOrThrow(),
      "Invalid scene graph configuration: 'slab_thickness' \\(nan\\) is "
      "invalid. The slab_thickness must be positive and finite; given nan");
  props.slab_thickness = kInf;
  DRAKE_EXPECT_THROWS_MESSAGE(
      config.ValidateOrThrow(),
      "Invalid scene graph configuration: 'slab_thickness' \\(inf\\) is "
      "invalid. The slab_thickness must be positive and finite; given inf");
}

GTEST_TEST(SceneGraphConfigTest, ValidateMargin) {
  SceneGraphConfig config;
  auto& props = config.default_proximity_properties;
  props.margin = -1;
  DRAKE_EXPECT_THROWS_MESSAGE(
      config.ValidateOrThrow(),
      "Invalid scene graph configuration: 'margin' \\(-1\\) is "
      "invalid. The margin must be non-negative and finite; given -1");
  props.margin = kNan;
  DRAKE_EXPECT_THROWS_MESSAGE(
      config.ValidateOrThrow(),
      "Invalid scene graph configuration: 'margin' \\(nan\\) is "
      "invalid. The margin must be non-negative and finite; given nan");
  props.margin = kInf;
  DRAKE_EXPECT_THROWS_MESSAGE(
      config.ValidateOrThrow(),
      "Invalid scene graph configuration: 'margin' \\(inf\\) is "
      "invalid. The margin must be non-negative and finite; given inf");
  props.margin = 0;
  EXPECT_NO_THROW(config.ValidateOrThrow());
}

GTEST_TEST(SceneGraphConfigTest, ValidateDynamicFriction) {
  SceneGraphConfig config;
  auto& props = config.default_proximity_properties;
  // Keep static >= dynamic so CoulombFriction isn't the failure mode.
  props.static_friction = 10;
  props.dynamic_friction = -1;
  DRAKE_EXPECT_THROWS_MESSAGE(
      config.ValidateOrThrow(),
      "Invalid scene graph configuration: 'dynamic_friction' \\(-1\\) is "
      "invalid. The friction coefficient can't be negative; given -1");
  props.dynamic_friction = kNan;
  DRAKE_EXPECT_THROWS_MESSAGE(
      config.ValidateOrThrow(),
      "Invalid scene graph configuration: 'dynamic_friction' \\(nan\\) is "
      "invalid. The friction coefficient can't be negative; given nan");
}

GTEST_TEST(SceneGraphConfigTest, ValidateStaticFriction) {
  SceneGraphConfig config;
  auto& props = config.default_proximity_properties;
  props.static_friction = -1;
  DRAKE_EXPECT_THROWS_MESSAGE(
      config.ValidateOrThrow(),
      "Invalid scene graph configuration: 'static_friction' \\(-1\\) is "
      "invalid. The friction coefficient can't be negative; given -1");
  props.static_friction = kNan;
  DRAKE_EXPECT_THROWS_MESSAGE(
      config.ValidateOrThrow(),
      "Invalid scene graph configuration: 'static_friction' \\(nan\\) is "
      "invalid. The friction coefficient can't be negative; given nan");
}

GTEST_TEST(SceneGraphConfigTest, ValidateHuntCrossley) {
  SceneGraphConfig config;
  auto& props = config.default_proximity_properties;
  props.hunt_crossley_dissipation = -1;
  DRAKE_EXPECT_THROWS_MESSAGE(
      config.ValidateOrThrow(),
      "Invalid scene graph configuration: 'hunt_crossley_dissipation' "
      "\\(-1\\) is invalid. The dissipation can't be negative; given -1");
  props.hunt_crossley_dissipation = kNan;
  DRAKE_EXPECT_THROWS_MESSAGE(
      config.ValidateOrThrow(),
      "Invalid scene graph configuration: 'hunt_crossley_dissipation' "
      "\\(nan\\) is invalid. The dissipation can't be negative; given nan");
}

GTEST_TEST(SceneGraphConfigTest, ValidateRelaxationTime) {
  SceneGraphConfig config;
  auto& props = config.default_proximity_properties;
  props.relaxation_time = -1;
  DRAKE_EXPECT_THROWS_MESSAGE(
      config.ValidateOrThrow(),
      "Invalid scene graph configuration: 'relaxation_time' \\(-1\\) is "
      "invalid. The relaxation_time must be non-negative and finite; given "
      "-1");
  props.relaxation_time = kNan;
  DRAKE_EXPECT_THROWS_MESSAGE(
      config.ValidateOrThrow(),
      "Invalid scene graph configuration: 'relaxation_time' \\(nan\\) is "
      "invalid. The relaxation_time must be non-negative and finite; given "
      "nan");
  props.relaxation_time = kInf;
  DRAKE_EXPECT_THROWS_MESSAGE(
      config.ValidateOrThrow(),
      "Invalid scene graph configuration: 'relaxation_time' \\(inf\\) is "
      "invalid. The relaxation_time must be non-negative and finite; given "
      "inf");
}

GTEST_TEST(SceneGraphConfigTest, ValidatePointStiffness) {
  SceneGraphConfig config;
  auto& props = config.default_proximity_properties;
  props.point_stiffness = -1;
  DRAKE_EXPECT_THROWS_MESSAGE(
      config.ValidateOrThrow(),
      "Invalid scene graph configuration: 'point_stiffness' \\(-1\\) is "
      "invalid. The point_contact_stiffness must be strictly positive; given "
      "-1");
  props.point_stiffness = 0;
  DRAKE_EXPECT_THROWS_MESSAGE(
      config.ValidateOrThrow(),
      "Invalid scene graph configuration: 'point_stiffness' \\(0\\) is "
      "invalid. The point_contact_stiffness must be strictly positive; given "
      "0");
  props.point_stiffness = kNan;
  DRAKE_EXPECT_THROWS_MESSAGE(
      config.ValidateOrThrow(),
      "Invalid scene graph configuration: 'point_stiffness' \\(nan\\) is "
      "invalid. The point_contact_stiffness must be strictly positive; given "
      "nan");
  // +∞ is intentionally allowed for point stiffness.
  props.point_stiffness = kInf;
  EXPECT_NO_THROW(config.ValidateOrThrow());
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
