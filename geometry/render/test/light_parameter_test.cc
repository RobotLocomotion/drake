#include "drake/geometry/render/light_parameter.h"

#include <string>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/yaml/yaml_io.h"

namespace drake {
namespace geometry {
namespace render {
namespace {

using Eigen::Vector3d;

// Test for valid default values.
GTEST_TEST(LightParamterTest, DefaultValues) {
  const LightParameter light;
  EXPECT_EQ(light.type, fmt::to_string(LightType::kDirectional));
  EXPECT_EQ(light.color, Rgba(1, 1, 1));
  EXPECT_TRUE(CompareMatrices(light.attenuation_values, Vector3d(1, 0, 0)));
  EXPECT_TRUE(CompareMatrices(light.position, Vector3d(0, 0, 0)));
  EXPECT_EQ(light.frame, fmt::to_string(LightFrame::kCamera));
  EXPECT_EQ(light.intensity, 1.0);
  EXPECT_TRUE(CompareMatrices(light.direction, Vector3d{0, 0, 1}));
  EXPECT_EQ(light.cone_angle, 0);
}

GTEST_TEST(LightParamterTest, ToString) {
  const LightType light = LightType::kDirectional;
  const LightFrame frame = LightFrame::kCamera;
  EXPECT_EQ(to_string(light), "directional");
  EXPECT_EQ(to_string(frame), "camera");
  EXPECT_EQ(fmt::to_string(light), "directional");
  EXPECT_EQ(fmt::to_string(frame), "camera");
}

GTEST_TEST(LightParameterTest, Serialization) {
  const LightParameter light{.type = "spotlight",
                             .color = Rgba(0.25, 0.75, 0.5),
                             .attenuation_values = {0.25, 0.5, 0.75},
                             .position = {1, 2, 3},
                             .frame = "world",
                             .intensity = 0.5,
                             .direction = {1, 0, 1},
                             .cone_angle = 90.0};

  const std::string yaml = yaml::SaveYamlString<LightParameter>(light);
  const LightParameter dut = yaml::LoadYamlString<LightParameter>(yaml);

  EXPECT_EQ(light.type, dut.type);
  EXPECT_EQ(light.color, dut.color);
  EXPECT_TRUE(
      CompareMatrices(light.attenuation_values, dut.attenuation_values));
  EXPECT_TRUE(CompareMatrices(light.position, dut.position));
  EXPECT_EQ(light.frame, dut.frame);
  EXPECT_EQ(light.intensity, dut.intensity);
  EXPECT_TRUE(CompareMatrices(light.direction, dut.direction));
  EXPECT_EQ(light.cone_angle, dut.cone_angle);
}

}  // namespace
}  // namespace render
}  // namespace geometry
}  // namespace drake
