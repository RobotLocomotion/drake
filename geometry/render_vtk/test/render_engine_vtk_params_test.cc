#include "drake/geometry/render_vtk/render_engine_vtk_params.h"

#include <gtest/gtest.h>

#include "drake/common/yaml/yaml_io.h"

namespace drake {
namespace geometry {
namespace {

// Test serialization/deserialization of the basic configuration.
GTEST_TEST(RenderEngineVtkParams, BasicSerialization) {
  using Params = RenderEngineVtkParams;
  const Params original{.default_diffuse = Eigen::Vector4d{1.0, 0.5, 0.25, 1.0},
                        .default_clear_color = Eigen::Vector3d{0.25, 0.5, 1.0},
                        .lights = {{.type = "point"}}};
  const std::string yaml = yaml::SaveYamlString<Params>(original);
  const Params dut = yaml::LoadYamlString<Params>(yaml);
  EXPECT_EQ(dut.default_diffuse, original.default_diffuse);
  EXPECT_EQ(dut.default_clear_color, original.default_clear_color);
  ASSERT_EQ(dut.lights.size(), 1);
  EXPECT_EQ(dut.lights.at(0).type, "point");
  EXPECT_FALSE(dut.environment_map.has_value());
}

// Test the serialization of params with an environment map.
GTEST_TEST(RenderEngineVtkParams, SerializationWithEquirectangularMap) {
  using Params = RenderEngineVtkParams;
  const Params original{
      .environment_map = EnvironmentMap{
          .skybox = false, .texture = EquirectangularMap{.path = "local.hdr"}}};
  const std::string yaml = yaml::SaveYamlString<Params>(original);
  const Params dut = yaml::LoadYamlString<Params>(yaml);
  ASSERT_TRUE(dut.environment_map.has_value());
  EXPECT_FALSE(dut.environment_map->skybox);
  ASSERT_TRUE(
      std::holds_alternative<EquirectangularMap>(dut.environment_map->texture));
  EXPECT_EQ(std::get<EquirectangularMap>(dut.environment_map->texture).path,
            "local.hdr");
}

}  // namespace
}  // namespace geometry
}  // namespace drake
