#include "drake/geometry/render_gl/render_engine_gl_params.h"

#include <string>

#include <gtest/gtest.h>

#include "drake/common/yaml/yaml_io.h"

namespace drake {
namespace geometry {
namespace {

GTEST_TEST(RenderEngineGlParams, Serialization) {
  using Params = RenderEngineGlParams;
  const Params original{
      .default_diffuse = Rgba{1.0, 0.5, 0.25},
      .default_clear_color = Rgba{0.25, 0.5, 1.0},
      .lights = {{.type = "point"}},
      .cast_shadows = true,
      .shadow_map_size = 512,
  };
  const std::string yaml = yaml::SaveYamlString<Params>(original);
  const Params dut = yaml::LoadYamlString<Params>(yaml);
  EXPECT_EQ(dut.default_diffuse, original.default_diffuse);
  EXPECT_EQ(dut.default_clear_color, original.default_clear_color);
  ASSERT_EQ(dut.lights.size(), 1);
  EXPECT_EQ(dut.lights.at(0).type, "point");
  EXPECT_TRUE(dut.cast_shadows);
  EXPECT_EQ(dut.shadow_map_size, 512);
}

}  // namespace
}  // namespace geometry
}  // namespace drake
