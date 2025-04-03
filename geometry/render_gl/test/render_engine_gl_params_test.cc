#include "drake/geometry/render_gl/render_engine_gl_params.h"

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
  };
  const std::string yaml = yaml::SaveYamlString<Params>(original);
  const Params dut = yaml::LoadYamlString<Params>(yaml);
  EXPECT_EQ(dut.default_diffuse, original.default_diffuse);
  EXPECT_EQ(dut.default_clear_color, original.default_clear_color);
  ASSERT_EQ(dut.lights.size(), 1);
  EXPECT_EQ(dut.lights.at(0).type, "point");
}

// This is intended to confirm that dut can be operator=='d.
GTEST_TEST(RenderEngineGlParams, EqualsSmokeTest) {
  RenderEngineGlParams params1{.lights = {{.type = "spot"}}};
  RenderEngineGlParams params1_copy = params1;
  RenderEngineGlParams params2{.lights = {{.type = "point"}}};

  EXPECT_TRUE(params1 == params1_copy);
  EXPECT_FALSE(params1 == params2);
}

}  // namespace
}  // namespace geometry
}  // namespace drake
