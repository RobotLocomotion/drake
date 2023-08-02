#include "drake/geometry/render_vtk/render_engine_vtk_params.h"

#include <gtest/gtest.h>

#include "drake/common/yaml/yaml_io.h"

namespace drake {
namespace geometry {
namespace {

GTEST_TEST(RenderEngineVtkParams, Serialization) {
  using Params = RenderEngineVtkParams;
  const Params original{
      .default_diffuse = Eigen::Vector4d{1.0, 0.5, 0.25, 1.0},
      .default_clear_color = Eigen::Vector3d{0.25, 0.5, 1.0},
  };
  const std::string yaml = yaml::SaveYamlString<Params>(original);
  const Params dut = yaml::LoadYamlString<Params>(yaml);
  EXPECT_EQ(dut.default_diffuse, original.default_diffuse);
  EXPECT_EQ(dut.default_clear_color, original.default_clear_color);
}

}  // namespace
}  // namespace geometry
}  // namespace drake
