#include "drake/geometry/render_vtk/render_engine_vtk_params.h"

#include <string>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/yaml/yaml_io.h"

namespace drake {
namespace geometry {
namespace render_vtk {
namespace internal {
namespace {

using yaml::LoadYamlString;
using yaml::SaveYamlString;

const char* const kExampleConfig = R"""(
default_diffuse: [1.0, 0.5, 0.25, 1.0]
default_clear_color: [0.25, 0.5, 1.0]
lights:
  - type: spot
    color:
      rgba: [0.5, 0.5, 0.5, 1.0]
    attenuation_values: [1.0, 0.0, 0.0]
    position: [1.0, 2.0, 3.0]
    frame: world
    intensity: 0.9
    direction: [0.0, 0.0, 1.0]
    cone_angle: 45.0
environment_map:
  skybox: false
  texture: !EquirectangularMap
    path: /path/to/environment_image.hdr
exposure: 1.0
ssao:
  radius: 0.2
  bias: 0.001
  sample_count: 64
  intensity_scale: 0.9
  intensity_shift: 0.01
  blur: false
cast_shadows: true
shadow_map_size: 512
force_to_pbr: true
gltf_extensions:
  KHR_draco_mesh_compression:
    warn_unimplemented: true
backend: EGL
)""";

// Test deserialization/re-serialization of the basic configuration.
GTEST_TEST(RenderEngineVtkParams, LoadSaveRoundTrip) {
  using Params = RenderEngineVtkParams;
  const auto config = LoadYamlString<Params>(kExampleConfig);
  EXPECT_EQ("\n" + SaveYamlString(config), kExampleConfig);

  // Sanity check a few values just to make sure something actually loaded.
  EXPECT_EQ(config.default_clear_color[0], 0.25);
  EXPECT_FALSE(config.environment_map.value().skybox);
  EXPECT_EQ(config.lights.at(0).type, "spot");
  EXPECT_EQ(config.gltf_extensions.size(), 1);
}

// Test serialization/deserialization of the basic configuration.
GTEST_TEST(RenderEngineVtkParams, SaveLoadRoundTrip) {
  using Params = RenderEngineVtkParams;
  const Params original{.default_diffuse = Eigen::Vector4d{1.0, 0.5, 0.25, 1.0},
                        .default_clear_color = Eigen::Vector3d{0.25, 0.5, 1.0},
                        .lights = {{.type = "point"}}};
  const std::string yaml = SaveYamlString<Params>(original);
  const Params dut = LoadYamlString<Params>(yaml);
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
  const std::string yaml = SaveYamlString<Params>(original);
  const Params dut = LoadYamlString<Params>(yaml);
  ASSERT_TRUE(dut.environment_map.has_value());
  EXPECT_FALSE(dut.environment_map->skybox);
  ASSERT_TRUE(
      std::holds_alternative<EquirectangularMap>(dut.environment_map->texture));
  EXPECT_EQ(std::get<EquirectangularMap>(dut.environment_map->texture).path,
            "local.hdr");
}

#if defined(__APPLE__)
constexpr bool kApple = true;
#else
constexpr bool kApple = false;
#endif

GTEST_TEST(RenderEngineVtkParams, Backend) {
  RenderEngineVtkParams dut;

  dut.backend = "";
  EXPECT_EQ(
      ParseRenderEngineVtkBackend(dut),
      kApple ? RenderEngineVtkBackend::kCocoa : RenderEngineVtkBackend::kEgl);

  dut.backend = "Cocoa";
  EXPECT_EQ(
      ParseRenderEngineVtkBackend(dut),
      kApple ? RenderEngineVtkBackend::kCocoa : RenderEngineVtkBackend::kEgl);

  dut.backend = "EGL";
  EXPECT_EQ(
      ParseRenderEngineVtkBackend(dut),
      kApple ? RenderEngineVtkBackend::kCocoa : RenderEngineVtkBackend::kEgl);

  dut.backend = "GLX";
  EXPECT_EQ(
      ParseRenderEngineVtkBackend(dut),
      kApple ? RenderEngineVtkBackend::kCocoa : RenderEngineVtkBackend::kGlx);

  dut.backend = "foobar";
  DRAKE_EXPECT_THROWS_MESSAGE(ParseRenderEngineVtkBackend(dut),
                              ".*backend.*foobar.*");
}

}  // namespace
}  // namespace internal
}  // namespace render_vtk
}  // namespace geometry
}  // namespace drake
