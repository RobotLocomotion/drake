#include "drake/systems/sensors/camera_config.h"

#include <limits>
#include <string>
#include <vector>

#include <fmt/format.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/schema/transform.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/render_gl/render_engine_gl_params.h"
#include "drake/geometry/render_gltf_client/render_engine_gltf_client_params.h"
#include "drake/geometry/render_vtk/render_engine_vtk_params.h"
#include "drake/geometry/rgba.h"
#include "drake/systems/sensors/camera_info.h"

namespace drake {
namespace geometry {

// Some quick and dirty comparison operators so we can use gtest matchers with
// the render engine parameter types.
bool operator==(const RenderEngineVtkParams& p1,
                const RenderEngineVtkParams& p2) {
  return yaml::SaveYamlString(p1) == yaml::SaveYamlString(p2);
}

bool operator==(const RenderEngineGlParams& p1,
                const RenderEngineGlParams& p2) {
  return yaml::SaveYamlString(p1) == yaml::SaveYamlString(p2);
}

bool operator==(const RenderEngineGltfClientParams& p1,
                const RenderEngineGltfClientParams& p2) {
  return yaml::SaveYamlString(p1) == yaml::SaveYamlString(p2);
}

}  // namespace geometry
namespace systems {
namespace sensors {
namespace {

using Eigen::Vector3d;
using geometry::RenderEngineGlParams;
using geometry::RenderEngineGltfClientParams;
using geometry::RenderEngineVtkParams;
using geometry::Rgba;
using geometry::render::ColorRenderCamera;
using geometry::render::DepthRenderCamera;
using geometry::render::RenderCameraCore;
using math::RigidTransformd;
using schema::Transform;
using yaml::LoadYamlString;
using yaml::SaveYamlString;

// Confirms that a default-constructed camera config has valid values.
GTEST_TEST(CameraConfigTest, DefaultValues) {
  CameraConfig config;
  ASSERT_NO_THROW(config.ValidateOrThrow());

  // The default background color is documented as matching the color in
  // RenderEngineVtkParams. Confirm they match. If either changes, this test
  // will fail and we'll have to decide what to do about it.
  Rgba vtk_background;
  vtk_background.set(RenderEngineVtkParams().default_clear_color);
  EXPECT_EQ(config.background, vtk_background);
}

// Tests the principal point logic.
GTEST_TEST(CameraConfigTest, PrincipalPoint) {
  const CameraConfig kDefault;

  {
    // If we never set the principal point, it always remains centered on
    // the image (even as we change image size). The computed center should
    // always match the center computed by CameraInfo.

    // Two arbitrary image sizes to confirm that it respects image size. We'll
    // use an arbitrary focal length value that doesn't affect the outcome.
    constexpr double kFovY = 1.5;
    {
      int w = 328, h = 488;
      CameraConfig c{.width = w, .height = h};
      const Vector2<double> center = c.principal_point();
      CameraInfo intrinsics(c.width, c.height, kFovY);
      EXPECT_EQ(center.x(), intrinsics.center_x());
      EXPECT_EQ(center.y(), intrinsics.center_y());
    }

    {
      int w = 207, h = 833;
      CameraConfig c{.width = w, .height = h};
      const Vector2<double> center = c.principal_point();
      CameraInfo intrinsics(c.width, c.height, kFovY);
      EXPECT_EQ(center.x(), intrinsics.center_x());
      EXPECT_EQ(center.y(), intrinsics.center_y());
    }
  }

  // Confirm that the x- and y-components of the principal point are independent
  // and, after setting them, they no longer change w.r.t. image size.
  {
    // Center x.
    CameraConfig c;
    c.center_x = 17;
    EXPECT_EQ(*c.center_x, 17);
    const Vector2<double> center1 = c.principal_point();
    EXPECT_EQ(center1.x(), 17);
    // The y-position of the principal point hasn't moved.
    EXPECT_NEAR(center1.y(), c.height / 2, 0.5);
    // Doesn't change when the width changes.
    c.width = 180;
    const Vector2<double> center2 = c.principal_point();
    EXPECT_EQ(center2.x(), 17);
    EXPECT_EQ(center2.y(), center1.y());
  }

  {
    // Center y.
    CameraConfig c;
    c.center_y = 19;
    EXPECT_EQ(*c.center_y, 19);
    const Vector2<double> center1 = c.principal_point();
    // The x-position of the principal point hasn't moved.
    EXPECT_NEAR(center1.x(), c.width / 2, 0.5);
    EXPECT_EQ(center1.y(), 19);
    // Doesn't change when the height changes.
    c.height = 123;
    const Vector2<double> center2 = c.principal_point();
    EXPECT_EQ(center2.x(), center1.x());
    EXPECT_EQ(center2.y(), 19);
  }
}

// Simply confirm that the FocalLength has the appropriate semantics of defining
// one or both values (and does proper validation).
GTEST_TEST(CameraConfigTest, CameraConfigFocalLength) {
  constexpr double kFocalX = 250;
  constexpr double kFocalY = 250;

  {
    // Just specifying focal x reports that value for x and y.
    const CameraConfig::FocalLength focal{.x = kFocalX};
    EXPECT_EQ(focal.focal_x(), kFocalX);
    EXPECT_EQ(focal.focal_y(), kFocalX);
  }

  {
    // Just specifying focal y reports that value for x and y.
    const CameraConfig::FocalLength focal{.y = kFocalX};
    EXPECT_EQ(focal.focal_x(), kFocalX);
    EXPECT_EQ(focal.focal_y(), kFocalX);
  }

  {
    // Specifying both gets propagated.
    const CameraConfig::FocalLength focal{.x = kFocalX, .y = kFocalY};
    EXPECT_EQ(focal.focal_x(), kFocalX);
    EXPECT_EQ(focal.focal_y(), kFocalY);
  }

  {
    // Specifying no values throws (and both focal_x an focal_y validate).
    const CameraConfig::FocalLength null_focal;
    DRAKE_EXPECT_THROWS_MESSAGE(null_focal.focal_x(),
                                ".*you must define .* for FocalLength.");
    DRAKE_EXPECT_THROWS_MESSAGE(null_focal.focal_y(),
                                ".*you must define .* for FocalLength.");
  }
}

GTEST_TEST(CameraConfigTest, CameraConfigFovDegrees) {
  constexpr int kWidth = 640;
  constexpr int kHeight = 480;
  constexpr double kFocalX = 250;
  constexpr double kFocalY = 250;
  const CameraInfo intrinsics(kWidth, kHeight, kFocalX, kFocalY, kWidth * 0.5,
                              kHeight * 0.5);
  const double kFovXDeg = intrinsics.fov_x() * 180 / M_PI;
  const double kFovYDeg = intrinsics.fov_y() * 180 / M_PI;

  {
    // Just specifying fov x (a) computes the right focal length and (b) the
    // same value for x and y.
    const CameraConfig::FovDegrees fov{.x = kFovXDeg};
    EXPECT_DOUBLE_EQ(fov.focal_x(kWidth, kHeight), kFocalX);
    EXPECT_DOUBLE_EQ(fov.focal_y(kWidth, kHeight), kFocalX);
  }

  {
    // Just specifying fov y (a) computes the right focal length and (b) the
    // same value for x and y.
    const CameraConfig::FovDegrees fov{.y = kFovYDeg};
    EXPECT_DOUBLE_EQ(fov.focal_x(kWidth, kHeight), kFocalY);
    EXPECT_DOUBLE_EQ(fov.focal_y(kWidth, kHeight), kFocalY);
  }

  {
    // Specifying both right focal length for each direction independently.
    const CameraConfig::FovDegrees fov{.x = kFovXDeg, .y = kFovYDeg};
    EXPECT_DOUBLE_EQ(fov.focal_x(kWidth, kHeight), kFocalX);
    EXPECT_DOUBLE_EQ(fov.focal_y(kWidth, kHeight), kFocalY);
  }

  {
    // Specifying no values throws (and both focal_x an focal_y validate).
    CameraConfig::FovDegrees null_fov;
    DRAKE_EXPECT_THROWS_MESSAGE(null_fov.focal_x(kWidth, kHeight),
                                ".*you must define .* for FovDegrees.");
    DRAKE_EXPECT_THROWS_MESSAGE(null_fov.focal_y(kWidth, kHeight),
                                ".*you must define .* for FovDegrees.");
  }
}

// Tests the focal length semantics.
GTEST_TEST(CameraConfigTest, FocalLength) {
  {
    // Both values match by default.
    CameraConfig c;
    EXPECT_EQ(c.focal_x(), c.focal_y());
  }

  {
    // Both change independently when assigning an FocalLength value.
    CameraConfig c;
    const double old_focal = c.focal_x();
    const double new_focal_x = old_focal + 10;
    const double new_focal_y = old_focal - 10;
    c.focal = CameraConfig::FocalLength{new_focal_x, new_focal_y};
    EXPECT_EQ(c.focal_x(), new_focal_x);
    EXPECT_EQ(c.focal_y(), new_focal_y);
  }

  {
    // Both change independently when assigning a FovDegrees value. We'll
    // create field of view angles from target focal lengths, and make sure we
    // get back the expected focal lengths.
    CameraConfig c;
    const double target_focal_x = 250;
    const double target_focal_y = 275;
    // We'll use CameraInfo to convert focal lengths into fovs; the center point
    // doesn't contribute to this calculation.)
    const CameraInfo intrinsics(c.width, c.height, target_focal_x,
                                target_focal_y, 0.5 * c.width, 0.5 * c.height);
    const double fov_x_rad = intrinsics.fov_x();
    const double fov_y_rad = intrinsics.fov_y();

    c.focal = CameraConfig::FovDegrees{fov_x_rad * 180 / M_PI,
                                       fov_y_rad * 180 / M_PI};
    EXPECT_DOUBLE_EQ(c.focal_x(), target_focal_x);
    EXPECT_DOUBLE_EQ(c.focal_y(), target_focal_y);
  }
}

// Confirm that serialization happens the right way:
//    background:
//      rgba: [r, g, b, a]
// We don't have to worry about malformed specifications, that is handled by
// the Rgba's logic and tests. We just need to make sure we're invoking it
// correctly.
GTEST_TEST(CameraConfigTest, SerializeBackgroundRgba) {
  CameraConfig config;
  config.background = Rgba(0.1, 0.2, 0.3, 0.4);
  // Serialize so that only background gets written by providing default.
  const std::string yaml =
      SaveYamlString<CameraConfig>(config, {}, CameraConfig());
  EXPECT_EQ(yaml, "background:\n  rgba: [0.1, 0.2, 0.3, 0.4]\n");
}

GTEST_TEST(CameraConfigTest, SerializationDefaultRoundTrip) {
  const CameraConfig original;
  const std::string yaml = SaveYamlString<CameraConfig>(original);
  EXPECT_NO_THROW(LoadYamlString<CameraConfig>(yaml)) << "with yaml:\n" << yaml;
}

// Various tests to support the documented examples in the documentation. The
// number of each parsed config should correspond to the number in the docs for
// the renderer_class field.
GTEST_TEST(CameraConfigTest, DeserializingRendererClass) {
  auto parse = [](const char* yaml) -> CameraConfig {
    return LoadYamlString<CameraConfig>(yaml, {}, CameraConfig());
  };

  const CameraConfig config_1 = parse("renderer_class: RenderEngineVtk");
  EXPECT_THAT(config_1.renderer_class,
              testing::VariantWith<std::string>("RenderEngineVtk"));

  const CameraConfig config_2 =
      parse("renderer_class: !RenderEngineVtkParams {}");
  EXPECT_THAT(
      config_2.renderer_class,
      testing::VariantWith<RenderEngineVtkParams>(RenderEngineVtkParams{}));

  const CameraConfig config_3 = parse(R"""(
   renderer_class: !RenderEngineVtkParams
     default_clear_color: [0, 0, 0])""");
  EXPECT_THAT(config_3.renderer_class,
              testing::VariantWith<RenderEngineVtkParams>(
                  RenderEngineVtkParams{.default_clear_color = {0, 0, 0}}));

  const CameraConfig config_4 = parse(R"""(
   renderer_class: !RenderEngineGlParams
     default_clear_color:
       rgba: [0, 0, 0, 1])""");
  EXPECT_THAT(config_4.renderer_class,
              testing::VariantWith<RenderEngineGlParams>(
                  RenderEngineGlParams{.default_clear_color = Rgba(0, 0, 0)}));

  const CameraConfig config_5 = parse(R"""(
   renderer_class: !RenderEngineGltfClientParams
     base_url: http://10.10.10.1
     render_endpoint: server
     verbose: true
     cleanup: false)""");
  EXPECT_THAT(config_5.renderer_class,
              testing::VariantWith<RenderEngineGltfClientParams>(
                  RenderEngineGltfClientParams{.base_url = "http://10.10.10.1",
                                               .render_endpoint = "server",
                                               .verbose = true,
                                               .cleanup = false}));

  const CameraConfig config_6 = parse("renderer_class: \"\"");
  EXPECT_THAT(config_6.renderer_class, testing::VariantWith<std::string>(""));
}

// Helper functions for validating a render camera.

void CoreIsValid(const RenderCameraCore& core, const CameraConfig& config,
                 const RigidTransformd& X_BS) {
  EXPECT_EQ(core.renderer_name(), config.renderer_name);
  EXPECT_EQ(core.intrinsics().width(), config.width);
  EXPECT_EQ(core.intrinsics().height(), config.height);
  EXPECT_EQ(core.intrinsics().focal_x(), config.focal_x());
  EXPECT_EQ(core.intrinsics().focal_y(), config.focal_y());
  EXPECT_EQ(core.intrinsics().center_x(), *config.center_x);
  EXPECT_EQ(core.intrinsics().center_y(), *config.center_y);
  EXPECT_EQ(core.clipping().near(), config.clipping_near);
  EXPECT_EQ(core.clipping().far(), config.clipping_far);
  EXPECT_TRUE(core.sensor_pose_in_camera_body().IsExactlyEqualTo(X_BS));
}

void ColorIsValid(const ColorRenderCamera& camera, const CameraConfig& config) {
  EXPECT_EQ(camera.show_window(), config.show_rgb);
  // X_BS = I by convention for CameraConfig.
  CoreIsValid(camera.core(), config, {});
}

void DepthIsValid(const DepthRenderCamera& camera, const CameraConfig& config) {
  EXPECT_EQ(camera.depth_range().min_depth(), config.z_near);
  EXPECT_EQ(camera.depth_range().max_depth(), config.z_far);
  CoreIsValid(camera.core(), config, config.X_BD.GetDeterministicValue());
}

// Tests the logic for creating the pair of cameras. Generally, it confirms
// that valid config values propagate through.
GTEST_TEST(CameraConfigTest, MakeCameras) {
  // These values are *supposed* to be different from the default values except
  // for X_PB, fps, rgb, depth, and do_compress. None of those contribute
  // to the test result.
  CameraConfig config{.width = 320,
                      .height = 240,
                      .focal = CameraConfig::FocalLength{470.0, 480.0},
                      .center_x = 237,
                      .center_y = 233,
                      .clipping_near = 0.075,
                      .clipping_far = 17.5,
                      .z_near = 0.15,
                      .z_far = 4.75,
                      .X_PB = {},
                      .X_BD = Transform{RigidTransformd{Vector3d::UnitX()}},
                      .renderer_name = "test_renderer",
                      .background = Rgba(0.1, 0.2, 0.3, 0.4),
                      .name = "test_camera",
                      .fps = 17,
                      .rgb = false,
                      .depth = true,
                      .label = true,
                      .show_rgb = true};

  // Check that all the values have propagated.
  const auto [color, depth] = config.MakeCameras();
  SCOPED_TRACE("Fully specified values");
  ColorIsValid(color, config);
  DepthIsValid(depth, config);
}

// CameraConfig explicitly validates several things:
//    - X_BD.base_frame is empty,
//    - X_BC.base_frame is empty,
//    - name is not empty,
//    - renderer_name is not empty,
//    - renderer_class is one of the documented set of supported strings, and
//    - fps is a positive, finite value.
// It relies on Drake's geometry::render artifacts to validate the other
// restricted values. Finally, validation should be part of serialization.
//
// This test confirms CameraConfigs *specific* validation responsibilities,
// and provides smoke tests that indicate that the other fields are validated
// and that Serialize validates.
GTEST_TEST(CameraConfigTest, Validation) {
  const CameraConfig kDefault;
  // Reality check that the default configuration is valid.
  EXPECT_NO_THROW(kDefault.ValidateOrThrow());

  {
    CameraConfig config;
    config.X_BD.base_frame = "error";
    DRAKE_EXPECT_THROWS_MESSAGE(
        config.ValidateOrThrow(),
        ".*X_BD must not specify a base frame. 'error' found.");
  }

  {
    CameraConfig config;
    config.X_BC.base_frame = "error";
    DRAKE_EXPECT_THROWS_MESSAGE(
        config.ValidateOrThrow(),
        ".*X_BC must not specify a base frame. 'error' found.");
  }

  // We require a non-empty name.
  DRAKE_EXPECT_THROWS_MESSAGE(CameraConfig{.name = ""}.ValidateOrThrow(),
                              ".* name cannot be empty.");
  // We require a non-empty renderer-name.
  DRAKE_EXPECT_THROWS_MESSAGE(
      CameraConfig{.renderer_name = ""}.ValidateOrThrow(),
      ".*renderer_name cannot be empty.");

  // Good renderer_class strings don't throw (we already know that default
  // doesn't throw).
  EXPECT_NO_THROW(
      CameraConfig{.renderer_class = "RenderEngineVtk"}.ValidateOrThrow());
  EXPECT_NO_THROW(
      CameraConfig{.renderer_class = "RenderEngineGl"}.ValidateOrThrow());
  // Bad renderer_class strings throw -- proof the field is being validated.
  DRAKE_EXPECT_THROWS_MESSAGE(
      CameraConfig{.renderer_class = "BadName"}.ValidateOrThrow(),
      ".*the given renderer_class value.*");

  // Non-positive, non-finite values all throw.
  DRAKE_EXPECT_THROWS_MESSAGE(CameraConfig{.fps = 0}.ValidateOrThrow(),
                              ".*FPS.*");
  DRAKE_EXPECT_THROWS_MESSAGE(CameraConfig{.fps = -11}.ValidateOrThrow(),
                              ".*FPS.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      CameraConfig{.fps = std::numeric_limits<double>::infinity()}
          .ValidateOrThrow(),
      ".*FPS.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      CameraConfig{.fps = std::numeric_limits<double>::quiet_NaN()}
          .ValidateOrThrow(),
      ".*FPS.*");

  // Indicator that geometry::render is being exercised to validate other
  // values.
  EXPECT_THROW(CameraConfig{.width = -5}.ValidateOrThrow(), std::exception);

  // An invalid configuration should throw when serializing.
  DRAKE_EXPECT_THROWS_MESSAGE(SaveYamlString(CameraConfig{.name = ""}),
                              ".*name cannot be empty.");
  // An invalid sub-struct should throw when serializing.
  DRAKE_EXPECT_THROWS_MESSAGE(
      SaveYamlString(CameraConfig{.focal = CameraConfig::FovDegrees{}}),
      ".*must define at least x or y.*");

  // However, if rgb = depth = label = false, then the configuration is by
  // definition valid, even with otherwise bad values elsewhere.
  CameraConfig config_no_render{.rgb = false, .depth = false, .label = false};
  config_no_render.focal = CameraConfig::FovDegrees{};
  EXPECT_NO_THROW(config_no_render.ValidateOrThrow());
  EXPECT_NO_THROW(SaveYamlString(config_no_render));
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
