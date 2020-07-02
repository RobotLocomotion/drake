#include "drake/geometry/render/render_camera.h"

#include <stdexcept>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace geometry {
namespace render {
namespace {

using Eigen::Vector3d;
using math::RigidTransformd;
using systems::sensors::CameraInfo;

GTEST_TEST(ClippingRangeTest, Constructor) {
  {
    // Case: Valid values.
    const ClippingRange range{0.25, 10.75};
    EXPECT_EQ(range.near(), 0.25);
    EXPECT_EQ(range.far(), 10.75);
  }

  {
    // Case: Bad values.
    const char* error_message =
        "The clipping range values must both be positive and far must be "
        "greater than near. Instantiated with near = {} and far = {}";
    DRAKE_EXPECT_THROWS_MESSAGE(ClippingRange(-0.1, 10), std::runtime_error,
                                fmt::format(error_message, -0.1, 10.));
    DRAKE_EXPECT_THROWS_MESSAGE(ClippingRange(0.1, -10), std::runtime_error,
                                fmt::format(error_message, 0.1, -10.0));
    DRAKE_EXPECT_THROWS_MESSAGE(ClippingRange(1.5, 1.0), std::runtime_error,
                                fmt::format(error_message, 1.5, 1.0));
  }
}

GTEST_TEST(RenderCameraCoreTest, Constructor) {
  const std::string renderer_name = "some_renderer";
  const CameraInfo intrinsics{320, 240, M_PI};
  const ClippingRange clipping{0.25, 5.};
  const RigidTransformd X_BS{Vector3d{1.5, 2.5, 3.5}};
  const RenderCameraCore core{renderer_name, intrinsics, clipping, X_BS};

  EXPECT_EQ(core.renderer_name(), renderer_name);
  EXPECT_TRUE(CompareMatrices(core.intrinsics().intrinsic_matrix(),
                              intrinsics.intrinsic_matrix()));
  EXPECT_EQ(core.clipping().near(), clipping.near());
  EXPECT_EQ(core.clipping().far(), clipping.far());
  EXPECT_TRUE(CompareMatrices(core.sensor_pose_in_camera_body().GetAsMatrix4(),
                              X_BS.GetAsMatrix4()));
}

GTEST_TEST(ColorRenderCameraTest, Constructor) {
  const std::string renderer_name = "some_renderer";
  const CameraInfo intrinsics{320, 240, M_PI};
  const ClippingRange clipping{0.25, 5.};
  const RigidTransformd X_BS{Vector3d{1.5, 2.5, 3.5}};
  const RenderCameraCore core{renderer_name, intrinsics, clipping, X_BS};
  const bool show_window = true;

  const ColorRenderCamera camera(core, show_window);

  EXPECT_EQ(camera.core().renderer_name(), renderer_name);
  EXPECT_TRUE(CompareMatrices(camera.core().intrinsics().intrinsic_matrix(),
                              intrinsics.intrinsic_matrix()));
  EXPECT_EQ(camera.core().clipping().near(), clipping.near());
  EXPECT_EQ(camera.core().clipping().far(), clipping.far());
  EXPECT_TRUE(
      CompareMatrices(camera.core().sensor_pose_in_camera_body().GetAsMatrix4(),
                      X_BS.GetAsMatrix4()));
  EXPECT_EQ(camera.show_window(), show_window);
}

GTEST_TEST(DepthRangeTest, Constructor) {
  {
    // Case: Valid values.
    const DepthRange range{0.25, 10.75};
    EXPECT_EQ(range.min_depth(), 0.25);
    EXPECT_EQ(range.max_depth(), 10.75);
  }

  {
    // Case: Bad values.
    const char* error_message =
        "The depth range values must both be positive and the maximum depth "
        "must be greater than the minimum depth. Instantiated with min = {} "
        "and max = {}";
    DRAKE_EXPECT_THROWS_MESSAGE(DepthRange(-0.1, 10), std::runtime_error,
                                fmt::format(error_message, -0.1, 10.));
    DRAKE_EXPECT_THROWS_MESSAGE(DepthRange(0.1, -10), std::runtime_error,
                                fmt::format(error_message, 0.1, -10.0));
    DRAKE_EXPECT_THROWS_MESSAGE(DepthRange(1.5, 1.0), std::runtime_error,
                                fmt::format(error_message, 1.5, 1.0));
  }
}

GTEST_TEST(DepthRenderCameraTest, Constructor) {
  const std::string renderer_name = "some_renderer";
  const CameraInfo intrinsics{320, 240, M_PI};
  const ClippingRange clipping{0.25, 5.};
  const RigidTransformd X_BS{Vector3d{1.5, 2.5, 3.5}};
  const RenderCameraCore core{renderer_name, intrinsics, clipping, X_BS};
  const DepthRange range{0.5, 4.5};

  const DepthRenderCamera camera(core, range);

  EXPECT_EQ(camera.core().renderer_name(), renderer_name);
  EXPECT_TRUE(CompareMatrices(camera.core().intrinsics().intrinsic_matrix(),
                              intrinsics.intrinsic_matrix()));
  EXPECT_EQ(camera.core().clipping().near(), clipping.near());
  EXPECT_EQ(camera.core().clipping().far(), clipping.far());
  EXPECT_TRUE(
      CompareMatrices(camera.core().sensor_pose_in_camera_body().GetAsMatrix4(),
                      X_BS.GetAsMatrix4()));
  EXPECT_EQ(camera.depth_range().min_depth(), range.min_depth());
  EXPECT_EQ(camera.depth_range().max_depth(), range.max_depth());

  DRAKE_EXPECT_THROWS_MESSAGE(
      DepthRenderCamera(core,
                        DepthRange(clipping.near() - 0.1, clipping.far())),
      std::runtime_error,
      "Depth camera's depth range extends beyond the clipping planes; near = "
      ".+, far = .+, min. depth = .+, max. depth = .+");
  DRAKE_EXPECT_THROWS_MESSAGE(
      DepthRenderCamera(core,
                        DepthRange(clipping.near(), clipping.far() + 0.1)),
      std::runtime_error,
      "Depth camera's depth range extends beyond the clipping planes; near = "
      ".+, far = .+, min. depth = .+, max. depth = .+");
  DRAKE_EXPECT_THROWS_MESSAGE(
      DepthRenderCamera(
          core, DepthRange(clipping.near() - 0.1, clipping.far() + 0.1)),
      std::runtime_error,
      "Depth camera's depth range extends beyond the clipping planes; near = "
      ".+, far = .+, min. depth = .+, max. depth = .+");
}

}  // namespace
}  // namespace render
}  // namespace geometry
}  // namespace drake
