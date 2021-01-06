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

using Eigen::Matrix4d;
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

/* Confirm that the correct projection matrix is created. Note: this doesn't
test that the matrix as defined as below is correct; for that, we appeal to
authority (the provided link). We merely show that the matrix is implemented
correctly.  */
GTEST_TEST(RenderCameraCoreTest, CalcProjectionMatrix) {
  /* Given this definition of the projection matrix:
   (See https://strawlab.org/2011/11/05/augmented-reality-with-OpenGL/)

            │ 2*fx/w     0      (w - 2*cx) / w       0    │
            │ 0        2*fy/h  -(h - 2*cy) / h       0    │
            │ 0          0        -(f+n) / d   -2*f*n / d │
            │ 0          0             -1            0    │

   To test this, we'll create a camera and have it compute a reference
   projection matrix. Then, by perturbing each camera parameter, we can predict
   the relationship between the reference matrix and the new matrix. The
   differences should be limited. By accounting for the known differences, we
   should be able to compare the test matrix with the reference matrix and
   show equivalency.

   We've picked arbitrary values for the intrinsics that would be atypical for
   rendering applications to prevent the possibility that the test parameters
   shadow otherwise hard-coded, default values. (e.g., a 640x480 image). */
  constexpr double kEps = std::numeric_limits<double>::epsilon();
  const int w{384};
  const int h{175};
  const double fx{378};
  const double fy{410};
  const double cx{w * 0.5 + 15};
  const double cy{h * 0.5 - 11};
  const double n{0.45};
  const double f{11.75};
  const RenderCameraCore base_cam{"", {w, h, fx, fy, cx, cy}, {n, f}, {}};
  Matrix4d X_DC_base = base_cam.CalcProjectionMatrix();

  {
    /* Confirm zeros in all the right places.  */
    Matrix4d X_DC_alt(X_DC_base);
    X_DC_alt(0, 1) = 0;
    X_DC_alt(0, 3) = 0;
    X_DC_alt(1, 0) = 0;
    X_DC_alt(1, 3) = 0;
    X_DC_alt(2, 0) = 0;
    X_DC_alt(2, 1) = 0;
    X_DC_alt(3, 0) = 0;
    X_DC_alt(3, 1) = 0;
    X_DC_alt(3, 3) = 0;
    EXPECT_TRUE(CompareMatrices(X_DC_base, X_DC_alt));
  }

  {
    /* Case: width. Doubling width should make:
         - X_DC(0, 0) * 2 = X_DC_base(0, 0) and
         - X_DC(0, 2) - cx / w = X_DC_base(0, 2).  */
    const RenderCameraCore cam{"", {2 * w, h, fx, fy, cx, cy}, {n, f}, {}};
    Eigen::Matrix4d X_DC = cam.CalcProjectionMatrix();
    X_DC(0, 0) *= 2;
    X_DC(0, 2) -= cx / w;
    EXPECT_TRUE(CompareMatrices(X_DC, X_DC_base));
  }

  {
    /* Case: height. Doubling height should make:
         - X_DC(1, 1) * 2 = X_DC_base(1, 1) and
         - X_DC(1, 2) + cy / h = X_DC_base(1, 2).  */
    const RenderCameraCore cam{"", {w, 2 * h, fx, fy, cx, cy}, {n, f}, {}};
    Eigen::Matrix4d X_DC = cam.CalcProjectionMatrix();
    X_DC(1, 1) *= 2;
    X_DC(1, 2) += cy / h;
    EXPECT_TRUE(CompareMatrices(X_DC, X_DC_base, kEps));
  }

  {
    /* Case: fx. Doubling fx should make X_DC(0, 0) / 2 = X_DC_base(0, 0).  */
    const RenderCameraCore cam{"", {w, h, 2 * fx, fy, cx, cy}, {n, f}, {}};
    Eigen::Matrix4d X_DC = cam.CalcProjectionMatrix();
    X_DC(0, 0) /= 2;
    EXPECT_TRUE(CompareMatrices(X_DC, X_DC_base, kEps));
  }

  {
    /* Case: fy. Doubling fy should make X_DC(1, 1) / 2 = X_DC_base(1, 1).  */
    const RenderCameraCore cam{"", {w, h, fx, 2 * fy, cx, cy}, {n, f}, {}};
    Eigen::Matrix4d X_DC = cam.CalcProjectionMatrix();
    X_DC(1, 1) /= 2;
    EXPECT_TRUE(CompareMatrices(X_DC, X_DC_base, kEps));
  }

  {
    /* Case: cx. Changing cx by +13 should make
        - X_DC(0, 2) + 2 * 13 / w = X_DC_base(0, 2).  */
    const RenderCameraCore cam{"", {w, h, fx, fy, cx + 13, cy}, {n, f}, {}};
    Eigen::Matrix4d X_DC = cam.CalcProjectionMatrix();
    X_DC(0, 2) += 2 * 13.0 / w;
    EXPECT_TRUE(CompareMatrices(X_DC, X_DC_base, kEps));
  }

  {
    /* Case: cy. Doubling cy +13 should make
        - X_DC(1, 2) - 2 * 13 / h = X_DC_base(1, 2).  */
    const RenderCameraCore cam{"", {w, h, fx, fy, cx, cy + 13}, {n, f}, {}};
    Eigen::Matrix4d X_DC = cam.CalcProjectionMatrix();
    X_DC(1, 2) -= 2 * 13.0 / h;
    EXPECT_TRUE(CompareMatrices(X_DC, X_DC_base, kEps));
  }

  {
    /* Case: n. The relationship between f and n is ugly enough that we
     can't really define the relationship between X_DC and X_DC_base. Instead,
     we'll calculate the expected value directly.  */
    const RenderCameraCore cam{"", {w, h, fx, fy, cx, cy}, {2 * n, f}, {}};
    Eigen::Matrix4d X_DC = cam.CalcProjectionMatrix();
    Matrix4d X_DC_expected = X_DC_base;
    X_DC_expected(2, 2) = -(f + 2 * n) / (f - 2 * n);
    X_DC_expected(2, 3) = -2 * f * 2 * n / (f - 2 * n);
    EXPECT_TRUE(CompareMatrices(X_DC, X_DC_expected, kEps));
  }

  {
    /* Case: f. The relationship between f and n is ugly enough that we
     can't really define the relationship between X_DC and X_DC_base. Instead,
     we'll calculate the expected value directly.  */
    const RenderCameraCore cam{"", {w, h, fx, fy, cx, cy}, {n, 2 * f}, {}};
    Eigen::Matrix4d X_DC = cam.CalcProjectionMatrix();
    Matrix4d X_DC_expected = X_DC_base;
    X_DC_expected(2, 2) = -(2 * f + n) / (2 * f - n);
    X_DC_expected(2, 3) = -2 * 2 * f * n / (2 * f - n);
    EXPECT_TRUE(CompareMatrices(X_DC, X_DC_expected, kEps));
  }
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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

/* Test the conversion constructor. This will be removed when CameraProperties
 is removed.  */
GTEST_TEST(RenderCameraCoreTest, CameraPropertiesConversion) {
  const CameraProperties props{640, 480, M_PI / 3, "some_name"};
  const RigidTransformd X_BS(Vector3d{1, 2, 3});
  for (const double clipping_far : {15.0, 22.5}) {
    const CameraInfo expected_intrinsics(props.width, props.height,
                                         props.fov_y);
    const RenderCameraCore cam{props, clipping_far, X_BS};
    EXPECT_EQ(cam.intrinsics().width(), expected_intrinsics.width());
    EXPECT_EQ(cam.intrinsics().height(), expected_intrinsics.height());
    EXPECT_EQ(cam.intrinsics().fov_y(), expected_intrinsics.fov_y());
    EXPECT_EQ(cam.intrinsics().fov_x(), expected_intrinsics.fov_x());
    EXPECT_EQ(cam.intrinsics().center_x(), expected_intrinsics.center_x());
    EXPECT_EQ(cam.intrinsics().center_y(), expected_intrinsics.center_y());
    EXPECT_EQ(cam.renderer_name(), props.renderer_name);
    EXPECT_EQ(cam.clipping().far(), clipping_far);
    // We're not putting a strict value on the near clipping plane; just that
    // it's strictly positive and less than the far value.
    EXPECT_GT(cam.clipping().near(), 0);
    EXPECT_LT(cam.clipping().near(), cam.clipping().far());
    EXPECT_TRUE(
        CompareMatrices(cam.sensor_pose_in_camera_body().GetAsMatrix34(),
                        X_BS.GetAsMatrix34()));
  }
}

/* Test the conversion constructor. This will be removed when CameraProperties
 is removed.  */
GTEST_TEST(ColorRenderCameraTest, CameraPropertiesConversion) {
  const CameraProperties props{640, 480, M_PI / 3, "some_name"};
  const RigidTransformd X_BS(Vector3d{1, 2, 3});
  for (const bool show_window : {true, false}) {
    const ColorRenderCamera cam{props, show_window, X_BS};
    // We'll test a single feature of the core camera data, assuming that its
    // success implies that the core data was correctly converted.
    EXPECT_EQ(cam.core().intrinsics().width(), props.width);
    // We *will* confirm that the conversion sets up valid clipping planes:
    // strictly positive with near < far.
    EXPECT_GT(cam.core().clipping().near(), 0);
    EXPECT_LT(cam.core().clipping().near(), cam.core().clipping().far());
    EXPECT_EQ(cam.show_window(), show_window);
    // Confirm that the pose gets passed along.
    EXPECT_TRUE(
        CompareMatrices(cam.core().sensor_pose_in_camera_body().GetAsMatrix34(),
                        X_BS.GetAsMatrix34()));
  }
}

/* Test the conversion constructor. This will be removed when
 DepthCameraProperties is removed.  */
GTEST_TEST(DepthRenderCameraTest, CameraPropertiesConversion) {
  const DepthCameraProperties props{640, 480, M_PI / 3, "some_name", 0.25, 7.5};
  const RigidTransformd X_BS(Vector3d{1, 2, 3});
    const DepthRenderCamera cam{props, X_BS};
    // We'll test a single feature of the core camera data, assuming that its
    // success implies that the core data was correctly converted.
    EXPECT_EQ(cam.core().intrinsics().width(), props.width);
    // We *will* confirm that the conversion sets up valid clipping planes:
    // strictly positive, near < far, and the depth range is "enclosed" in the
    // clipping range.
    EXPECT_GT(cam.core().clipping().near(), 0);
    EXPECT_LE(cam.core().clipping().near(), cam.depth_range().min_depth());
    EXPECT_LT(cam.core().clipping().near(), cam.core().clipping().far());
    EXPECT_GE(cam.core().clipping().far(), cam.depth_range().max_depth());
    EXPECT_EQ(cam.depth_range().min_depth(), props.z_near);
    EXPECT_EQ(cam.depth_range().max_depth(), props.z_far);
    // Confirm that the pose gets passed along.
    EXPECT_TRUE(
        CompareMatrices(cam.core().sensor_pose_in_camera_body().GetAsMatrix34(),
                        X_BS.GetAsMatrix34()));
}
#pragma GCC diagnostic pop

}  // namespace
}  // namespace render
}  // namespace geometry
}  // namespace drake
