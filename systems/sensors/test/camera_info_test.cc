#include "drake/systems/sensors/camera_info.h"

#include <limits>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace systems {
namespace sensors {
namespace {
// This is because there is a precision difference between Ubuntu and Mac.
const double kTolerance = 1e-12;

const int kWidth = 640;
const int kHeight = 480;
const double kFx = 554.25625842204079;  // In pixels.
const double kFy = 579.41125496954282;  // In pixels.
const double kCx = kWidth * 0.5 - 0.5;
const double kCy = kHeight * 0.5 - 0.5;
const double kVerticalFov = 0.78539816339744828;  // 45.0 degrees.

void Verify(const Eigen::Matrix3d& expected, const CameraInfo& dut) {
  EXPECT_EQ(kWidth, dut.width());
  EXPECT_EQ(kHeight, dut.height());
  EXPECT_NEAR(expected(0, 0), dut.focal_x(), kTolerance);
  EXPECT_NEAR(expected(1, 1), dut.focal_y(), kTolerance);
  EXPECT_NEAR(expected(0, 2), dut.center_x(), kTolerance);
  EXPECT_NEAR(expected(1, 2), dut.center_y(), kTolerance);
  EXPECT_TRUE(CompareMatrices(expected, dut.intrinsic_matrix(), kTolerance));
}

GTEST_TEST(TestCameraInfo, ConstructionTest) {
  const Eigen::Matrix3d expected(
      (Eigen::Matrix3d() << kFx, 0., kCx, 0., kFy, kCy, 0., 0., 1.).finished());

  {
    SCOPED_TRACE("Spelled out");
    CameraInfo dut(kWidth, kHeight, kFx, kFy, kCx, kCy);
    Verify(expected, dut);
  }
  {
    SCOPED_TRACE("Matrix");
    CameraInfo dut(kWidth, kHeight, expected);
    Verify(expected, dut);
  }
}

// The focal lengths become identical with this constructor.
GTEST_TEST(TestCameraInfo, ConstructionWithFovTest) {
  const Eigen::Matrix3d expected(
      (Eigen::Matrix3d() << kFy, 0., kCx, 0., kFy, kCy, 0., 0., 1.).finished());

  CameraInfo dut(kWidth, kHeight, kVerticalFov);
  Verify(expected, dut);
}

// Confirms that the reported field of view (in radians) is the same as is
// given.
GTEST_TEST(TestCameraInfo, FieldOfView) {
  // Pick some arbitrary angle that isn't a "nice" angle.
  const double fov_y = M_PI / 7;
  const double kEps = std::numeric_limits<double>::epsilon();

  {
    // Square camera: fields of view are equal in x- and y-directions.
    CameraInfo camera(100, 100, fov_y);
    EXPECT_NEAR(camera.fov_y(), fov_y, kEps);
    EXPECT_NEAR(camera.fov_x(), fov_y, kEps);
  }

  {
    // Rectangular camera: has an identical *focal lengths* in the x- and y-
    // directions. But the rectangular image leads to different fields of view.
    const int w = 100;
    const int h = 200;
    CameraInfo camera{w, h, fov_y};
    const double fov_x = 2 * atan(w * tan(fov_y / 2) / h);
    EXPECT_NEAR(camera.fov_y(), fov_y, kEps);
    EXPECT_NEAR(camera.fov_x(), fov_x, kEps);
  }
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
