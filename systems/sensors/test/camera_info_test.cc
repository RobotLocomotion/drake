#include "drake/systems/sensors/camera_info.h"

#include <limits>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace systems {
namespace sensors {
namespace {

using Eigen::Matrix3d;

// This is because there is a precision difference between Ubuntu and Mac.
const double kTolerance = 1e-12;

const int kWidth = 640;
const int kHeight = 480;
const double kFx = 554.25625842204079;  // In pixels.
const double kFy = 579.41125496954282;  // In pixels.
const double kCx = kWidth * 0.5 - 0.5;
const double kCy = kHeight * 0.5 - 0.5;
const double kVerticalFov = 0.78539816339744828;  // 45.0 degrees.

void Verify(const Matrix3d& expected, const CameraInfo& dut) {
  EXPECT_EQ(kWidth, dut.width());
  EXPECT_EQ(kHeight, dut.height());
  EXPECT_NEAR(expected(0, 0), dut.focal_x(), kTolerance);
  EXPECT_NEAR(expected(1, 1), dut.focal_y(), kTolerance);
  EXPECT_NEAR(expected(0, 2), dut.center_x(), kTolerance);
  EXPECT_NEAR(expected(1, 2), dut.center_y(), kTolerance);
  EXPECT_TRUE(CompareMatrices(expected, dut.intrinsic_matrix(), kTolerance));
}

GTEST_TEST(TestCameraInfo, ConstructionTest) {
  const Matrix3d expected(
      (Matrix3d() << kFx, 0., kCx, 0., kFy, kCy, 0., 0., 1.).finished());

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
  const Matrix3d expected(
      (Matrix3d() << kFy, 0., kCx, 0., kFy, kCy, 0., 0., 1.).finished());

  CameraInfo dut(kWidth, kHeight, kVerticalFov);
  Verify(expected, dut);
}

// Confirms that values that lie outside of the valid range throw. We're largely
// going to focus on using the parameterized constructor (w, h, fx, fy, cx, cz)
// even though the throwing happens in the matrix-based constructor.
//
//  1. It's simpler to iterate over different kinds of bad values via this
//     constructor compactly.
//  2. We know it simply constructs a matrix and forwards it along, so we'll be
//     exercising that constructor.
//
// We will evaluate the matrix-based constructor to test for the "malformed
// matrix condition".
GTEST_TEST(TestCameraInfo, BadConstructionThrows) {
  constexpr double kInf = std::numeric_limits<double>::infinity();
  constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
  constexpr double kW = kWidth;
  constexpr double kH = kHeight;

  // Bad image size.
  DRAKE_EXPECT_THROWS_MESSAGE(CameraInfo(-13, kH, kFx, kFy, kCx, kCy),
                              "[^]*Width.+-13[^]*");
  DRAKE_EXPECT_THROWS_MESSAGE(CameraInfo(kW, -17, kFx, kFy, kCx, kCy),
                              "[^]*Height.+-17[^]*");
  // Bad focal length.
  DRAKE_EXPECT_THROWS_MESSAGE(CameraInfo(kW, kH, 0, kFy, kCx, kCy),
                              "[^]*Focal X.+0[^]*");
  DRAKE_EXPECT_THROWS_MESSAGE(CameraInfo(kW, kH, -10, kFy, kCx, kCy),
                              "[^]*Focal X.+-10[^]*");
  DRAKE_EXPECT_THROWS_MESSAGE(CameraInfo(kW, kH, kInf, kFy, kCx, kCy),
                              "[^]*Focal X.+inf[^]*");
  DRAKE_EXPECT_THROWS_MESSAGE(CameraInfo(kW, kH, kNaN, kFy, kCx, kCy),
                              "[^]*Focal X.+nan[^]*");

  DRAKE_EXPECT_THROWS_MESSAGE(CameraInfo(kW, kH, kFx, 0, kCx, kCy),
                              "[^]*Focal Y.+0[^]*");
  DRAKE_EXPECT_THROWS_MESSAGE(CameraInfo(kW, kH, kFx, -10, kCx, kCy),
                              "[^]*Focal Y.+-10[^]*");
  DRAKE_EXPECT_THROWS_MESSAGE(CameraInfo(kW, kH, kFx, kInf, kCx, kCy),
                              "[^]*Focal Y.+inf[^]*");
  DRAKE_EXPECT_THROWS_MESSAGE(CameraInfo(kW, kH, kFx, kNaN, kCx, kCy),
                              "[^]*Focal Y.+nan[^]*");

  // Bad principal point.
  DRAKE_EXPECT_THROWS_MESSAGE(CameraInfo(kW, kH, kFx, kFy, 0, kCy),
                              "[^]*Center X.+0[^]*");
  DRAKE_EXPECT_THROWS_MESSAGE(CameraInfo(kW, kH, kFx, kFy, -10, kCy),
                              "[^]*Center X.+-10[^]*");
  DRAKE_EXPECT_THROWS_MESSAGE(CameraInfo(kW, kH, kFx, kFy, kW, kCy),
                              "[^]*Center X.+\\d+[^]*");
  DRAKE_EXPECT_THROWS_MESSAGE(CameraInfo(kW, kH, kFx, kFy, kW + 1, kCy),
                              "[^]*Center X.+\\d+[^]*");

  DRAKE_EXPECT_THROWS_MESSAGE(CameraInfo(kW, kH, kFx, kFy, kCx, 0),
                              "[^]*Center Y.+0[^]*");
  DRAKE_EXPECT_THROWS_MESSAGE(CameraInfo(kW, kH, kFx, kFy, kCx, -10),
                              "[^]*Center Y.+-10[^]*");
  DRAKE_EXPECT_THROWS_MESSAGE(CameraInfo(kW, kH, kFx, kFy, kCx, kH),
                              "[^]*Center Y.+\\d+[^]*");
  DRAKE_EXPECT_THROWS_MESSAGE(CameraInfo(kW, kH, kFx, kFy, kCx, kH + 1),
                              "[^]*Center Y.+\\d+[^]*");

  // All bad values get independently enumerated.
  try {
    CameraInfo(-1, -1, -1, -1, -1, -1);
    GTEST_FAIL() << "Didn't throw an exception!";
  } catch (std::exception& e) {
    EXPECT_THAT(e.what(), ::testing::HasSubstr("Width")) << e.what();
    EXPECT_THAT(e.what(), ::testing::HasSubstr("Height")) << e.what();
    EXPECT_THAT(e.what(), ::testing::HasSubstr("Focal X")) << e.what();
    EXPECT_THAT(e.what(), ::testing::HasSubstr("Focal Y")) << e.what();
    EXPECT_THAT(e.what(), ::testing::HasSubstr("Center X")) << e.what();
    EXPECT_THAT(e.what(), ::testing::HasSubstr("Center Y")) << e.what();
  }

  // Test for a malformed matrix; we'll start with an otherwise valid matrix and
  // perturb the off-diagonal and homogeneous row entries to become "malformed".
  const Matrix3d K_valid(
      (Matrix3d() << kFy, 0., kCx, 0., kFy, kCy, 0., 0., 1.).finished());
  EXPECT_NO_THROW(CameraInfo(kW, kH, K_valid));

  const std::vector<std::pair<int, int>> indices{
      {0, 1}, {1, 0}, {2, 0}, {2, 1}, {2, 2}};
  for (auto [i, j] : indices) {
    Matrix3d K = K_valid;
    K(i, j) = -1;
    DRAKE_EXPECT_THROWS_MESSAGE(CameraInfo(kW, kH, K),
                                "[^]*intrinsic matrix is malformed[^]*");
  }
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
