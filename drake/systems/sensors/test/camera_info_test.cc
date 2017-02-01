#include "drake/systems/sensors/camera_info.h"

#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"

namespace drake {
namespace systems {
namespace sensors {
namespace {

const double kTolerance = 1e-10;

const int kWidth = 640;
const int kHeight = 480;
const double kFx = 554.25625842204079;  // in pixels
const double kFy = 579.41125496954282;  // in pixels
const double kCx = kWidth * 0.5;
const double kCy = kHeight * 0.5;
const double kVerticalFovRad = 0.78539816339744828;  // = 45.0 in degrees

GTEST_TEST(TestCameraInfo, ConstructionTest) {
  CameraInfo dut(kWidth, kHeight, kFx, kFy, kCx, kCy);

  Eigen::Matrix3d expected_intrinsic;
  expected_intrinsic << kFx, 0., kCx, 0., kFy, kCy, 0., 0., 1.;

  EXPECT_EQ(kWidth, dut.width());
  EXPECT_EQ(kHeight, dut.height());
  EXPECT_NEAR(kFx, dut.fx(), kTolerance);
  EXPECT_NEAR(kFy, dut.fy(), kTolerance);
  EXPECT_NEAR(kCx, dut.cx(), kTolerance);
  EXPECT_NEAR(kCy, dut.cy(), kTolerance);
  EXPECT_TRUE(CompareMatrices(expected_intrinsic, dut.intrinsic_matrix(),
                              kTolerance));
}

GTEST_TEST(TestCameraInfo, ConstructionWithFovTest) {
  CameraInfo dut(kWidth, kHeight, kVerticalFovRad);

  Eigen::Matrix3d expected_intrinsic;
  expected_intrinsic << kFx, 0., kCx, 0., kFy, kCy, 0., 0., 1.;

  EXPECT_EQ(kWidth, dut.width());
  EXPECT_EQ(kHeight, dut.height());
  EXPECT_NEAR(kFx, dut.fx(), kTolerance);
  EXPECT_NEAR(kFy, dut.fy(), kTolerance);
  EXPECT_NEAR(kCx, dut.cx(), kTolerance);
  EXPECT_NEAR(kCy, dut.cy(), kTolerance);
  EXPECT_TRUE(CompareMatrices(expected_intrinsic, dut.intrinsic_matrix(),
                              kTolerance));
}

GTEST_TEST(TestCameraInfo, CopyConstructorTest) {
  CameraInfo camera_info(kWidth, kHeight, kFx, kFy, kCx, kCy);
  CameraInfo dut(camera_info);

  Eigen::Matrix3d expected_intrinsic;
  expected_intrinsic << kFx, 0., kCx, 0., kFy, kCy, 0., 0., 1.;

  EXPECT_EQ(kWidth, dut.width());
  EXPECT_EQ(kHeight, dut.height());
  EXPECT_NEAR(kFx, dut.fx(), kTolerance);
  EXPECT_NEAR(kFy, dut.fy(), kTolerance);
  EXPECT_NEAR(kCx, dut.cx(), kTolerance);
  EXPECT_NEAR(kCy, dut.cy(), kTolerance);
  EXPECT_TRUE(CompareMatrices(expected_intrinsic, dut.intrinsic_matrix(),
                              kTolerance));
}

GTEST_TEST(TestCameraInfo, MoveConstructorTest) {
  CameraInfo camera_info(kWidth, kHeight, kFx, kFy, kCx, kCy);
  CameraInfo dut(std::move(camera_info));

  Eigen::Matrix3d expected_intrinsic;
  expected_intrinsic << kFx, 0., kCx, 0., kFy, kCy, 0., 0., 1.;

  EXPECT_EQ(kWidth, dut.width());
  EXPECT_EQ(kHeight, dut.height());
  EXPECT_NEAR(kFx, dut.fx(), kTolerance);
  EXPECT_NEAR(kFy, dut.fy(), kTolerance);
  EXPECT_NEAR(kCx, dut.cx(), kTolerance);
  EXPECT_NEAR(kCy, dut.cy(), kTolerance);
  EXPECT_TRUE(CompareMatrices(expected_intrinsic, dut.intrinsic_matrix(),
                              kTolerance));
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
