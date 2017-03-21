#include "drake/systems/sensors/camera_info.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"

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
const double kCx = kWidth * 0.5;
const double kCy = kHeight * 0.5;
const double kVerticalFov = 0.78539816339744828;  // 45.0 degrees.

class CameraInfoTest : public ::testing::Test {
 public:
  CameraInfoTest() : expected_intrinsic_(
      (Eigen::Matrix3d() <<
           kFx, 0., kCx, 0., kFy, kCy, 0., 0., 1.).finished()) {}

  void SetUp() {}

  void Verify(const CameraInfo& dut) const {
    EXPECT_EQ(kWidth, dut.width());
    EXPECT_EQ(kHeight, dut.height());
    EXPECT_NEAR(kFx, dut.focal_x(), kTolerance);
    EXPECT_NEAR(kFy, dut.focal_y(), kTolerance);
    EXPECT_NEAR(kCx, dut.center_x(), kTolerance);
    EXPECT_NEAR(kCy, dut.center_y(), kTolerance);
    EXPECT_TRUE(CompareMatrices(expected_intrinsic_, dut.intrinsic_matrix(),
                                kTolerance));
  }

 private:
  const Eigen::Matrix3d expected_intrinsic_;
};

TEST_F(CameraInfoTest, ConstructionTest) {
  CameraInfo dut(kWidth, kHeight, kFx, kFy, kCx, kCy);
  Verify(dut);
}

TEST_F(CameraInfoTest, ConstructionWithFovTest) {
  CameraInfo dut(kWidth, kHeight, kVerticalFov);
  Verify(dut);
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
