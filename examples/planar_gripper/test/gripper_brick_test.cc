#include "drake/examples/planar_gripper/gripper_brick.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace examples {
namespace planar_gripper {
GTEST_TEST(GripperBrickHelperTest, Test) {
  GripperBrickHelper<double> dut;
  EXPECT_TRUE(CompareMatrices(dut.p_L2Tip(), Eigen::Vector3d(0, 0, -0.086)));
  EXPECT_EQ(dut.finger_tip_radius(), 0.015);
  EXPECT_TRUE(
      CompareMatrices(dut.brick_size(), Eigen::Vector3d(0.025, 0.092, 0.092)));
}
}  // namespace planar_gripper
}  // namespace examples
}  // namespace drake
