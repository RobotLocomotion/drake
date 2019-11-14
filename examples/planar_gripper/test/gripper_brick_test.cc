#include "drake/examples/planar_gripper/gripper_brick.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace examples {
namespace planar_gripper {
GTEST_TEST(GripperBrickHelperTest, Test) {
  GripperBrickHelper<double> dut;
  EXPECT_TRUE(CompareMatrices(dut.p_L2Fingertip(),
                              Eigen::Vector3d(0, 0, -0.0713), 1E-16));
  EXPECT_EQ(dut.finger_tip_radius(), 0.015);
  EXPECT_TRUE(
      CompareMatrices(dut.brick_size(), Eigen::Vector3d(0.07, 0.1, 0.1)));

  auto diagram_context = dut.diagram().CreateDefaultContext();
  systems::Context<double>* plant_mutable_context =
      &(dut.diagram().GetMutableSubsystemContext(dut.plant(),
                                                 diagram_context.get()));

  Eigen::VectorXd q(9);
  // Set q to arbitrary values.
  q << 0.2, 0.4, 0.6, 0.9, 1.2, 1.4, 1.6, 1.8, 2;
  dut.plant().SetPositions(plant_mutable_context, q);

  // Now evaluate the pose of link 2 for each finger.
  for (Finger finger : {Finger::kFinger1, Finger::kFinger2, Finger::kFinger3}) {
    const auto X_WLink2 = dut.plant().CalcRelativeTransform(
        *plant_mutable_context, dut.plant().world_frame(),
        dut.finger_link2_frame(finger));
    const Eigen::AngleAxis<double> angle_axis =
        X_WLink2.rotation().ToAngleAxis();
    const double theta = dut.CalcFingerLink2Orientation(
        finger, q(dut.finger_base_position_index(finger)),
        q(dut.finger_mid_position_index(finger)));
    if (angle_axis.axis().dot(Eigen::Vector3d::UnitX()) > 1 - 1E-14) {
      EXPECT_NEAR(std::fmod(theta - angle_axis.angle(), 2 * M_PI), 0, 1E-14);
    } else if (angle_axis.axis().dot(-Eigen::Vector3d::UnitX()) > 1 - 1E-14) {
      EXPECT_NEAR(std::fmod(theta + angle_axis.angle(), 2 * M_PI), 0, 1E-14);
    } else {
      throw std::runtime_error("The rotation axis should be x axis.");
    }
  }
}
}  // namespace planar_gripper
}  // namespace examples
}  // namespace drake
