#include "drake/examples/kuka_iiwa_arm/pick_and_place/world_state.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {
namespace {

GTEST_TEST(PickAndPlaceWorldStateTest, EndEffectorTest) {
  WorldState dut;

  Isometry3<double> iiwa_base{Isometry3<double>::Identity()};

  lcmt_iiwa_status iiwa_msg{};
  iiwa_msg.utime = 1000;
  iiwa_msg.num_joints = kIiwaArmNumJoints;

  // Arbitrary position/velocity taken from an LCM message emitted by
  // a running test.
  iiwa_msg.joint_position_measured.push_back(-0.5707351);
  iiwa_msg.joint_position_measured.push_back(0.979246);
  iiwa_msg.joint_position_measured.push_back(0.8769545);
  iiwa_msg.joint_position_measured.push_back(-0.72);
  iiwa_msg.joint_position_measured.push_back(0.4279);
  iiwa_msg.joint_position_measured.push_back(0.674535);
  iiwa_msg.joint_position_measured.push_back(-1.325);
  iiwa_msg.joint_velocity_estimated.push_back(-0.381015);
  iiwa_msg.joint_velocity_estimated.push_back(0.653732);
  iiwa_msg.joint_velocity_estimated.push_back(0.5854421);
  iiwa_msg.joint_velocity_estimated.push_back(-0.4807268);
  iiwa_msg.joint_velocity_estimated.push_back(0.45032358);
  iiwa_msg.joint_velocity_estimated.push_back(-0.8845549);
  iiwa_msg.joint_velocity_estimated.push_back(0.0);

  dut.HandleIiwaStatus(iiwa_msg, iiwa_base);

  EXPECT_EQ(dut.get_iiwa_time(), iiwa_msg.utime * 1e-6);
  EXPECT_TRUE(dut.get_iiwa_base().isApprox(Isometry3<double>::Identity()));
}

}  // namespace
}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
