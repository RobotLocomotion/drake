#include "drake/manipulation/kinova_jaco/jaco_command_receiver.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace manipulation {
namespace kinova_jaco {
namespace {

using Eigen::VectorXd;

class JacoCommandReceiverTest : public testing::Test {
 public:
  JacoCommandReceiverTest()
      : dut_(),
        context_ptr_(dut_.CreateDefaultContext()),
        context_(*context_ptr_),
        fixed_input_(FixInput()) {}

  // For use only by our constructor.
  systems::FixedInputPortValue& FixInput() {
    return dut_.get_input_port().FixValue(&context_, lcmt_jaco_command{});
  }

  // Test cases should call this to set the DUT's input value.
  void SetInput(const lcmt_jaco_command& message) {
    // TODO(jwnimmer-tri) This systems framework API is not very ergonomic.
    fixed_input_.GetMutableData()->
        template get_mutable_value<lcmt_jaco_command>() = message;
  }

  VectorXd state() const {
    return dut_.get_output_port().Eval(context_);
  }

 protected:
  JacoCommandReceiver dut_;
  std::unique_ptr<systems::Context<double>> context_ptr_;
  systems::Context<double>& context_;
  systems::FixedInputPortValue& fixed_input_;
};

TEST_F(JacoCommandReceiverTest, AcceptanceTest) {
  constexpr int total_dof =
      kJacoDefaultArmNumJoints + kJacoDefaultArmNumFingers;

  // Check that the commanded pose starts out at zero
  EXPECT_TRUE(CompareMatrices(state(), VectorXd::Zero(total_dof * 2)));

  // Check that we can set a different initial position.
  const VectorXd q0 = VectorXd::LinSpaced(total_dof, 0.1, 0.2);
  dut_.set_initial_position(&context_, q0);
  EXPECT_TRUE(CompareMatrices(state().head(total_dof), q0));
  EXPECT_TRUE(CompareMatrices(state().tail(total_dof),
                              VectorXd::Zero(total_dof)));

  // Check that a real command trumps the initial position.
  const VectorXd q1_arm =
      VectorXd::LinSpaced(kJacoDefaultArmNumJoints, 0.3, 0.4);
  const VectorXd v1_arm =
      VectorXd::LinSpaced(kJacoDefaultArmNumJoints, 0.5, 0.6);
  const VectorXd q1_finger =
      VectorXd::LinSpaced(kJacoDefaultArmNumFingers, 1.3, 1.4) *
      kFingerUrdfToSdk;
  const VectorXd v1_finger =
      VectorXd::LinSpaced(kJacoDefaultArmNumFingers, 1.5, 1.6) *
      kFingerUrdfToSdk;
  lcmt_jaco_command command{};
  command.utime = 0;
  command.num_joints = kJacoDefaultArmNumJoints;
  command.joint_position = {q1_arm.data(), q1_arm.data() + q1_arm.size()};
  command.joint_velocity = {v1_arm.data(), v1_arm.data() + v1_arm.size()};
  command.num_fingers = kJacoDefaultArmNumFingers;
  command.finger_position =
      {q1_finger.data(), q1_finger.data() + q1_finger.size()};
  command.finger_velocity =
      {v1_finger.data(), v1_finger.data() + v1_finger.size()};
  SetInput(command);
  EXPECT_TRUE(CompareMatrices(state().head(kJacoDefaultArmNumJoints), q1_arm));
  EXPECT_TRUE(CompareMatrices(state().segment(
      total_dof, kJacoDefaultArmNumJoints), v1_arm));

  EXPECT_TRUE(CompareMatrices(state().segment(
      kJacoDefaultArmNumJoints, kJacoDefaultArmNumFingers), q1_finger *
                              kFingerSdkToUrdf));
  EXPECT_TRUE(
      CompareMatrices(state().tail(kJacoDefaultArmNumFingers),
                      v1_finger * kFingerSdkToUrdf));
}

}  // namespace
}  // namespace kinova_jaco
}  // namespace manipulation
}  // namespace drake
