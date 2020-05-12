#include "drake/manipulation/kinova_jaco/jaco_status_receiver.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace manipulation {
namespace kinova_jaco {
namespace {

using Eigen::VectorXd;

class JacoStatusReceiverTest : public testing::Test {
 public:
  JacoStatusReceiverTest()
      : dut_(),
        context_ptr_(dut_.CreateDefaultContext()),
        context_(*context_ptr_),
        fixed_input_(
            dut_.get_input_port().FixValue(&context_, lcmt_jaco_status{})) {}

  void Copy(const Eigen::VectorXd& from, std::vector<double>* to) {
    *to = {from.data(), from.data() + from.size()};
  }

 protected:
  JacoStatusReceiver dut_;
  std::unique_ptr<systems::Context<double>> context_ptr_;
  systems::Context<double>& context_;
  systems::FixedInputPortValue& fixed_input_;
  lcmt_jaco_status status_{};
};

TEST_F(JacoStatusReceiverTest, AcceptanceTest) {
  // Confirm that output is zero for uninitialized lcm input.
  const int num_output_ports = dut_.num_output_ports();
  ASSERT_EQ(num_output_ports, 4);

  constexpr int total_dof =
      kJacoDefaultArmNumJoints + kJacoDefaultArmNumFingers;

  for (int i = 0; i < num_output_ports; ++i) {
    const systems::LeafSystem<double>& leaf = dut_;
    const auto& port = leaf.get_output_port(i);
    if (i == 0) {
      EXPECT_TRUE(
          CompareMatrices(port.Eval(context_), VectorXd::Zero(total_dof * 2)));
    } else {
      EXPECT_TRUE(CompareMatrices(
          port.Eval(context_), VectorXd::Zero(total_dof)));
    }
  }

  // Populate the status message with distinct values.
  const VectorXd state = VectorXd::LinSpaced(total_dof * 2, 0.0, 1.0);
  const VectorXd torque = VectorXd::LinSpaced(total_dof, 2.0, 3.0);
  const VectorXd torque_external = VectorXd::LinSpaced(total_dof, 4.0, 5.0);
  const VectorXd current = VectorXd::LinSpaced(total_dof, 6.0, 7.0);

  status_.utime = 1;
  status_.num_joints = kJacoDefaultArmNumJoints;
  status_.num_fingers = kJacoDefaultArmNumFingers;
  Copy(state.head(kJacoDefaultArmNumJoints), &status_.joint_position);
  Copy(state.segment(total_dof, kJacoDefaultArmNumJoints) / 2,
       &status_.joint_velocity);
  Copy(state.segment(kJacoDefaultArmNumJoints, kJacoDefaultArmNumFingers) *
       kFingerUrdfToSdk,
       &status_.finger_position);
  Copy(state.tail(kJacoDefaultArmNumFingers) * kFingerUrdfToSdk,
       &status_.finger_velocity);
  Copy(torque.head(kJacoDefaultArmNumJoints), &status_.joint_torque);
  Copy(torque.tail(kJacoDefaultArmNumFingers), &status_.finger_torque);
  Copy(torque_external.head(kJacoDefaultArmNumJoints),
       &status_.joint_torque_external);
  Copy(torque_external.tail(kJacoDefaultArmNumFingers),
       &status_.finger_torque_external);
  Copy(current.head(kJacoDefaultArmNumJoints), &status_.joint_current);
  Copy(current.tail(kJacoDefaultArmNumFingers), &status_.finger_current);

  // TODO(jwnimmer-tri) This systems framework API is not very ergonomic.
  fixed_input_.GetMutableData()->
      template get_mutable_value<lcmt_jaco_status>() = status_;

  // Confirm that real message values are output correctly.
  EXPECT_TRUE(CompareMatrices(
      dut_.get_state_output_port().Eval(context_), state, 1e-15));
  EXPECT_TRUE(CompareMatrices(
      dut_.get_torque_output_port().Eval(context_), torque));
  EXPECT_TRUE(CompareMatrices(
      dut_.get_torque_external_output_port().Eval(context_),
      torque_external));
  EXPECT_TRUE(CompareMatrices(
      dut_.get_current_output_port().Eval(context_), current));
}

}  // namespace
}  // namespace kinova_jaco
}  // namespace manipulation
}  // namespace drake
