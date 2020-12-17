#include "drake/manipulation/kuka_iiwa/iiwa_command_receiver.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {
namespace {

using Eigen::VectorXd;
constexpr int N = kIiwaArmNumJoints;

class IiwaCommandReceiverTest : public testing::Test {
 public:
  IiwaCommandReceiverTest()
      : dut_(),
        context_ptr_(dut_.CreateDefaultContext()),
        context_(*context_ptr_),
        fixed_input_(FixInput()) {}

  // For use only by our constructor.
  systems::FixedInputPortValue& FixInput() {
    return dut_.get_message_input_port().FixValue(
        &context_, lcmt_iiwa_command{});
  }

  // Test cases should call this to set the DUT's input value.
  void SetInput(lcmt_iiwa_command message) {
    // TODO(jwnimmer-tri) This systems framework API is not very ergonomic.
    fixed_input_.GetMutableData()->
        template get_mutable_value<lcmt_iiwa_command>() = message;
  }

  VectorXd position() const {
    const auto& port = dut_.get_commanded_position_output_port();
    const VectorXd result = port.Eval(context_);
    return result;
  }

  VectorXd torque() const {
    return dut_.get_commanded_torque_output_port().Eval(context_);
  }

 protected:
  IiwaCommandReceiver dut_;
  std::unique_ptr<systems::Context<double>> context_ptr_;
  systems::Context<double>& context_;
  systems::FixedInputPortValue& fixed_input_;
};

TEST_F(IiwaCommandReceiverTest, AcceptanceTestWithoutMeasuredPositionInput) {
  // When no message has been received and *no* position measurement is
  // connected, the command is all zeros.
  const VectorXd zero = VectorXd::Zero(N);
  EXPECT_TRUE(CompareMatrices(position(), zero));
  EXPECT_TRUE(CompareMatrices(torque(), zero));
}

TEST_F(IiwaCommandReceiverTest, AcceptanceTestWithMeasuredPositionInput) {
  const VectorXd zero = VectorXd::Zero(N);

  // When no message has been received and a measurement *is* connected, the
  // command is to hold at the current position.
  const VectorXd q0 = VectorXd::LinSpaced(N, 0.2, 0.3);
  dut_.get_position_measured_input_port().FixValue(&context_, q0);
  EXPECT_TRUE(CompareMatrices(position(), q0));
  EXPECT_TRUE(CompareMatrices(torque(), zero));

  // Check that a real command trumps the initial position.
  // First, try with empty torques.
  const VectorXd q1 = VectorXd::LinSpaced(N, 0.3, 0.4);
  lcmt_iiwa_command command{};
  command.utime = 0;
  command.num_joints = N;
  command.joint_position = {q1.data(), q1.data() + q1.size()};
  SetInput(command);
  EXPECT_TRUE(CompareMatrices(position(), q1));
  EXPECT_TRUE(CompareMatrices(torque(), zero));

  // Now provide torques.
  const VectorXd t1 = VectorXd::LinSpaced(N, 0.5, 0.6);
  command.num_torques = N;
  command.joint_torque = {t1.data(), t1.data() + t1.size()};
  SetInput(command);
  EXPECT_TRUE(CompareMatrices(position(), q1));
  EXPECT_TRUE(CompareMatrices(torque(), t1));
}

TEST_F(IiwaCommandReceiverTest, AcceptanceTestWithLatching) {
  const VectorXd zero = VectorXd::Zero(N);

  // When no message has been received and a measurement *is* connected, the
  // command is to hold at the current position.
  const VectorXd q0 = VectorXd::LinSpaced(N, 0.0, 0.1);
  dut_.get_position_measured_input_port().FixValue(&context_, q0);
  EXPECT_TRUE(CompareMatrices(position(), q0));
  EXPECT_TRUE(CompareMatrices(torque(), zero));

  // Prior to any update events, changes to position_measured feed through.
  const VectorXd q1 = VectorXd::LinSpaced(N, 0.1, 0.2);
  dut_.get_position_measured_input_port().FixValue(&context_, q1);
  EXPECT_TRUE(CompareMatrices(position(), q1));
  EXPECT_TRUE(CompareMatrices(torque(), zero));

  // Once an update event occurs, further changes to position_measured have no
  // effect.
  dut_.LatchInitialPosition(&context_);
  const VectorXd q2 = VectorXd::LinSpaced(N, 0.3, 0.4);
  dut_.get_position_measured_input_port().FixValue(&context_, q2);
  EXPECT_TRUE(CompareMatrices(position(), q1));
  EXPECT_TRUE(CompareMatrices(torque(), zero));

  // Check that a real command trumps the initial position.
  // First, try with empty torques.
  const VectorXd q3 = VectorXd::LinSpaced(N, 0.4, 0.5);
  const VectorXd t3 = VectorXd::LinSpaced(N, 0.5, 0.6);
  lcmt_iiwa_command command{};
  command.utime = 0;
  command.num_joints = N;
  command.joint_position = {q3.data(), q3.data() + q3.size()};
  command.num_torques = N;
  command.joint_torque = {t3.data(), t3.data() + t3.size()};
  SetInput(command);
  EXPECT_TRUE(CompareMatrices(position(), q3));
  EXPECT_TRUE(CompareMatrices(torque(), t3));
}

}  // namespace
}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
