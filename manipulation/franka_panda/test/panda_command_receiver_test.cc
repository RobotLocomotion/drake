#include "drake/manipulation/franka_panda/panda_command_receiver.h"

#include <memory>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/lcmt_panda_status.hpp"

namespace drake {
namespace manipulation {
namespace franka_panda {

using drake::CompareMatrices;
using drake::lcmt_panda_command;
using drake::lcmt_panda_status;
using drake::systems::Context;
using drake::systems::FixedInputPortValue;
using Eigen::VectorXd;
constexpr int N = kPandaArmNumJoints;

class PandaCommandReceiverTest : public testing::Test {
 public:
  PandaCommandReceiverTest()
      : dut_(kPandaArmNumJoints,
             PandaControlModes::kPosition | PandaControlModes::kTorque),
        context_ptr_(dut_.CreateDefaultContext()),
        context_(*context_ptr_),
        fixed_input_(FixInput()) {}

  // For use only by our constructor.
  FixedInputPortValue& FixInput() {
    return dut_.get_message_input_port().FixValue(&context_,
                                                  lcmt_panda_command{});
  }

  // Test cases should call this to set the DUT's input value.
  void SetInput(lcmt_panda_command message) {
    // TODO(jwnimmer-tri) This systems framework API is not very ergonomic.
    fixed_input_.GetMutableData()
        ->template get_mutable_value<lcmt_panda_command>() = message;
  }

  VectorXd position() const {
    const auto& port = dut_.get_commanded_position_output_port();
    const VectorXd result = port.Eval(context_);
    return result;
  }

  VectorXd velocity() const {
    return dut_.get_commanded_velocity_output_port().Eval(context_);
  }

  VectorXd torque() const {
    return dut_.get_commanded_torque_output_port().Eval(context_);
  }

 protected:
  PandaCommandReceiver dut_;
  std::unique_ptr<Context<double>> context_ptr_;
  Context<double>& context_;
  FixedInputPortValue& fixed_input_;
};

TEST_F(PandaCommandReceiverTest, AcceptanceTestWithMeasuredPositionInput) {
  const VectorXd zero = VectorXd::Zero(N);

  // We cannot query commanded velocity.
  EXPECT_THROW(velocity(), std::runtime_error);

  // When no message has been received and a measurement *is* connected, the
  // command is to hold at the current position.
  const VectorXd q0 = VectorXd::LinSpaced(N, 0.2, 0.3);
  dut_.get_position_measured_input_port().FixValue(&context_, q0);
  EXPECT_TRUE(CompareMatrices(position(), q0));
  EXPECT_TRUE(CompareMatrices(torque(), zero));

  // Check that a real command trumps the initial position.
  const VectorXd q1 = VectorXd::LinSpaced(N, 0.3, 0.4);
  const VectorXd t1 = VectorXd::LinSpaced(N, 0.5, 0.6);
  lcmt_panda_command command{};
  command.control_mode_expected =
      to_int(PandaControlModes::kPosition | PandaControlModes::kTorque);
  command.utime = 0;
  command.num_joint_position = N;
  command.joint_position = {q1.data(), q1.data() + q1.size()};
  command.num_joint_torque = N;
  command.joint_torque = {t1.data(), t1.data() + t1.size()};
  SetInput(command);
  EXPECT_TRUE(CompareMatrices(position(), q1));
  EXPECT_THROW(velocity(), std::runtime_error);
  EXPECT_TRUE(CompareMatrices(torque(), t1));
}

TEST_F(PandaCommandReceiverTest, AcceptanceTestWithLatching) {
  const VectorXd zero = VectorXd::Zero(N);

  // We cannot query commanded velocity.
  EXPECT_THROW(velocity(), std::runtime_error);

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
  const VectorXd v3 = VectorXd::LinSpaced(N, 0.5, 0.6);
  const VectorXd t3 = VectorXd::LinSpaced(N, 0.6, 0.7);
  lcmt_panda_command command{};
  command.control_mode_expected =
      to_int(PandaControlModes::kPosition | PandaControlModes::kTorque);
  command.utime = 0;
  command.num_joint_position = N;
  command.joint_position = {q3.data(), q3.data() + q3.size()};
  command.num_joint_torque = N;
  command.joint_torque = {t3.data(), t3.data() + t3.size()};
  SetInput(command);
  EXPECT_TRUE(CompareMatrices(position(), q3));
  EXPECT_TRUE(CompareMatrices(torque(), t3));
}

GTEST_TEST(PandaCommandReceiverTestVelocity, VelocityControl) {
  PandaCommandReceiver receiver(kPandaArmNumJoints,
                                PandaControlModes::kVelocity);
  EXPECT_EQ(receiver.num_discrete_state_groups(), 0);

  EXPECT_THROW(receiver.get_commanded_position_output_port(),
               std::runtime_error);
  EXPECT_THROW(receiver.get_commanded_torque_output_port(), std::runtime_error);

  lcmt_panda_command command{};
  command.control_mode_expected = to_int(PandaControlModes::kVelocity);
  command.num_joint_velocity = kPandaArmNumJoints;
  command.joint_velocity.resize(kPandaArmNumJoints, 0.1);

  auto context = receiver.CreateDefaultContext();
  receiver.get_message_input_port().FixValue(context.get(), command);
  const VectorXd velocity =
      receiver.get_commanded_velocity_output_port().Eval(*context);
  EXPECT_TRUE(CompareMatrices(velocity, VectorXd::Constant(7, 0.1)));
}

TEST_F(PandaCommandReceiverTest, BadControlMode) {
  EXPECT_THROW(PandaCommandReceiver(7, static_cast<PandaControlMode>(0x9999)),
               std::exception);
}

}  // namespace franka_panda
}  // namespace manipulation
}  // namespace drake
