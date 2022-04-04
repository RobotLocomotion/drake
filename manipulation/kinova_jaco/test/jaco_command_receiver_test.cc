#include "drake/manipulation/kinova_jaco/jaco_command_receiver.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace manipulation {
namespace kinova_jaco {
namespace {

using Eigen::VectorXd;
constexpr int N = kJacoDefaultArmNumJoints;
constexpr int N_F = kJacoDefaultArmNumFingers;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

class JacoCommandReceiverTestBase : public testing::Test {
 public:
  JacoCommandReceiverTestBase(int num_joints, int num_fingers)
      : dut_(num_joints, num_fingers),
        context_ptr_(dut_.CreateDefaultContext()),
        context_(*context_ptr_),
        fixed_input_(FixInput()) {}

  // For use only by our constructor.
  systems::FixedInputPortValue& FixInput() {
    return dut_.get_message_input_port().FixValue(
        &context_, lcmt_jaco_command{});
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

  VectorXd position() const {
    return dut_.get_commanded_position_output_port().Eval(context_);
  }

  VectorXd velocity() const {
    return dut_.get_commanded_velocity_output_port().Eval(context_);
  }

 protected:
  JacoCommandReceiver dut_;
  std::unique_ptr<systems::Context<double>> context_ptr_;
  systems::Context<double>& context_;
  systems::FixedInputPortValue& fixed_input_;
};

class JacoCommandReceiverTest : public JacoCommandReceiverTestBase {
 public:
  JacoCommandReceiverTest()
      : JacoCommandReceiverTestBase(
            kJacoDefaultArmNumJoints, kJacoDefaultArmNumFingers) {}
};

class JacoCommandReceiverNoFingersTest : public JacoCommandReceiverTestBase {
 public:
  JacoCommandReceiverNoFingersTest()
      : JacoCommandReceiverTestBase(
            kJacoDefaultArmNumJoints, 0) {}
};

TEST_F(JacoCommandReceiverTest, DeprecatedInitialPositionTest) {
  constexpr int total_dof =
      kJacoDefaultArmNumJoints + kJacoDefaultArmNumFingers;

  // Check that the commanded pose starts out at zero.
  EXPECT_TRUE(CompareMatrices(state(), VectorXd::Zero(total_dof * 2)));

  // Check that we can set a different initial position.
  const VectorXd q0 = VectorXd::LinSpaced(total_dof, 0.1, 0.2);
  dut_.set_initial_position(&context_, q0);
  EXPECT_TRUE(CompareMatrices(state().head(total_dof), q0));
  EXPECT_TRUE(CompareMatrices(state().tail(total_dof),
                              VectorXd::Zero(total_dof)));
}

#pragma GCC diagnostic pop

TEST_F(JacoCommandReceiverTest, AcceptanceTestWithoutMeasuredPositionInput) {
  // When no message has been received and *no* position measurement is
  // connected, the command is all zeros.
  const VectorXd zero = VectorXd::Zero(N + N_F);
  EXPECT_TRUE(CompareMatrices(position(), zero));
  EXPECT_TRUE(CompareMatrices(velocity(), zero));
}

TEST_F(JacoCommandReceiverNoFingersTest,
       AcceptanceTestWithoutMeasuredPositionInputNoFingers) {
  // When no message has been received and *no* position measurement is
  // connected, the command is all zeros.
  const VectorXd zero = VectorXd::Zero(N);
  EXPECT_TRUE(CompareMatrices(position(), zero));
  EXPECT_TRUE(CompareMatrices(velocity(), zero));
}

TEST_F(JacoCommandReceiverNoFingersTest,
       AcceptanceTestWithMeasuredPositionInputNoFingers) {
  const VectorXd zero = VectorXd::Zero(N);

  // When no message has been received and a measurement *is* connected, the
  // command is to hold at the current position.
  const VectorXd q0 = VectorXd::LinSpaced(N, 0.2, 0.3);
  dut_.get_position_measured_input_port().FixValue(&context_, q0);
  EXPECT_TRUE(CompareMatrices(position(), q0));
  EXPECT_TRUE(CompareMatrices(velocity(), zero));

  // Check that a real command trumps the initial position.
  const VectorXd q1 = VectorXd::LinSpaced(N, 0.3, 0.4);
  const VectorXd v1 = VectorXd::LinSpaced(N, 0.5, 0.6);
  lcmt_jaco_command command{};
  command.utime = 0;
  command.num_joints = N;
  command.joint_position = {q1.data(), q1.data() + q1.size()};
  command.joint_velocity = {v1.data(), v1.data() + v1.size()};
  SetInput(command);
  EXPECT_TRUE(CompareMatrices(position(), q1));
  EXPECT_TRUE(CompareMatrices(velocity(), v1));
}

TEST_F(JacoCommandReceiverNoFingersTest,
       AcceptanceTestWithLatchingNoFingers) {
  const VectorXd zero = VectorXd::Zero(N);

  // When no message has been received and a measurement *is* connected, the
  // command is to hold at the current position.
  const VectorXd q0 = VectorXd::LinSpaced(N, 0.0, 0.1);
  dut_.get_position_measured_input_port().FixValue(&context_, q0);
  EXPECT_TRUE(CompareMatrices(position(), q0));
  EXPECT_TRUE(CompareMatrices(velocity(), zero));

  // Prior to any update events, changes to position_measured feed through.
  const VectorXd q1 = VectorXd::LinSpaced(N, 0.1, 0.2);
  dut_.get_position_measured_input_port().FixValue(&context_, q1);
  EXPECT_TRUE(CompareMatrices(position(), q1));
  EXPECT_TRUE(CompareMatrices(velocity(), zero));

  // Once an update event occurs, further changes to position_measured have no
  // effect.
  dut_.LatchInitialPosition(&context_);
  const VectorXd q2 = VectorXd::LinSpaced(N, 0.3, 0.4);
  dut_.get_position_measured_input_port().FixValue(&context_, q2);
  EXPECT_TRUE(CompareMatrices(position(), q1));
  EXPECT_TRUE(CompareMatrices(velocity(), zero));

  // Check that a real command trumps the initial position.
  const VectorXd q3 = VectorXd::LinSpaced(N, 0.4, 0.5);
  const VectorXd v3 = VectorXd::LinSpaced(N, 0.5, 0.6);
  lcmt_jaco_command command{};
  command.utime = 0;
  command.num_joints = N;
  command.joint_position = {q3.data(), q3.data() + q3.size()};
  command.joint_velocity = {v3.data(), v3.data() + v3.size()};
  SetInput(command);
  EXPECT_TRUE(CompareMatrices(position(), q3));
  EXPECT_TRUE(CompareMatrices(velocity(), v3));
}

TEST_F(JacoCommandReceiverTest,
       AcceptanceTestWithMeasuredPositionInput) {
  const VectorXd zero = VectorXd::Zero(N + N_F);

  // When no message has been received and a measurement *is* connected, the
  // command is to hold at the current position.
  const VectorXd q0 = VectorXd::LinSpaced(N + N_F, 0.2, 0.3);
  dut_.get_position_measured_input_port().FixValue(&context_, q0);
  EXPECT_TRUE(CompareMatrices(position(), q0));
  EXPECT_TRUE(CompareMatrices(velocity(), zero));

  // Check that a real command trumps the initial position.
  const VectorXd q1 = VectorXd::LinSpaced(N, 0.3, 0.4);
  const VectorXd v1 = VectorXd::LinSpaced(N, 0.5, 0.6);
  const VectorXd f_q1 = VectorXd::LinSpaced(N_F, 1.3, 1.4);
  const VectorXd f_v1 = VectorXd::LinSpaced(N_F, 1.5, 1.6);
  lcmt_jaco_command command{};
  command.utime = 0;
  command.num_joints = N;
  command.joint_position = {q1.data(), q1.data() + q1.size()};
  command.joint_velocity = {v1.data(), v1.data() + v1.size()};
  command.num_fingers = N_F;
  command.finger_position = {f_q1.data(), f_q1.data() + f_q1.size()};
  command.finger_velocity = {f_v1.data(), f_v1.data() + f_v1.size()};
  SetInput(command);

  VectorXd position_expected(N + N_F);
  position_expected.head(N) = q1;
  position_expected.tail(N_F) = f_q1 * kFingerSdkToUrdf;

  VectorXd velocity_expected(N + N_F);
  velocity_expected.head(N) = v1;
  velocity_expected.tail(N_F) = f_v1 * kFingerSdkToUrdf;

  EXPECT_TRUE(CompareMatrices(position(), position_expected));
  EXPECT_TRUE(CompareMatrices(velocity(), velocity_expected));
}

TEST_F(JacoCommandReceiverTest, AcceptanceTestWithLatching) {
  const VectorXd zero = VectorXd::Zero(N + N_F);

  // When no message has been received and a measurement *is* connected, the
  // command is to hold at the current position.
  const VectorXd q0 = VectorXd::LinSpaced(N + N_F, 0.0, 0.1);
  dut_.get_position_measured_input_port().FixValue(&context_, q0);
  EXPECT_TRUE(CompareMatrices(position(), q0));
  EXPECT_TRUE(CompareMatrices(velocity(), zero));

  // Prior to any update events, changes to position_measured feed through.
  const VectorXd q1 = VectorXd::LinSpaced(N + N_F, 0.1, 0.2);
  dut_.get_position_measured_input_port().FixValue(&context_, q1);
  EXPECT_TRUE(CompareMatrices(position(), q1));
  EXPECT_TRUE(CompareMatrices(velocity(), zero));

  // Once an update event occurs, further changes to position_measured have no
  // effect.
  dut_.LatchInitialPosition(&context_);
  const VectorXd q2 = VectorXd::LinSpaced(N + N_F, 0.3, 0.4);
  dut_.get_position_measured_input_port().FixValue(&context_, q2);
  EXPECT_TRUE(CompareMatrices(position(), q1));
  EXPECT_TRUE(CompareMatrices(velocity(), zero));

  // Check that a real command trumps the initial position.
  const VectorXd q3 = VectorXd::LinSpaced(N, 0.4, 0.5);
  const VectorXd v3 = VectorXd::LinSpaced(N, 0.5, 0.6);
  const VectorXd f_q3 = VectorXd::LinSpaced(N_F, 0.4, 0.5);
  const VectorXd f_v3 = VectorXd::LinSpaced(N_F, 1.5, 1.6);
  lcmt_jaco_command command{};
  command.utime = 0;
  command.num_joints = N;
  command.num_fingers = N_F;
  command.joint_position = {q3.data(), q3.data() + q3.size()};
  command.joint_velocity = {v3.data(), v3.data() + v3.size()};
  command.finger_position = {f_q3.data(), f_q3.data() + f_q3.size()};
  command.finger_velocity = {f_v3.data(), f_v3.data() + f_v3.size()};
  SetInput(command);

  VectorXd position_expected(N + N_F);
  position_expected.head(N) = q3;
  position_expected.tail(N_F) = f_q3 * kFingerSdkToUrdf;

  VectorXd velocity_expected(N + N_F);
  velocity_expected.head(N) = v3;
  velocity_expected.tail(N_F) = f_v3 * kFingerSdkToUrdf;
  EXPECT_TRUE(CompareMatrices(position(), position_expected));
  EXPECT_TRUE(CompareMatrices(velocity(), velocity_expected));
}

}  // namespace
}  // namespace kinova_jaco
}  // namespace manipulation
}  // namespace drake
