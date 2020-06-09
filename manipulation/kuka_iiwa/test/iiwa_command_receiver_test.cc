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

  void UpdateFallbackValue() {
    auto events = dut_.AllocateCompositeEventCollection();
    double next_event_time = dut_.CalcNextUpdateTime(context_, events.get());
    if (next_event_time == context_.get_time()) {
      const auto& discrete_events = events->get_discrete_update_events();
      DRAKE_THROW_UNLESS(discrete_events.HasEvents());
      auto discrete_vars = dut_.AllocateDiscreteVariables();
      dut_.CalcDiscreteVariableUpdates(context_, discrete_events,
                                       discrete_vars.get());
      dut_.ApplyDiscreteVariableUpdate(discrete_events, discrete_vars.get(),
                                       &context_);
    }
  }

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
  UpdateFallbackValue();
  EXPECT_TRUE(CompareMatrices(position(), zero));
  EXPECT_TRUE(CompareMatrices(torque(), zero));
}

TEST_F(IiwaCommandReceiverTest, AcceptanceTestWithMeasuredPositionInput) {
  const VectorXd zero = VectorXd::Zero(N);
  // When no message has been received and a measurement *is* connected, the
  // command is to hold at the current position.
  const VectorXd q0 = VectorXd::LinSpaced(N, 0.2, 0.3);
  dut_.get_position_measured_input_port().FixValue(&context_, q0);
  UpdateFallbackValue();
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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
TEST_F(IiwaCommandReceiverTest,
       DeprecatedAcceptanceTestWithoutMeasuredPositionInput) {
  // Check that the commanded pose starts out at zero
  const VectorXd zero = VectorXd::Zero(N);
  UpdateFallbackValue();
  EXPECT_TRUE(CompareMatrices(position(), zero));
  EXPECT_TRUE(CompareMatrices(torque(), zero));
}

TEST_F(IiwaCommandReceiverTest,
       DeprecatedAcceptanceTestWithMeasuredPositionInput) {
  const VectorXd zero = VectorXd::Zero(N);
  // Check that we can set a different initial position.
  const VectorXd q0 = VectorXd::LinSpaced(N, 0.1, 0.2);
  dut_.set_initial_position(&context_, q0);
  UpdateFallbackValue();
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
#pragma GCC diagnostic pop

}  // namespace
}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
