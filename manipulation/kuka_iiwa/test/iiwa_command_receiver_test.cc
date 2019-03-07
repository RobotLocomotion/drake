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

class IiwaCommandReceiverTest : public testing::TestWithParam<int> {
 public:
  IiwaCommandReceiverTest()
      : dut_(),
        context_ptr_(dut_.CreateDefaultContext()),
        context_(*context_ptr_),
        fixed_input_(FixInput()) {}

  // For use only by our constructor.
  systems::FixedInputPortValue& FixInput() {
    switch (GetParam()) {
      case 0:
        return context_.FixInputPort(
            dut_.get_input_port().get_index(),
            Value<lcmt_iiwa_command>{});
      case 1:
        return context_.FixInputPort(
            dut_.GetInputPort("command_message").get_index(),
            Value<lcmt_iiwa_command>{});
    }
    throw std::logic_error("Bad param");
  }

  // Test cases should call this to set the DUT's input value.
  void SetInput(lcmt_iiwa_command message) {
    switch (GetParam()) {
      case 0:
      case 1: {
        // TODO(jwnimmer-tri) This systems framework API is not very ergonomic.
        fixed_input_.GetMutableData()->
            template get_mutable_value<lcmt_iiwa_command>() = message;
        return;
      }
    }
    throw std::logic_error("Bad param");
  }

  VectorXd position() const {
    const auto& port = dut_.get_commanded_position_output_port();
    const VectorXd result = port.Eval(context_);
    EXPECT_EQ(state().head(N), result);  // Sanity check the deprecated output.
    return result;
  }

  VectorXd torque() const {
    return dut_.get_commanded_torque_output_port().Eval(context_);
  }

  VectorXd state() const {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    return dut_.get_commanded_state_output_port().Eval(context_);
#pragma GCC diagnostic pop
  }

 protected:
  IiwaCommandReceiver dut_;
  std::unique_ptr<systems::Context<double>> context_ptr_;
  systems::Context<double>& context_;
  systems::FixedInputPortValue& fixed_input_;
};

TEST_P(IiwaCommandReceiverTest, AcceptanceTest) {
  // Check that the commanded pose starts out at zero
  const VectorXd zero = VectorXd::Zero(N);
  EXPECT_TRUE(CompareMatrices(position(), zero));
  EXPECT_TRUE(CompareMatrices(torque(), zero));

  // Check that we can set a different initial position.
  const VectorXd q0 = VectorXd::LinSpaced(N, 0.1, 0.2);
  dut_.set_initial_position(&context_, q0);
  EXPECT_TRUE(CompareMatrices(position(), q0));
  EXPECT_TRUE(CompareMatrices(torque(), zero));

  // Check that a real command trumps the initial position.
  // First, try with empty torques.
  const VectorXd q1 = VectorXd::LinSpaced(N, 0.3, 0.4);
  lcmt_iiwa_command command{};
  command.utime = 1;
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

// This test can be removed when the state output port disappears.
TEST_P(IiwaCommandReceiverTest, DeprecatedVelocityTest) {
  const VectorXd zero = VectorXd::Zero(N);

  // First message with position = q1.
  const VectorXd q1 = VectorXd::LinSpaced(N, 0.1, 0.2);
  lcmt_iiwa_command command{};
  command.utime = 1;
  command.num_joints = N;
  command.joint_position = {q1.data(), q1.data() + q1.size()};
  SetInput(command);
  EXPECT_TRUE(CompareMatrices(state().head(N), q1));
  EXPECT_TRUE(CompareMatrices(state().tail(N), q1 * 200, 1e-10));

  // Simulate the timed event.
  // TODO(jwnimmer-tri) This systems framework API is not very ergonomic.
  auto update = dut_.AllocateDiscreteVariables();
  update->SetFrom(context_.get_discrete_state());
  auto events = dut_.GetPeriodicEvents();
  ASSERT_EQ(events.size(), 1);
  const auto& event_data = events.begin()->first;
  const auto& event_list = events.begin()->second;
  EXPECT_EQ(event_data.offset_sec(), 0.0);
  ASSERT_EQ(event_list.size(), 1);
  const auto& event = *event_list.front();
  ASSERT_TRUE(event.is_discrete_update());
  const auto& discrete_event =
      dynamic_cast<const systems::DiscreteUpdateEvent<double>&>(event);
  discrete_event.handle(context_, update.get());
  context_.get_mutable_discrete_state().SetFrom(*update);

  // Second message with position = q1.
  const VectorXd q2 = VectorXd::LinSpaced(N, 0.3, 0.4);
  command.utime = 2;
  command.joint_position = {q2.data(), q2.data() + q2.size()};
  SetInput(command);
  EXPECT_TRUE(CompareMatrices(state().head(N), q2));
  EXPECT_TRUE(CompareMatrices(state().tail(N), (q2 - q1) * 200, 1e-10));
}

INSTANTIATE_TEST_CASE_P(
    EachInput, IiwaCommandReceiverTest,
    ::testing::Values(0, 1));

}  // namespace
}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
