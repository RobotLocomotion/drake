#include "drake/manipulation/kuka_iiwa/iiwa_status_receiver.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {
namespace {

using Eigen::VectorXd;
constexpr int N = kIiwaArmNumJoints;

class IiwaStatusReceiverTest : public testing::Test {
 public:
  IiwaStatusReceiverTest()
      : dut_(),
        context_ptr_(dut_.CreateDefaultContext()),
        context_(*context_ptr_),
        fixed_input_(context_.FixInputPort(
            dut_.get_input_port().get_index(),
            Value<lcmt_iiwa_status>{})) {}

  // Some sugar to consolidate the deprecation suppression.
  const systems::OutputPort<double>& state_port() const {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    return dut_.get_state_output_port();
#pragma GCC diagnostic pop
  }

  void Copy(const Eigen::VectorXd& from, std::vector<double>* to) {
    *to = {from.data(), from.data() + from.size()};
  }

 protected:
  IiwaStatusReceiver dut_;
  std::unique_ptr<systems::Context<double>> context_ptr_;
  systems::Context<double>& context_;
  systems::FixedInputPortValue& fixed_input_;
  lcmt_iiwa_status status_{};
};

TEST_F(IiwaStatusReceiverTest, AcceptanceTest) {
  // Confirm that output is zero for uninitialized lcm input.
  const int num_output_ports = dut_.num_output_ports();
  EXPECT_EQ(num_output_ports, 7);
  for (int i = 0; i < num_output_ports; ++i) {
    const systems::LeafSystem<double>& leaf = dut_;
    const auto& port = leaf.get_output_port(i);
    const int size = (&port == &state_port()) ? (2 * N) : N;
    EXPECT_TRUE(CompareMatrices(
        port.Eval(context_),
        VectorXd::Zero(size)));
  }

  // Populate the status message with distinct values.
  const VectorXd position_commanded = VectorXd::LinSpaced(N, 0.0, 1.0);
  const VectorXd position_measured = VectorXd::LinSpaced(N, 2.0, 3.0);
  const VectorXd velocity_estimated = VectorXd::LinSpaced(N, 4.0, 5.0);
  const VectorXd torque_commanded = VectorXd::LinSpaced(N, 6.0, 7.0);
  const VectorXd torque_measured = VectorXd::LinSpaced(N, 8.0, 9.0);
  const VectorXd torque_external = VectorXd::LinSpaced(N, 10.0, 11.0);
  status_.utime = 1;
  status_.num_joints = N;
  Copy(position_commanded, &status_.joint_position_commanded);
  Copy(position_measured, &status_.joint_position_measured);
  Copy(velocity_estimated, &status_.joint_velocity_estimated);
  Copy(torque_commanded, &status_.joint_torque_commanded);
  Copy(torque_measured, &status_.joint_torque_measured);
  Copy(torque_external, &status_.joint_torque_external);
  // TODO(jwnimmer-tri) This systems framework API is not very ergonomic.
  fixed_input_.GetMutableData()->
      template get_mutable_value<lcmt_iiwa_status>() = status_;

  // Confirm that real message values are output correctly.
  EXPECT_TRUE(CompareMatrices(
      dut_.get_position_commanded_output_port().Eval(context_),
      position_commanded));
  EXPECT_TRUE(CompareMatrices(
      dut_.get_position_measured_output_port().Eval(context_),
      position_measured));
  EXPECT_TRUE(CompareMatrices(
      dut_.get_velocity_estimated_output_port().Eval(context_),
      velocity_estimated));
  EXPECT_TRUE(CompareMatrices(
      dut_.get_torque_commanded_output_port().Eval(context_),
      torque_commanded));
  EXPECT_TRUE(CompareMatrices(
      dut_.get_torque_measured_output_port().Eval(context_),
      torque_measured));
  EXPECT_TRUE(CompareMatrices(
      dut_.get_torque_external_output_port().Eval(context_),
      torque_external));
  EXPECT_TRUE(CompareMatrices(
      state_port().Eval(context_).head(N),
      position_measured));
  EXPECT_TRUE(CompareMatrices(
      state_port().Eval(context_).tail(N),
      velocity_estimated));
}

}  // namespace
}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
