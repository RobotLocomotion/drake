#include "drake/manipulation/franka_panda/panda_status_receiver.h"

#include <memory>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/limit_malloc.h"

namespace drake {
namespace manipulation {
namespace franka_panda {

using drake::CompareMatrices;
using drake::lcmt_panda_status;
using drake::systems::Context;
using drake::systems::FixedInputPortValue;
using drake::systems::LeafSystem;
using drake::test::LimitMalloc;
using Eigen::VectorXd;

constexpr int N = kPandaArmNumJoints;

class PandaStatusReceiverTest : public testing::Test {
 public:
  PandaStatusReceiverTest()
      : dut_(),
        context_ptr_(dut_.CreateDefaultContext()),
        context_(*context_ptr_),
        fixed_input_(
            dut_.get_input_port().FixValue(&context_, lcmt_panda_status{})) {}

  void Copy(const Eigen::VectorXd& from, std::vector<double>* to) {
    *to = {from.data(), from.data() + from.size()};
  }

 protected:
  PandaStatusReceiver dut_;
  std::unique_ptr<Context<double>> context_ptr_;
  Context<double>& context_;
  FixedInputPortValue& fixed_input_;
  lcmt_panda_status status_{};
};

TEST_F(PandaStatusReceiverTest, AcceptanceTest) {
  // Confirm that output is zero for uninitialized lcm input.
  const int num_output_ports = dut_.num_output_ports();
  EXPECT_EQ(num_output_ports, 8);
  for (int i = 0; i < num_output_ports; ++i) {
    const LeafSystem<double>& leaf = dut_;
    const auto& port = leaf.get_output_port(i);
    EXPECT_TRUE(CompareMatrices(port.Eval(context_), VectorXd::Zero(N)));
  }

  // Populate the status message with distinct values.
  const VectorXd position_commanded = VectorXd::LinSpaced(N, 0.0, 1.0);
  const VectorXd position_measured = VectorXd::LinSpaced(N, 1.0, 2.0);
  const VectorXd velocity_commanded = VectorXd::LinSpaced(N, 2.0, 3.0);
  const VectorXd velocity_measured = VectorXd::LinSpaced(N, 3.0, 4.0);
  const VectorXd acceleration_commanded = VectorXd::LinSpaced(N, 4.0, 5.0);
  const VectorXd torque_commanded = VectorXd::LinSpaced(N, 6.0, 7.0);
  const VectorXd torque_measured = VectorXd::LinSpaced(N, 8.0, 9.0);
  const VectorXd torque_external = VectorXd::LinSpaced(N, 10.0, 11.0);
  status_.utime = 1;
  status_.num_joints = N;
  Copy(position_commanded, &status_.joint_position_desired);
  Copy(position_measured, &status_.joint_position);
  Copy(velocity_commanded, &status_.joint_velocity_desired);
  Copy(velocity_measured, &status_.joint_velocity);
  Copy(acceleration_commanded, &status_.joint_acceleration_desired);
  Copy(torque_commanded, &status_.joint_torque_desired);
  Copy(torque_measured, &status_.joint_torque);
  Copy(torque_external, &status_.joint_torque_external);
  // TODO(jwnimmer-tri) This systems framework API is not very ergonomic.
  fixed_input_.GetMutableData()
      ->template get_mutable_value<lcmt_panda_status>() = status_;

  // Confirm that real message values are output correctly.
  EXPECT_TRUE(
      CompareMatrices(dut_.get_position_commanded_output_port().Eval(context_),
                      position_commanded));
  EXPECT_TRUE(
      CompareMatrices(dut_.get_position_measured_output_port().Eval(context_),
                      position_measured));
  EXPECT_TRUE(
      CompareMatrices(dut_.get_velocity_commanded_output_port().Eval(context_),
                      velocity_commanded));
  EXPECT_TRUE(
      CompareMatrices(dut_.get_velocity_measured_output_port().Eval(context_),
                      velocity_measured));
  EXPECT_TRUE(CompareMatrices(
      dut_.get_acceleration_commanded_output_port().Eval(context_),
      acceleration_commanded));
  EXPECT_TRUE(
      CompareMatrices(dut_.get_torque_commanded_output_port().Eval(context_),
                      torque_commanded));
  EXPECT_TRUE(CompareMatrices(
      dut_.get_torque_measured_output_port().Eval(context_), torque_measured));
  EXPECT_TRUE(CompareMatrices(
      dut_.get_torque_external_output_port().Eval(context_), torque_external));
}

// This class is likely to be used on the critical path for robot control, so
// we insist that it must not perform heap operations while in steady-state.
TEST_F(PandaStatusReceiverTest, MallocTest) {
  // Set as input a non-trivial message.
  status_.utime = 1;
  status_.num_joints = N;
  status_.joint_position_desired.resize(N);
  status_.joint_position.resize(N);
  status_.joint_velocity_desired.resize(N);
  status_.joint_velocity.resize(N);
  status_.joint_acceleration_desired.resize(N);
  status_.joint_torque_desired.resize(N);
  status_.joint_torque.resize(N);
  status_.joint_torque_external.resize(N);
  // TODO(jwnimmer-tri) This systems framework API is not very ergonomic.
  fixed_input_.GetMutableData()
      ->template get_mutable_value<lcmt_panda_status>() = status_;

  // Compute the output. No heap changes are allowed.
  {
    LimitMalloc guard;
    const LeafSystem<double>& leaf = dut_;
    const int num_output_ports = leaf.num_output_ports();
    for (int i = 0; i < num_output_ports; ++i) {
      const auto& port = leaf.get_output_port(i);
      port.Eval(context_);
    }
  }
}

}  // namespace franka_panda
}  // namespace manipulation
}  // namespace drake
