#include "drake/manipulation/kuka_iiwa/iiwa_command_sender.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/limit_malloc.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {
namespace {

using drake::test::LimitMalloc;
using Eigen::VectorXd;

constexpr int N = kIiwaArmNumJoints;

class IiwaCommandSenderTest : public testing::Test {
 public:
  IiwaCommandSenderTest() {}

 protected:
  template <typename... Args>
  void MakeDut(Args&&... args) {
    dut_ = std::make_unique<IiwaCommandSender>(
          kIiwaArmNumJoints, std::forward<Args>(args)...);
    context_ptr_ = dut().CreateDefaultContext();
  }

  IiwaCommandSender& dut() { return *dut_; }
  systems::Context<double>& context() { return *context_ptr_; }
  const lcmt_iiwa_command& output() {
    return dut().get_output_port().Eval<lcmt_iiwa_command>(context());
  }

  std::unique_ptr<IiwaCommandSender> dut_;
  std::unique_ptr<systems::Context<double>> context_ptr_;

  const Vector1d time_{Vector1d(1.2)};

  const VectorXd q0_{VectorXd::LinSpaced(N, 0.1, 0.2)};
  const std::vector<double> std_q0_{q0_.data(), q0_.data() + q0_.size()};

  const VectorXd t0_{VectorXd::LinSpaced(N, 0.3, 0.4)};
  const std::vector<double> std_t0_{t0_.data(), t0_.data() + t0_.size()};
};

TEST_F(IiwaCommandSenderTest, AcceptanceTest) {
  // Default is position and torque.
  MakeDut();
  // Position is required.
  EXPECT_THROW(output(), std::logic_error);

  dut().get_position_input_port().FixValue(&context(), q0_);
  EXPECT_EQ(output().utime, 0);
  EXPECT_EQ(output().num_joints, N);
  EXPECT_EQ(output().joint_position, std_q0_);
  EXPECT_EQ(output().num_torques, 0);

  // Time is optional.
  dut().get_time_input_port().FixValue(&context(), time_);
  // Torque is optional.
  dut().get_torque_input_port().FixValue(&context(), t0_);
  EXPECT_EQ(output().utime, time_[0] * 1e6);
  EXPECT_EQ(output().num_joints, N);
  EXPECT_EQ(output().joint_position, std_q0_);
  EXPECT_EQ(output().num_torques, N);
  EXPECT_EQ(output().joint_torque, std_t0_);
}

TEST_F(IiwaCommandSenderTest, PositionOnlyTest) {
  MakeDut(IiwaControlMode::kPosition);
  // Should not have torque input port.
  EXPECT_THROW(dut().get_torque_input_port(), std::runtime_error);
  // Position is required.
  EXPECT_THROW(output(), std::logic_error);

  dut().get_position_input_port().FixValue(&context(), q0_);
  EXPECT_EQ(output().utime, 0);
  EXPECT_EQ(output().num_joints, N);
  EXPECT_EQ(output().joint_position, std_q0_);
  EXPECT_EQ(output().num_torques, 0);
}

TEST_F(IiwaCommandSenderTest, TorqueOnlyTest) {
  MakeDut(IiwaControlMode::kTorque);
  // Should not have position input port.
  EXPECT_THROW(dut().get_position_input_port(), std::runtime_error);
  // Torque is required.
  EXPECT_THROW(output(), std::logic_error);

  dut().get_torque_input_port().FixValue(&context(), t0_);
  EXPECT_EQ(output().utime, 0);
  EXPECT_EQ(output().num_joints, 0);
  EXPECT_EQ(output().num_torques, N);
  EXPECT_EQ(output().joint_torque, std_t0_);
}

// This class is likely to be used on the critical path for robot control, so
// we insist that it must not perform heap operations while in steady-state.
TEST_F(IiwaCommandSenderTest, MallocTest) {
  MakeDut();
  // Initialize and then invalidate the cached output.
  auto& q = dut().get_position_input_port().FixValue(&context(), q0_);
  output();
  q.GetMutableVectorData<double>();

  // Recompute the output. No heap changes are allowed.
  {
    LimitMalloc guard;
    output();
  }

  // Add torques, re-initialize, and then invalidate the cached output.
  auto& tau = dut().get_torque_input_port().FixValue(&context(), t0_);
  output();
  tau.GetMutableVectorData<double>();

  // Recompute the output. No heap changes are allowed.
  {
    LimitMalloc guard;
    output();
  }

  // Add time, re-initialize, and then invalidate the cached output.
  auto& utime = dut().get_time_input_port().FixValue(&context(), time_);
  output();
  utime.GetMutableVectorData<double>();

  // Recompute the output. No heap changes are allowed.
  {
    LimitMalloc guard;
    output();
  }
}

}  // namespace
}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
