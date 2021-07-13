#include "drake/manipulation/kuka_iiwa/iiwa_command_sender.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/limit_malloc.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {
namespace {

using Eigen::VectorXd;
using drake::test::LimitMalloc;

constexpr int N = kIiwaArmNumJoints;

class IiwaCommandSenderTest : public testing::Test {
 public:
  IiwaCommandSenderTest()
      : dut_(),
        context_ptr_(dut_.CreateDefaultContext()),
        context_(*context_ptr_) {}

  const lcmt_iiwa_command& output() const {
    return dut_.get_output_port().Eval<lcmt_iiwa_command>(context_);
  }

 protected:
  IiwaCommandSender dut_;
  std::unique_ptr<systems::Context<double>> context_ptr_;
  systems::Context<double>& context_;

  const VectorXd q0_{VectorXd::LinSpaced(N, 0.1, 0.2)};
  const std::vector<double> std_q0_{q0_.data(), q0_.data() + q0_.size()};

  const VectorXd t0_{VectorXd::LinSpaced(N, 0.3, 0.4)};
  const std::vector<double> std_t0_{t0_.data(), t0_.data() + t0_.size()};
};

TEST_F(IiwaCommandSenderTest, AcceptanceTest) {
  dut_.get_position_input_port().FixValue(&context_, q0_);
  EXPECT_EQ(output().num_joints, N);
  EXPECT_EQ(output().joint_position, std_q0_);
  EXPECT_EQ(output().num_torques, 0);

  dut_.get_torque_input_port().FixValue(&context_, t0_);
  EXPECT_EQ(output().num_joints, N);
  EXPECT_EQ(output().joint_position, std_q0_);
  EXPECT_EQ(output().num_torques, N);
  EXPECT_EQ(output().joint_torque, std_t0_);
}

// This class is likely to be used on the critical path for robot control, so
// we insist that it must not perform heap operations while in steady-state.
TEST_F(IiwaCommandSenderTest, MallocTest) {
  // Initialize and then invalidate the cached output.
  auto& q = dut_.get_position_input_port().FixValue(&context_, q0_);
  output();
  q.GetMutableVectorData<double>();

  // Recompute the output. No heap changes are allowed.
  {
    LimitMalloc guard;
    output();
  }

  // Add torques, re-initialize, and then invalidate the cached output.
  auto& tau = dut_.get_torque_input_port().FixValue(&context_, t0_);
  output();
  tau.GetMutableVectorData<double>();

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
