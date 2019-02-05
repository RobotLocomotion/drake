#include "drake/manipulation/kuka_iiwa/iiwa_command_sender.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {
namespace {

using Eigen::VectorXd;
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
};

TEST_F(IiwaCommandSenderTest, AcceptanceTest) {
  const VectorXd q0 = VectorXd::LinSpaced(N, 0.1, 0.2);
  const std::vector<double> std_q0 = {q0.data(), q0.data() + q0.size()};
  context_.FixInputPort(dut_.get_position_input_port().get_index(), q0);
  EXPECT_EQ(output().num_joints, N);
  EXPECT_EQ(output().joint_position, std_q0);
  EXPECT_EQ(output().num_torques, 0);

  const VectorXd t0 = VectorXd::LinSpaced(N, 0.3, 0.4);
  const std::vector<double> std_t0 = {t0.data(), t0.data() + t0.size()};
  context_.FixInputPort(dut_.get_torque_input_port().get_index(), t0);
  EXPECT_EQ(output().num_joints, N);
  EXPECT_EQ(output().joint_position, std_q0);
  EXPECT_EQ(output().num_torques, N);
  EXPECT_EQ(output().joint_torque, std_t0);
}

}  // namespace
}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
