#include "drake/manipulation/kinova_jaco/jaco_command_sender.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace manipulation {
namespace kinova_jaco {
namespace {

using Eigen::VectorXd;

class JacoCommandSenderTest : public testing::Test {
 public:
  JacoCommandSenderTest()
      : dut_(),
        context_ptr_(dut_.CreateDefaultContext()),
        context_(*context_ptr_) {}

  const lcmt_jaco_command& output() const {
    return dut_.get_output_port().Eval<lcmt_jaco_command>(context_);
  }

 protected:
  JacoCommandSender dut_;
  std::unique_ptr<systems::Context<double>> context_ptr_;
  systems::Context<double>& context_;
};

const std::vector<double> ToStdVec(const Eigen::VectorXd& in) {
  return std::vector<double>{in.data(), in.data() + in.size()};
}

TEST_F(JacoCommandSenderTest, AcceptanceTest) {
  constexpr int total_dof =
      kJacoDefaultArmNumJoints + kJacoDefaultArmNumFingers;
  const VectorXd state =
      VectorXd::LinSpaced(total_dof * 2, 0.3, 0.4);

  dut_.get_input_port().FixValue(&context_, state);
  EXPECT_EQ(output().num_joints, kJacoDefaultArmNumJoints);
  EXPECT_EQ(output().joint_position,
            ToStdVec(state.head(kJacoDefaultArmNumJoints)));
  EXPECT_EQ(output().joint_velocity,
            ToStdVec(state.segment(total_dof, kJacoDefaultArmNumJoints)));
  EXPECT_EQ(output().num_fingers, kJacoDefaultArmNumFingers);
  EXPECT_EQ(output().finger_position, ToStdVec(state.segment(
      kJacoDefaultArmNumJoints, kJacoDefaultArmNumFingers)
                                               * kFingerUrdfToSdk));
  EXPECT_EQ(output().finger_velocity,
            ToStdVec(state.tail(kJacoDefaultArmNumFingers)
                     * kFingerUrdfToSdk));
}

}  // namespace
}  // namespace kinova_jaco
}  // namespace manipulation
}  // namespace drake
