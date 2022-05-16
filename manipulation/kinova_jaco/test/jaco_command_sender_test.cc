#include "drake/manipulation/kinova_jaco/jaco_command_sender.h"

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

class JacoCommandSenderTestBase : public testing::Test {
 public:
  JacoCommandSenderTestBase(int num_joints, int num_fingers)
      : dut_(num_joints, num_fingers),
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

class JacoCommandSenderTest : public JacoCommandSenderTestBase {
 public:
  JacoCommandSenderTest()
      : JacoCommandSenderTestBase(
            kJacoDefaultArmNumJoints, kJacoDefaultArmNumFingers) {}
};

class JacoCommandSenderNoFingersTest : public JacoCommandSenderTestBase {
 public:
  JacoCommandSenderNoFingersTest()
      : JacoCommandSenderTestBase(
            kJacoDefaultArmNumJoints, 0) {}
};

const std::vector<double> ToStdVec(const Eigen::VectorXd& in) {
  return std::vector<double>{in.data(), in.data() + in.size()};
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

TEST_F(JacoCommandSenderTest, DeprecatedAcceptanceTest) {
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
#pragma GCC diagnostic pop

TEST_F(JacoCommandSenderTest, AcceptanceTestWithFingers) {
  const VectorXd q0 = VectorXd::LinSpaced(N + N_F, 0.2, 0.3);
  const VectorXd v0 = VectorXd::LinSpaced(N + N_F, 0.3, 0.4);

  dut_.get_position_input_port().FixValue(&context_, q0);
  dut_.get_velocity_input_port().FixValue(&context_, v0);

  EXPECT_EQ(output().num_joints, kJacoDefaultArmNumJoints);
  EXPECT_EQ(output().joint_position, ToStdVec(q0.head(N)));
  EXPECT_EQ(output().joint_velocity, ToStdVec(v0.head(N)));
  EXPECT_EQ(output().num_fingers, kJacoDefaultArmNumFingers);
  EXPECT_EQ(output().finger_position,
            ToStdVec(q0.tail(N_F) * kFingerUrdfToSdk));
  EXPECT_EQ(output().finger_velocity,
            ToStdVec(v0.tail(N_F) * kFingerUrdfToSdk));
}

TEST_F(JacoCommandSenderNoFingersTest, AcceptanceNoFingers) {
  const VectorXd q0 = VectorXd::LinSpaced(N, 0.2, 0.3);
  const VectorXd v0 = VectorXd::LinSpaced(N, 0.3, 0.4);

  dut_.get_position_input_port().FixValue(&context_, q0);
  dut_.get_velocity_input_port().FixValue(&context_, v0);

  EXPECT_EQ(output().num_joints, kJacoDefaultArmNumJoints);
  EXPECT_EQ(output().joint_position, ToStdVec(q0));
  EXPECT_EQ(output().joint_velocity, ToStdVec(v0));
}

}  // namespace
}  // namespace kinova_jaco
}  // namespace manipulation
}  // namespace drake
