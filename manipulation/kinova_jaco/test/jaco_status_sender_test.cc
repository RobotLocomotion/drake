#include "drake/manipulation/kinova_jaco/jaco_status_sender.h"

#include <memory>
#include <vector>

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

class JacoStatusSenderTestBase : public testing::Test {
 public:
  JacoStatusSenderTestBase(int num_joints, int num_fingers)
      : dut_(num_joints, num_fingers),
        context_ptr_(dut_.CreateDefaultContext()),
        context_(*context_ptr_) {}

  const lcmt_jaco_status& output() const {
    return dut_.get_output_port().Eval<lcmt_jaco_status>(context_);
  }

  void Fix(const systems::InputPort<double>& port, const Eigen::VectorXd& v) {
    port.FixValue(&context_, v);
  }

 protected:
  JacoStatusSender dut_;
  std::unique_ptr<systems::Context<double>> context_ptr_;
  systems::Context<double>& context_;
};

class JacoStatusSenderTest : public JacoStatusSenderTestBase {
 public:
  JacoStatusSenderTest()
      : JacoStatusSenderTestBase(kJacoDefaultArmNumJoints,
                                 kJacoDefaultArmNumFingers) {}
};

class JacoStatusSenderNoFingersTest : public JacoStatusSenderTestBase {
 public:
  JacoStatusSenderNoFingersTest()
      : JacoStatusSenderTestBase(kJacoDefaultArmNumJoints, 0) {}
};

const std::vector<double> ToStdVec(const Eigen::VectorXd& in) {
  return std::vector<double>{in.data(), in.data() + in.size()};
}

TEST_F(JacoStatusSenderTest, AcceptanceTestWithFingers) {
  const VectorXd q0 = VectorXd::LinSpaced(N + N_F, 0.2, 0.3);
  const VectorXd v0 = VectorXd::LinSpaced(N + N_F, 0.3, 0.4);
  const VectorXd zero = VectorXd::Zero(N);
  const VectorXd finger_zero = VectorXd::Zero(N_F);

  dut_.get_position_input_port().FixValue(&context_, q0);
  dut_.get_velocity_input_port().FixValue(&context_, v0);

  EXPECT_EQ(output().utime, 0);
  EXPECT_EQ(output().num_joints, kJacoDefaultArmNumJoints);
  EXPECT_EQ(output().joint_position, ToStdVec(q0.head(N)));
  EXPECT_EQ(output().joint_velocity, ToStdVec(v0.head(N) / 2));
  EXPECT_EQ(output().num_fingers, kJacoDefaultArmNumFingers);
  EXPECT_EQ(output().finger_position,
            ToStdVec(q0.tail(N_F) * kFingerUrdfToSdk));
  EXPECT_EQ(output().finger_velocity,
            ToStdVec(v0.tail(N_F) * kFingerUrdfToSdk));
  EXPECT_EQ(output().joint_torque, ToStdVec(zero));
  EXPECT_EQ(output().joint_torque_external, ToStdVec(zero));
  EXPECT_EQ(output().joint_current, ToStdVec(zero));
  EXPECT_EQ(output().finger_torque, ToStdVec(finger_zero));
  EXPECT_EQ(output().finger_torque_external, ToStdVec(finger_zero));
  EXPECT_EQ(output().finger_current, ToStdVec(finger_zero));

  const VectorXd time_measured = Vector1d(1.2);
  const VectorXd t1 = VectorXd::LinSpaced(N + N_F, 0.4, 0.5);
  const VectorXd t_ext1 = VectorXd::LinSpaced(N + N_F, 0.5, 0.6);
  const VectorXd current1 = VectorXd::LinSpaced(N + N_F, 0.6, 0.7);

  dut_.get_time_measured_input_port().FixValue(&context_, time_measured);
  dut_.get_torque_input_port().FixValue(&context_, t1);
  dut_.get_torque_external_input_port().FixValue(&context_, t_ext1);
  dut_.get_current_input_port().FixValue(&context_, current1);

  EXPECT_EQ(output().utime, time_measured[0] * 1e6);
  EXPECT_EQ(output().joint_torque, ToStdVec(t1.head(N)));
  EXPECT_EQ(output().joint_torque_external, ToStdVec(t_ext1.head(N)));
  EXPECT_EQ(output().joint_current, ToStdVec(current1.head(N)));
  EXPECT_EQ(output().finger_torque, ToStdVec(t1.tail(N_F)));
  EXPECT_EQ(output().finger_torque_external, ToStdVec(t_ext1.tail(N_F)));
  EXPECT_EQ(output().finger_current, ToStdVec(current1.tail(N_F)));
}

TEST_F(JacoStatusSenderNoFingersTest, AcceptanceNoFingers) {
  const VectorXd q0 = VectorXd::LinSpaced(N, 0.2, 0.3);
  const VectorXd v0 = VectorXd::LinSpaced(N, 0.3, 0.4);

  dut_.get_position_input_port().FixValue(&context_, q0);
  dut_.get_velocity_input_port().FixValue(&context_, v0);

  EXPECT_EQ(output().num_joints, kJacoDefaultArmNumJoints);
  EXPECT_EQ(output().joint_position, ToStdVec(q0));
  EXPECT_EQ(output().joint_velocity, ToStdVec(v0 / 2));
  EXPECT_EQ(output().num_fingers, 0);
}

}  // namespace
}  // namespace kinova_jaco
}  // namespace manipulation
}  // namespace drake
