#include "drake/manipulation/kinova_jaco/jaco_status_receiver.h"

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

class JacoStatusReceiverTestBase : public testing::Test {
 public:
  JacoStatusReceiverTestBase(int num_joints, int num_fingers)
      : dut_(num_joints, num_fingers),
        context_ptr_(dut_.CreateDefaultContext()),
        context_(*context_ptr_),
        fixed_input_(
            dut_.get_input_port().FixValue(&context_, lcmt_jaco_status{})) {}

  // Test cases should call this to set the DUT's input value.
  void SetInput() {
    fixed_input_.GetMutableData()
        ->template get_mutable_value<lcmt_jaco_status>() = status_;
  }

  void Copy(const Eigen::VectorXd& from, std::vector<double>* to) {
    *to = {from.data(), from.data() + from.size()};
  }

 protected:
  JacoStatusReceiver dut_;
  std::unique_ptr<systems::Context<double>> context_ptr_;
  systems::Context<double>& context_;
  systems::FixedInputPortValue& fixed_input_;
  lcmt_jaco_status status_{};
};

class JacoStatusReceiverTest : public JacoStatusReceiverTestBase {
 public:
  JacoStatusReceiverTest()
      : JacoStatusReceiverTestBase(kJacoDefaultArmNumJoints,
                                   kJacoDefaultArmNumFingers) {}
};

class JacoStatusReceiverNoFingersTest : public JacoStatusReceiverTestBase {
 public:
  JacoStatusReceiverNoFingersTest()
      : JacoStatusReceiverTestBase(kJacoDefaultArmNumJoints, 0) {}
};

TEST_F(JacoStatusReceiverTest, ZeroOutputTest) {
  // Confirm that output is zero for uninitialized lcm input.
  const int num_output_ports = dut_.num_output_ports();
  for (int i = 0; i < num_output_ports; ++i) {
    const systems::LeafSystem<double>& leaf = dut_;
    const auto& port = leaf.get_output_port(i);
    EXPECT_TRUE(
        CompareMatrices(port.Eval(context_), VectorXd::Zero(port.size())));
  }
}

TEST_F(JacoStatusReceiverTest, AcceptanceTest) {
  const int utime = 1661199485;
  const VectorXd q0 = VectorXd::LinSpaced(N, 0.2, 0.3);
  const VectorXd v0 = VectorXd::LinSpaced(N, 0.3, 0.4);
  const VectorXd f_q0 = VectorXd::LinSpaced(N_F, 1.2, 1.3);
  const VectorXd f_v0 = VectorXd::LinSpaced(N_F, 1.3, 1.4);
  const VectorXd t0 = VectorXd::LinSpaced(N, 0.4, 0.5);
  const VectorXd t_ext0 = VectorXd::LinSpaced(N, 0.5, 0.6);
  const VectorXd current0 = VectorXd::LinSpaced(N, 0.6, 0.7);
  const VectorXd f_t0 = VectorXd::LinSpaced(N_F, 1.4, 1.5);
  const VectorXd f_t_ext0 = VectorXd::LinSpaced(N_F, 1.5, 1.6);
  const VectorXd f_current0 = VectorXd::LinSpaced(N_F, 1.6, 1.7);

  status_.utime = utime;
  status_.num_joints = N;
  status_.num_fingers = N_F;
  Copy(q0, &status_.joint_position);
  Copy(v0, &status_.joint_velocity);
  Copy(f_q0, &status_.finger_position);
  Copy(f_v0, &status_.finger_velocity);
  Copy(t0, &status_.joint_torque);
  Copy(t_ext0, &status_.joint_torque_external);
  Copy(current0, &status_.joint_current);
  Copy(f_t0, &status_.finger_torque);
  Copy(f_t_ext0, &status_.finger_torque_external);
  Copy(f_current0, &status_.finger_current);

  SetInput();

  VectorXd position_expected(N + N_F);
  position_expected.head(N) = q0;
  position_expected.tail(N_F) = f_q0 * kFingerSdkToUrdf;

  VectorXd velocity_expected(N + N_F);
  velocity_expected.head(N) = v0;
  velocity_expected.tail(N_F) = f_v0 * kFingerSdkToUrdf;

  EXPECT_TRUE(
      CompareMatrices(dut_.get_time_measured_output_port().Eval(context_),
                      Vector1d(utime) / 1e6));
  EXPECT_TRUE(
      CompareMatrices(dut_.get_position_measured_output_port().Eval(context_),
                      position_expected));
  EXPECT_TRUE(
      CompareMatrices(dut_.get_velocity_measured_output_port().Eval(context_),
                      velocity_expected));
  EXPECT_TRUE(CompareMatrices(
      dut_.get_torque_measured_output_port().Eval(context_).head(N), t0));
  EXPECT_TRUE(CompareMatrices(
      dut_.get_torque_measured_output_port().Eval(context_).tail(N_F), f_t0));
  EXPECT_TRUE(CompareMatrices(
      dut_.get_torque_external_output_port().Eval(context_).head(N), t_ext0));
  EXPECT_TRUE(CompareMatrices(
      dut_.get_torque_external_output_port().Eval(context_).tail(N_F),
      f_t_ext0));
  EXPECT_TRUE(CompareMatrices(
      dut_.get_current_output_port().Eval(context_).head(N), current0));
  EXPECT_TRUE(CompareMatrices(
      dut_.get_current_output_port().Eval(context_).tail(N_F), f_current0));
}

TEST_F(JacoStatusReceiverNoFingersTest, AcceptanceTestNoFingers) {
  const VectorXd q0 = VectorXd::LinSpaced(N, 0.2, 0.3);
  const VectorXd v0 = VectorXd::LinSpaced(N, 0.3, 0.4);
  const VectorXd t0 = VectorXd::LinSpaced(N, 0.4, 0.5);
  const VectorXd t_ext0 = VectorXd::LinSpaced(N, 0.5, 0.6);
  const VectorXd current0 = VectorXd::LinSpaced(N, 0.6, 0.7);

  status_.utime = 1;
  status_.num_joints = N;
  status_.num_fingers = 0;
  Copy(q0, &status_.joint_position);
  Copy(v0, &status_.joint_velocity);
  Copy(t0, &status_.joint_torque);
  Copy(t_ext0, &status_.joint_torque_external);
  Copy(current0, &status_.joint_current);

  SetInput();

  EXPECT_TRUE(CompareMatrices(
      dut_.get_position_measured_output_port().Eval(context_), q0));
  EXPECT_TRUE(CompareMatrices(
      dut_.get_velocity_measured_output_port().Eval(context_), v0));
  EXPECT_TRUE(CompareMatrices(
      dut_.get_torque_measured_output_port().Eval(context_), t0));
  EXPECT_TRUE(CompareMatrices(
      dut_.get_torque_external_output_port().Eval(context_), t_ext0));
  EXPECT_TRUE(
      CompareMatrices(dut_.get_current_output_port().Eval(context_), current0));
}

}  // namespace
}  // namespace kinova_jaco
}  // namespace manipulation
}  // namespace drake
