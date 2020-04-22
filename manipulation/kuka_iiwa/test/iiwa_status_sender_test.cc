#include "drake/manipulation/kuka_iiwa/iiwa_status_sender.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {
namespace {

using Eigen::VectorXd;
constexpr int N = kIiwaArmNumJoints;

class IiwaStatusSenderTest : public testing::Test {
 public:
  IiwaStatusSenderTest()
      : dut_(),
        context_ptr_(dut_.CreateDefaultContext()),
        context_(*context_ptr_) {}

  const lcmt_iiwa_status& output() const {
    const lcmt_iiwa_status& result =
        dut_.get_output_port().Eval<lcmt_iiwa_status>(context_);
    DRAKE_DEMAND(result.num_joints == N);
    DRAKE_DEMAND(result.joint_position_commanded.size() == N);
    DRAKE_DEMAND(result.joint_position_measured.size() == N);
    DRAKE_DEMAND(result.joint_velocity_estimated.size() == N);
    DRAKE_DEMAND(result.joint_torque_commanded.size() == N);
    DRAKE_DEMAND(result.joint_torque_measured.size() == N);
    DRAKE_DEMAND(result.joint_torque_external.size() == N);
    return result;
  }

  static std::vector<double> as_vector(const Eigen::VectorXd& v) {
    return {v.data(), v.data() + v.size()};
  }

  void Fix(const systems::InputPort<double>& port, const Eigen::VectorXd& v) {
    port.FixValue(&context_, v);
  }

 protected:
  IiwaStatusSender dut_;
  std::unique_ptr<systems::Context<double>> context_ptr_;
  systems::Context<double>& context_;
};

TEST_F(IiwaStatusSenderTest, AcceptanceTest) {
  const VectorXd q0_commanded = VectorXd::LinSpaced(N, 0.1, 0.2);
  const VectorXd q0_measured = VectorXd::LinSpaced(N, 0.11, 0.22);
  const VectorXd v0_estimated = VectorXd::LinSpaced(N, 0.2, 0.3);
  const VectorXd t0_commanded = VectorXd::LinSpaced(N, 0.4, 0.5);
  const VectorXd t0_measured = VectorXd::LinSpaced(N, 0.44, 0.55);
  const VectorXd t0_external = VectorXd::LinSpaced(N, 0.6, 0.7);

  // Fix only the required inputs ...
  Fix(dut_.get_position_commanded_input_port(), q0_commanded);
  Fix(dut_.get_position_measured_input_port(), q0_measured);
  Fix(dut_.get_torque_commanded_input_port(), t0_commanded);
  // ... so that some outputs have passthrough values ...
  EXPECT_EQ(output().joint_position_commanded, as_vector(q0_commanded));
  EXPECT_EQ(output().joint_position_measured, as_vector(q0_measured));
  EXPECT_EQ(output().joint_torque_commanded, as_vector(t0_commanded));
  // ... and some outputs have default values.
  EXPECT_EQ(output().joint_velocity_estimated, std::vector<double>(N, 0.0));
  EXPECT_EQ(output().joint_torque_measured, as_vector(t0_commanded));
  EXPECT_EQ(output().joint_torque_external, std::vector<double>(N, 0.0));

  // Fix all of the inputs ...
  Fix(dut_.get_velocity_estimated_input_port(), v0_estimated);
  Fix(dut_.get_torque_measured_input_port(), t0_measured);
  Fix(dut_.get_torque_external_input_port(), t0_external);
  // ... so all outputs have values.
  EXPECT_EQ(output().joint_position_commanded, as_vector(q0_commanded));
  EXPECT_EQ(output().joint_position_measured, as_vector(q0_measured));
  EXPECT_EQ(output().joint_torque_commanded, as_vector(t0_commanded));
  EXPECT_EQ(output().joint_velocity_estimated, as_vector(v0_estimated));
  EXPECT_EQ(output().joint_torque_measured, as_vector(t0_measured));
  EXPECT_EQ(output().joint_torque_external, as_vector(t0_external));
}

}  // namespace
}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
