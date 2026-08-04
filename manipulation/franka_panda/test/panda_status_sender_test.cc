#include "drake/manipulation/franka_panda/panda_status_sender.h"

#include <memory>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

namespace drake {
namespace manipulation {
namespace franka_panda {

using drake::lcmt_panda_status;
using drake::systems::Context;
using drake::systems::InputPort;
using Eigen::VectorXd;
constexpr int N = kPandaArmNumJoints;

class PandaStatusSenderTest : public testing::Test {
 public:
  PandaStatusSenderTest()
      : dut_(),
        context_ptr_(dut_.CreateDefaultContext()),
        context_(*context_ptr_) {}

  const lcmt_panda_status& output() const {
    const lcmt_panda_status& result =
        dut_.get_output_port().Eval<lcmt_panda_status>(context_);
    DRAKE_DEMAND(result.num_joints == N);
    DRAKE_DEMAND(result.joint_position.size() == N);
    DRAKE_DEMAND(result.joint_position_desired.size() == N);
    DRAKE_DEMAND(result.joint_velocity.size() == N);
    DRAKE_DEMAND(result.joint_velocity_desired.size() == N);
    DRAKE_DEMAND(result.joint_acceleration_desired.size() == N);
    DRAKE_DEMAND(result.joint_torque.size() == N);
    DRAKE_DEMAND(result.joint_torque_desired.size() == N);
    DRAKE_DEMAND(result.joint_torque_external.size() == N);
    return result;
  }

  static std::vector<double> as_vector(const Eigen::VectorXd& v) {
    return {v.data(), v.data() + v.size()};
  }

  void Fix(const InputPort<double>& port, const Eigen::VectorXd& v) {
    port.FixValue(&context_, v);
  }

 protected:
  PandaStatusSender dut_;
  std::unique_ptr<Context<double>> context_ptr_;
  Context<double>& context_;
};

TEST_F(PandaStatusSenderTest, AcceptanceTest) {
  const VectorXd q0_commanded = VectorXd::LinSpaced(N, 0.1, 0.2);
  const VectorXd q0_measured = VectorXd::LinSpaced(N, 0.11, 0.22);
  const VectorXd v0_commanded = VectorXd::LinSpaced(N, 0.2, 0.3);
  const VectorXd v0_measured = VectorXd::LinSpaced(N, 0.22, 0.33);
  const VectorXd a0_commanded = VectorXd::LinSpaced(N, 0.3, 0.4);
  const VectorXd t0_commanded = VectorXd::LinSpaced(N, 0.4, 0.5);
  const VectorXd t0_measured = VectorXd::LinSpaced(N, 0.44, 0.55);
  const VectorXd t0_external = VectorXd::LinSpaced(N, 0.6, 0.7);
  const std::vector<double> zeros(N, 0.0);

  // Fix only the required inputs ...
  Fix(dut_.get_position_measured_input_port(), q0_measured);
  Fix(dut_.get_torque_commanded_input_port(), t0_commanded);
  // ... so that some outputs have passthrough values ...
  EXPECT_EQ(output().joint_position, as_vector(q0_measured));
  EXPECT_EQ(output().joint_torque_desired, as_vector(t0_commanded));
  // ... and some outputs have default values.
  EXPECT_EQ(output().joint_position_desired, zeros);
  EXPECT_EQ(output().joint_velocity_desired, zeros);
  EXPECT_EQ(output().joint_velocity, zeros);
  EXPECT_EQ(output().joint_acceleration_desired, zeros);
  EXPECT_EQ(output().joint_torque, as_vector(t0_commanded));
  EXPECT_EQ(output().joint_torque_external, zeros);

  // Fix all of the inputs ...
  Fix(dut_.get_position_commanded_input_port(), q0_commanded);
  Fix(dut_.get_velocity_commanded_input_port(), v0_commanded);
  Fix(dut_.get_velocity_measured_input_port(), v0_measured);
  Fix(dut_.get_acceleration_commanded_input_port(), a0_commanded);
  Fix(dut_.get_torque_measured_input_port(), t0_measured);
  Fix(dut_.get_torque_external_input_port(), t0_external);
  // ... so all outputs have values.
  EXPECT_EQ(output().joint_position_desired, as_vector(q0_commanded));
  EXPECT_EQ(output().joint_position, as_vector(q0_measured));
  EXPECT_EQ(output().joint_velocity_desired, as_vector(v0_commanded));
  EXPECT_EQ(output().joint_velocity, as_vector(v0_measured));
  EXPECT_EQ(output().joint_torque_desired, as_vector(t0_commanded));
  EXPECT_EQ(output().joint_torque, as_vector(t0_measured));
  EXPECT_EQ(output().joint_torque_external, as_vector(t0_external));
}

}  // namespace franka_panda
}  // namespace manipulation
}  // namespace drake
