#include "drake/manipulation/kinova_jaco/jaco_status_sender.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace manipulation {
namespace kinova_jaco {
namespace {

using Eigen::VectorXd;

class JacoStatusSenderTest : public testing::Test {
 public:
  JacoStatusSenderTest()
      : dut_(),
        context_ptr_(dut_.CreateDefaultContext()),
        context_(*context_ptr_) {}

  const lcmt_jaco_status& output() const {
    const lcmt_jaco_status& result =
        dut_.get_output_port().Eval<lcmt_jaco_status>(context_);
    DRAKE_DEMAND(result.num_joints == kJacoDefaultArmNumJoints);
    DRAKE_DEMAND(result.joint_position.size() == kJacoDefaultArmNumJoints);
    DRAKE_DEMAND(result.joint_velocity.size() == kJacoDefaultArmNumJoints);
    DRAKE_DEMAND(result.joint_torque.size() == kJacoDefaultArmNumJoints);
    DRAKE_DEMAND(
        result.joint_torque_external.size() == kJacoDefaultArmNumJoints);
    DRAKE_DEMAND(result.num_fingers == kJacoDefaultArmNumFingers);
    DRAKE_DEMAND(result.finger_position.size() == kJacoDefaultArmNumFingers);
    DRAKE_DEMAND(result.finger_velocity.size() == kJacoDefaultArmNumFingers);
    DRAKE_DEMAND(result.finger_torque.size() == kJacoDefaultArmNumFingers);
    DRAKE_DEMAND(
        result.finger_torque_external.size() == kJacoDefaultArmNumFingers);
    return result;
  }

  static std::vector<double> as_vector(const Eigen::VectorXd& v) {
    return {v.data(), v.data() + v.size()};
  }

  void Fix(const systems::InputPort<double>& port, const Eigen::VectorXd& v) {
    port.FixValue(&context_, v);
  }

 protected:
  JacoStatusSender dut_;
  std::unique_ptr<systems::Context<double>> context_ptr_;
  systems::Context<double>& context_;
};

TEST_F(JacoStatusSenderTest, AcceptanceTest) {
  constexpr int total_dof =
      kJacoDefaultArmNumJoints + kJacoDefaultArmNumFingers;

  const VectorXd state = VectorXd::LinSpaced(total_dof * 2, 0.0, 1.0);
  const VectorXd torque = VectorXd::LinSpaced(total_dof, 2.0, 3.0);
  const VectorXd torque_external = VectorXd::LinSpaced(total_dof, 4.0, 5.0);
  const VectorXd current = VectorXd::LinSpaced(total_dof, 6.0, 7.0);

  // Fix only the required inputs ...
  Fix(dut_.get_state_input_port(), state);

  // ... so that some outputs have passthrough values ...
  EXPECT_EQ(output().joint_position,
            as_vector(state.head(kJacoDefaultArmNumJoints)));
  EXPECT_EQ(output().joint_velocity,
            as_vector(state.segment(total_dof, kJacoDefaultArmNumJoints) / 2));
  EXPECT_EQ(output().finger_position,
            as_vector(
                state.segment(kJacoDefaultArmNumJoints,
                              kJacoDefaultArmNumFingers) * kFingerUrdfToSdk));
  EXPECT_EQ(output().finger_velocity,
            as_vector(
                state.tail(kJacoDefaultArmNumFingers) * kFingerUrdfToSdk));
  // ... and some outputs have default values.
  EXPECT_EQ(output().joint_torque,
            std::vector<double>(kJacoDefaultArmNumJoints, 0.0));
  EXPECT_EQ(output().joint_torque_external,
            std::vector<double>(kJacoDefaultArmNumJoints, 0.0));
  EXPECT_EQ(output().joint_current,
            std::vector<double>(kJacoDefaultArmNumJoints, 0.0));
  EXPECT_EQ(output().finger_torque,
            std::vector<double>(kJacoDefaultArmNumFingers, 0.0));
  EXPECT_EQ(output().finger_torque_external,
            std::vector<double>(kJacoDefaultArmNumFingers, 0.0));
  EXPECT_EQ(output().finger_current,
            std::vector<double>(kJacoDefaultArmNumFingers, 0.0));

  // Fix all of the inputs ...
  Fix(dut_.get_torque_input_port(), torque);
  Fix(dut_.get_torque_external_input_port(), torque_external);
  Fix(dut_.get_current_input_port(), current);

  // ... so all ouputs have values.
  EXPECT_EQ(output().joint_torque,
            as_vector(torque.head(kJacoDefaultArmNumJoints)));
  EXPECT_EQ(output().joint_torque_external,
            as_vector(torque_external.head(kJacoDefaultArmNumJoints)));
  EXPECT_EQ(output().joint_current,
            as_vector(current.head(kJacoDefaultArmNumJoints)));
  EXPECT_EQ(output().finger_torque,
            as_vector(torque.tail(kJacoDefaultArmNumFingers)));
  EXPECT_EQ(output().finger_torque_external,
            as_vector(torque_external.tail(kJacoDefaultArmNumFingers)));
  EXPECT_EQ(output().finger_current,
            as_vector(current.tail(kJacoDefaultArmNumFingers)));
}

}  // namespace
}  // namespace kinova_jaco
}  // namespace manipulation
}  // namespace drake
