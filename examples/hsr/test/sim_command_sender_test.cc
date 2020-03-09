#include "drake/examples/hsr/sim_command_sender.h"

#include <gtest/gtest.h>

#include "drake/examples/hsr/test_utilities/sim_lcm_test_configs.h"
#include "drake/lcmt_hsr_sim_command.hpp"

namespace drake {
namespace examples {
namespace hsr {
namespace {

const char* kRobotModelUrdf =
    "drake/examples/hsr/models/urdfs/hsrb4s_fix_free_joints.urdf";
// This joint name must exist in the given urdf model and it must be an
// actuated joint.
const char* kTestJointName = "arm_roll_joint";

// Given a input state vector, test the message should be assembled properly.
// An arbitrary joint is used to confirm the values are assigned correctly.
GTEST_TEST(SimCommandSenderTest, MessageAssmeblyTest) {
  SimLcmTestConfig test_config{kRobotModelUrdf};

  const auto& robot_plant = test_config.robot_plant();
  SimCommandSender dut(&robot_plant);

  std::unique_ptr<systems::Context<double>> context =
      dut.CreateDefaultContext();

  const Eigen::VectorXd& input_value = test_config.desired_state_vector();
  dut.get_desired_state_input_port().FixValue(context.get(), input_value);

  const lcmt_hsr_sim_command& output =
      dut.get_sim_command_output_port().Eval<lcmt_hsr_sim_command>(*context);

  for (int i = 0; i < output.num_joints; ++i) {
    const std::string& joint_name = output.joint_name[i];
    if (joint_name == kTestJointName) {
      const auto& joint = robot_plant.GetJointByName(joint_name);
      EXPECT_EQ(output.joint_position[i], input_value[joint.position_start()]);
      EXPECT_EQ(
          output.joint_velocity[i],
          input_value[robot_plant.num_positions() + joint.velocity_start()]);
    }
  }
}

}  // namespace
}  // namespace hsr
}  // namespace examples
}  // namespace drake
