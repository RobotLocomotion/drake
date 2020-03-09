#include "drake/examples/hsr/sim_status_receiver.h"

#include <gtest/gtest.h>

#include "drake/examples/hsr/test_utilities/sim_lcm_test_configs.h"
#include "drake/lcmt_hsr_sim_status.hpp"

namespace drake {
namespace examples {
namespace hsr {
namespace {

const char* kRobotModelUrdf =
    "drake/examples/hsr/models/urdfs/hsrb4s_fix_free_joints.urdf";
// This joint name must exist in the given urdf model and it must be an
// actuated joint.
const char* kTestJointName = "arm_roll_joint";

// Given a input message, test the message should be parsed properly.
// An arbitrary joint is used to confirm the output values are reading
// correctly.
GTEST_TEST(SimStatusReceiverTest, MessageParsingTest) {
  SimLcmTestConfig test_config{kRobotModelUrdf};

  const auto& robot_plant = test_config.robot_plant();
  SimStatusReceiver dut(&robot_plant);

  std::unique_ptr<systems::Context<double>> context =
      dut.CreateDefaultContext();

  const auto& input_value = test_config.sim_status_message();
  dut.get_sim_status_input_port().FixValue(context.get(), input_value);

  const auto& output =
      dut.get_estimated_state_output_port().Eval<systems::BasicVector<double>>(
          *context);

  for (int i = 0; i < input_value.num_joints; ++i) {
    const std::string& joint_name = input_value.joint_name[i];
    if (joint_name == kTestJointName) {
      const auto& joint = robot_plant.GetJointByName(joint_name);
      EXPECT_EQ(input_value.joint_position[i], output[joint.position_start()]);
      EXPECT_EQ(input_value.joint_velocity[i],
                output[robot_plant.num_positions() + joint.velocity_start()]);
    }
  }
}

}  // namespace
}  // namespace hsr
}  // namespace examples
}  // namespace drake
