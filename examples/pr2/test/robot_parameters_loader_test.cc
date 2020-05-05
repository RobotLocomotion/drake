#include "drake/examples/pr2/robot_parameters_loader.h"

#include <string>

#include <gtest/gtest.h>

#include "drake/examples/pr2/robot_parameters.h"

namespace drake {
namespace examples {
namespace pr2 {
namespace {

// These are the parameters in the test .yaml files. They are listed here for
// easy access purpose.
constexpr int kNumTestRobotPart1Joints = 2;
constexpr int kNumTestRobotJoints = 3;
constexpr double kPositionLimitLower = -10.0;
constexpr double kPositionLimitUpper = 10.0;
constexpr double kVelocityLimit = 10.0;
constexpr double kPidGainKd = 40.0;

const char* kFilepathPrefix = "drake/examples/pr2/test/";

// Test the loading of a valid test yaml file. Confirm a) the number of
// joints and b) the values of different entries are loaded correctly.
GTEST_TEST(RobotParametersLoaderTest, LoadingTest) {
  const std::string robot_name = "test_robot_valid";
  RobotParameters robot_parameters;
  const bool load_successful =
      ReadParametersFromFile(robot_name, kFilepathPrefix, &robot_parameters);

  EXPECT_TRUE(load_successful);
  EXPECT_EQ(robot_parameters.name, robot_name);
  EXPECT_EQ(robot_parameters.model_instance_info.model_path, "test_path");
  EXPECT_EQ(robot_parameters.model_instance_info.child_frame_name, "base_link");
  EXPECT_TRUE(robot_parameters.model_instance_info.is_floating_base);

  const auto& parts_parameters = robot_parameters.parts_parameters;
  int num_test_robot_joints = 0;
  for (const auto& [part_name, part_parameters] : parts_parameters) {
    const int num_part_joints = part_parameters.joints_parameters.size();
    if (part_name == "part1") {
      EXPECT_EQ(kNumTestRobotPart1Joints, num_part_joints);
    }
    num_test_robot_joints += num_part_joints;
    for (const auto& joint_parameters : part_parameters.joints_parameters) {
      EXPECT_EQ(joint_parameters.position_limit_lower, kPositionLimitLower);
      EXPECT_EQ(joint_parameters.position_limit_upper, kPositionLimitUpper);
      EXPECT_EQ(joint_parameters.velocity_limit, kVelocityLimit);
      EXPECT_EQ(joint_parameters.actuators_parameters[0].gains.kd, kPidGainKd);
    }
  }
  EXPECT_EQ(num_test_robot_joints, kNumTestRobotJoints);
}

// Test the loading of a invalid test yaml file. Confirm that the loading result
// will be false if there are joints with duplicate name.
GTEST_TEST(RobotParametersLoaderTest, DuplicateJointNamesTest) {
  const std::string robot_name = "test_robot_invalid";
  RobotParameters robot_parameters;
  EXPECT_FALSE(
      ReadParametersFromFile(robot_name, kFilepathPrefix, &robot_parameters));
}

}  // namespace
}  // namespace pr2
}  // namespace examples
}  // namespace drake
