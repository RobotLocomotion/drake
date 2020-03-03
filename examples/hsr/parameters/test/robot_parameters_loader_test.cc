#include "drake/examples/hsr/parameters/robot_parameters_loader.h"

#include <string>

#include <gtest/gtest.h>

#include "drake/examples/hsr/parameters/robot_parameters.h"

namespace drake {
namespace examples {
namespace hsr {
namespace parameters {
namespace {

constexpr int kNumTestRobotPart1Joints = 1;
constexpr int kNumTestRobotJoints = 2;
constexpr double kPositionLimitLower = -10.0;
constexpr double kPositionLimitUpper = 10.0;
constexpr double kVelocityLimit = 10.0;
constexpr double kPidGainKd = 40.0;

// Test the loading of the default HSR yaml file. Confirm a) the number of
// joints and b) the values of different entries are loaded correctly.
GTEST_TEST(RobotParametersLoaderTest, LoadingTest) {
  const std::string robot_name = "test_robot";
  const std::string filepath_prefix = "drake/examples/hsr/parameters/test/";

  RobotParameters<double> robot_parameters;
  robot_parameters.name = robot_name;
  const bool load_successful =
      ReadParametersFromFile(robot_name, filepath_prefix, &robot_parameters);

  EXPECT_TRUE(load_successful);

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
      EXPECT_EQ(joint_parameters.pid_gains.kd, kPidGainKd);
    }
  }
  EXPECT_EQ(num_test_robot_joints, kNumTestRobotJoints);

  const auto& cameras_parameters = robot_parameters.cameras_parameters;
  const auto& hand_camera = cameras_parameters.find("hand_camera");
  EXPECT_TRUE(hand_camera != cameras_parameters.end());
  EXPECT_EQ(hand_camera->second.location.parent_frame_name, "hand");
}

}  // namespace
}  // namespace parameters
}  // namespace hsr
}  // namespace examples
}  // namespace drake
