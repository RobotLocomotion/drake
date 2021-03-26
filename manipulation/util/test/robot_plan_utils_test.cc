#include "drake/manipulation/util/robot_plan_utils.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"

namespace drake {
namespace manipulation {
namespace util {

const char kIiwaUrdf[] =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_no_collision.urdf";

GTEST_TEST(RobotPlanUtilsTest, GetJointNamesTest) {
  multibody::MultibodyPlant<double> plant(0.001);
  multibody::Parser(&plant).AddModelFromFile(FindResourceOrThrow(kIiwaUrdf));
  plant.WeldFrames(plant.world_frame(),
                   plant.GetBodyByName("base").body_frame());
  plant.Finalize();

  std::vector<std::string> joint_names = GetJointNames(plant);
  ASSERT_EQ(joint_names.size(), 7);
  EXPECT_EQ(joint_names[0], "iiwa_joint_1");
  EXPECT_EQ(joint_names[1], "iiwa_joint_2");
  EXPECT_EQ(joint_names[2], "iiwa_joint_3");
  EXPECT_EQ(joint_names[3], "iiwa_joint_4");
  EXPECT_EQ(joint_names[4], "iiwa_joint_5");
  EXPECT_EQ(joint_names[5], "iiwa_joint_6");
  EXPECT_EQ(joint_names[6], "iiwa_joint_7");
}

GTEST_TEST(RobotPlanUtilsTest, ApplyJointVelocityLimitsTest) {
  std::vector<double> times{0, 1};
  std::vector<Eigen::VectorXd> keyframes;
  keyframes.push_back(Eigen::VectorXd::Zero(2));
  keyframes.push_back(Eigen::VectorXd::Ones(2));
  Eigen::Vector2d limit(0.9, 1);

  ApplyJointVelocityLimits(keyframes, limit, &times);
  EXPECT_EQ(times[1], 1. / 0.9);
}

GTEST_TEST(RobotPlanUtilsTest, EncodeKeyFramesTest) {
  multibody::MultibodyPlant<double> plant(0.001);
  multibody::Parser(&plant).AddModelFromFile(FindResourceOrThrow(kIiwaUrdf));
  plant.WeldFrames(plant.world_frame(),
                   plant.GetBodyByName("base").body_frame());
  plant.Finalize();

  std::vector<std::string> joint_names = GetJointNames(plant);
  std::vector<double> times{0, 2};
  std::vector<Eigen::VectorXd> keyframes;
  Eigen::VectorXd q(7);
  q << 1, 2, 3, 4, 5, 6, 7;
  keyframes.push_back(q);
  q << 8, 9, 10, 11, 12, 13, 14;
  keyframes.push_back(q);

  lcmt_robot_plan plan =
      EncodeKeyFrames(joint_names, times, keyframes);

  ASSERT_EQ(plan.plan.size(), 2);
  EXPECT_EQ(plan.plan.size(), plan.num_states);
  for (int i = 0; i < static_cast<int>(plan.plan.size()); ++i) {
    const lcmt_robot_state& step = plan.plan[i];
    EXPECT_EQ(step.utime, i * 2 * 1e6);
    ASSERT_EQ(step.num_joints, 7);
    for (int j = 0; j < step.num_joints; j++) {
      EXPECT_EQ(step.joint_position[j], i * 7 + j + 1);
    }
  }
}

}  // namespace util
}  // namespace manipulation
}  // namespace drake
