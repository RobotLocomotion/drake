#include "ros/ros.h"

#include <gtest/gtest.h>

namespace drake {
namespace systems {
namespace test {
namespace {

GTEST_TEST(RosTest, TestRosNode) {
  // Verifies that ros core is running.
  bool ros_master_exists = ros::master::check();
  EXPECT_TRUE(ros_master_exists);

  if (ros_master_exists) {
    // Tests the ability to instantiate a ROS node.
    ros::NodeHandle nh;
    EXPECT_TRUE(nh.ok());
  }
}

}  // namespace
}  // namespace test
}  // namespace systems
}  // namespace drake

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "drake_ros_test_node");
  return RUN_ALL_TESTS();
}
