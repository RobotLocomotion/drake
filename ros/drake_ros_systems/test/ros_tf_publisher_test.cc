#include <gtest/gtest.h>

#include "ros/ros.h"

namespace drake {
namespace systems {
namespace test {
namespace {

GTEST_TEST(RosTfPublisherTest, TestRosTfPublisher) {
  auto tree = make_unique<RigidBodyTree<double>>();
  auto publisher = make_unique<RosTfPublisher<double>>(*tree);

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
