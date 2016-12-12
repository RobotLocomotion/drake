#include "drake/systems/ros_clock_publisher.h"

#include <gtest/gtest.h>

#include "drake/systems/framework/leaf_context.h"
#include "ros/ros.h"

using std::make_unique;
using std::string;

namespace drake {
namespace systems {
namespace test {
namespace {

// Tests the RosClockPublisher by instantiating it, making it publish a
// rosgraph_msgs/Clock message, and verifying that the expected
// rosgraph_msgs/Clock message was transmitted.
GTEST_TEST(RosClockPublisherTest, TestRosClockPublisher) {
  LeafContext<double> context;
  context.set_time(1.5);

  RosClockPublisher dut;
  dut.DoPublish(context);

  const rosgraph_msgs::Clock& message = dut.get_clock_message();
  EXPECT_EQ(message.clock.sec, 1);
  EXPECT_EQ(message.clock.nsec, 5e8);
}

}  // namespace
}  // namespace test
}  // namespace systems
}  // namespace drake

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "drake_ros_clock_publisher_test_node");
  return RUN_ALL_TESTS();
}
