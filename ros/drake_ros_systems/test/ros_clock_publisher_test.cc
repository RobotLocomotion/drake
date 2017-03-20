#include "drake/systems/ros_clock_publisher.h"

#include <mutex>

#include <gtest/gtest.h>
#include "ros/ros.h"
#include "rosgraph_msgs/Clock.h"

#include "drake/systems/framework/leaf_context.h"

using std::make_unique;
using std::string;

namespace drake {
namespace systems {
namespace test {
namespace {

// TODO(liang.fok) Update the unit test below once a ROS topic mocking framework
// exists.

// A test subscriber to ROS topic /clock. It is used to verify that the clock
// message published by the RosClockPublisher is actually receivable by a
// subscriber.
class ClockMessageReceiver {
 public:
  ClockMessageReceiver() : spinner_(1)  {
    received_message_.clock.sec = 0;
    received_message_.clock.nsec = 0;
    ros::NodeHandle node_handle;
    subscriber_ = node_handle.subscribe("clock", kSubscriberQueueSize,
                      &ClockMessageReceiver::ClockCallback, this);
     spinner_.start();
  }

  bool MessageReceived() {
    return message_received_;
  }

  rosgraph_msgs::Clock get_received_message() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return received_message_;
  }

 private:
  static const int kSubscriberQueueSize = 100;

  void ClockCallback(const rosgraph_msgs::Clock& msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    received_message_ = msg;
    message_received_ = true;
  }

  bool message_received_{false};
  ros::AsyncSpinner spinner_;
  ros::Subscriber subscriber_;
  std::mutex data_mutex_;
  rosgraph_msgs::Clock received_message_;
};

// Tests the RosClockPublisher by instantiating it, making it publish a
// rosgraph_msgs/Clock message, and verifying that the expected
// rosgraph_msgs/Clock message was transmitted.
GTEST_TEST(RosClockPublisherTest, TestRosClockPublisher) {
  ClockMessageReceiver message_receiver;

  LeafContext<double> context;
  context.set_time(1.5);

  RosClockPublisher dut;
  dut.DoPublish(context);

  // Verifies that the clock message was transmitted.
  const rosgraph_msgs::Clock& sent_message = dut.get_clock_message();
  EXPECT_EQ(sent_message.clock.sec, 1);
  EXPECT_EQ(sent_message.clock.nsec, 5e8);

  // Verifies that the clock message was received.
  int cycle_count = 0;
  static const int kMaxCycleCount = 10;
  while (dut.get_num_subscribers() == 0 && cycle_count++ < kMaxCycleCount) {
    ros::Duration(0.1).sleep();  // Sleeps for 0.1 seconds.
  }
  ASSERT_EQ(dut.get_num_subscribers(), 1);

  cycle_count = 0;
  while (!message_receiver.MessageReceived() &&
         cycle_count++ < kMaxCycleCount) {
    dut.DoPublish(context);
    ros::Duration(0.1).sleep();  // Sleeps for 0.1 seconds.
  }
  ASSERT_TRUE(message_receiver.MessageReceived());

  const rosgraph_msgs::Clock received_message =
      message_receiver.get_received_message();
  EXPECT_EQ(received_message.clock.sec, 1);
  EXPECT_EQ(received_message.clock.nsec, 5e8);
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
