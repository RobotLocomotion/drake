#include <gtest/gtest.h>

#include <chrono>
#include <mutex>
#include <thread>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

namespace drake {
namespace ros {
namespace automotive {
namespace {

typedef std::chrono::milliseconds TimeDuration;

using std::this_thread::sleep_for;

class JointStateCallback {
 public:
  JointStateCallback() { }

  void Callback(const sensor_msgs::JointState& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    received_joint_state_ = true;
  }

  bool received_state() const {
    bool result{false};
    std::lock_guard<std::mutex> lock(mutex_);
    result = received_joint_state_;
    return result;
  }

 private:
  bool received_joint_state_{false};
  mutable std::mutex mutex_;
};


GTEST_TEST(DrakeSingleCarInStataGarageTest, BasicTest) {
  EXPECT_TRUE(::ros::master::check());

  // Verifies that the ROS node running the Drake simulation exists. It loops
  // searching for the a ROS node called "/drake/single_car_in_stata_garage".
  // If it fails to find such a node, it sleeps for 100 milliseconds of wall
  // clock time before trying again. Wall clock time is used to prevent the
  // following loop from live locking when the simulation fails to start and
  // does not publish the simulated clock time.
  bool drake_node_found{false};
  int num_tries = 0;

  while (!drake_node_found && num_tries++ < 10) {
    ::ros::V_string node_list;
    EXPECT_TRUE(::ros::master::getNodes(node_list));

    for(auto it : node_list) {
      std::string node_name = it;
      if (node_name == "/drake/single_car_in_stata_garage")
        drake_node_found = true;
    }

    if (!drake_node_found)
      sleep_for(TimeDuration(100));  // Sleeps for 100 milliseconds.
    ::ros::spinOnce();
  }

  EXPECT_TRUE(drake_node_found);

  // Verifies that the Drake simulator is publishing `sensor_msgs::JointState`
  // messages on ROS topic `/drake/prius/joint_state`. This is used as an
  // indicator that the simulation is running.
  JointStateCallback callback;
  ::ros::NodeHandle node_handle;
  ::ros::Subscriber subscriber =
      node_handle.subscribe("/drake/prius/joint_state", 100 /* queue size */,
                            &JointStateCallback::Callback, &callback);

  int num_publishers{0};
  while (num_publishers == 0 && num_tries++ < 5) {
    num_publishers = subscriber.getNumPublishers();
    if (num_publishers == 0)
      sleep_for(TimeDuration(1000));  // Sleeps for 1000 milliseconds.
  }
  EXPECT_EQ(subscriber.getNumPublishers(), 1);

  bool received_state{false};
  num_tries = 0;
  while (!received_state && num_tries++ < 5) {
    ::ros::spinOnce();
    received_state = callback.received_state();
    if (!received_state)
      sleep_for(TimeDuration(1000));  // Sleeps for 1000 milliseconds.
  }

  EXPECT_TRUE(received_state);
}

}  // namespace
}  // namespace automotive
}  // namespace ros
}  // namespace drake

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "single_car_in_stata_garage_test");
  return RUN_ALL_TESTS();
}
