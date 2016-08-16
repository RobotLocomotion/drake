#include "drake/ros/parameter_server.h"

namespace drake {
namespace ros {

// namespace {

void WaitForParameter(const ::ros::NodeHandle& ros_node_handle,
    const std::string& parameter_name, double max_wait_time) {
  ::ros::Time begin_time = ::ros::Time::now();

  while (::ros::ok() && !ros_node_handle.hasParam(parameter_name)
      && (::ros::Time::now() - begin_time).toSec() < max_wait_time) {
    ::ros::Duration(0.5).sleep();  // Sleeps for half a second.
  }

  if (!ros_node_handle.hasParam(parameter_name)) {
    throw std::runtime_error("ERROR: Failed to obtain parameter \"" +
        parameter_name + "\" from the ROS parameter server.");
  }
}

// }  // namespace

}  // namespace ros
}  // namespace drake
