#include "ros/ros.h"

namespace drake {
namespace ros {

namespace {
// Waits up to @p max_wait_time for a particular parameter to exist on the ROS
// parameter server. Throws an std::runtime_error exception if the parameter
// fails to show up prior to this deadline.
void WaitForParameter(::ros::NodeHandle& ros_node_handle,
    const std::string& parameter_name, double max_wait_time) {
  ::ros::Time begin_time = ::ros::Time::now();

  while (::ros::ok() && !ros_node_handle.hasParam(parameter_name)
      && (::ros::Time::now() - begin_time).toSec() < max_wait_time) {
    ::ros::Duration(0.5).sleep(); // Sleeps for half a second.
  }

  if (!ros_node_handle.hasParam(parameter_name)) {
    throw std::runtime_error("ERROR: Failed to obtain parameter \"" +
        parameter_name + "\" from the ROS parameter server.");
  }
}

}  // namespace

/**
 * Returns a string parameter from the ROS parameter server. Throws an
 *`std::runtime_error` exception if it fails to obtain the parameter.
 *
 * @param[in] node_handle The ROS node handle to use to access the ROS parameter
 * server.
 *
 * @param[in] parameter_name The name of the parameter to obtain.
 *
 * @param[in] max_wait_time The maximum time to wait for the parameter to
 * become available on the ROS parameter server.
 *
 * @returns The integer value of the parameter.
 *
 * @throws std::runtime_error If the parameter is not available on the ROS
 * parameter server even after waiting @p max_wait_time seconds, or if the
 * parameter is not of the correct type.
 */
std::string GetStringParameter(::ros::NodeHandle &ros_node_handle,
    const std::string& parameter_name, double max_wait_time = 5.0) {
  WaitForParameter(ros_node_handle, parameter_name, max_wait_time);
  std::string parameter;
  if (!ros_node_handle.getParam(parameter_name, parameter)) {
    throw std::runtime_error(
      "ERROR: Failed to obtain parameter \"" + parameter_name + "\" from "
      "the ROS parameter server.");
  }

  return parameter;
}

/**
 * Returns an integer parameter from the ROS parameter server. Throws an
 *`std::runtime_error` exception if it fails to obtain the parameter.
 *
 * @param[in] node_handle The ROS node handle to use to access the ROS parameter
 * server.
 *
 * @param[in] parameter_name The name of the parameter to obtain.
 *
 * @param[in] max_wait_time The maximum time to wait for the parameter to
 * become available on the ROS parameter server.
 *
 * @returns The integer value of the parameter.
 *
 * @throws std::runtime_error If the parameter is not available on the ROS
 * parameter server even after waiting @p max_wait_time seconds, or if the
 * parameter is not of the correct type.
 */
int GetIntParameter(::ros::NodeHandle &node_handle,
    const std::string& parameter_name, double max_wait_time = 5.0) {
  WaitForParameter(node_handle, parameter_name, max_wait_time);
  int parameter;
  if (!node_handle.getParam(parameter_name, parameter)) {
    throw std::runtime_error(
      "ERROR: Failed to obtain parameter \"" + parameter_name + "\" from "
      "the ROS parameter server.");
  }

  return parameter;
}

}  // namespace ros
}  // namespace drake
