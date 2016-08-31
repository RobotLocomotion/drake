#pragma once

#include "ros/ros.h"

namespace drake {
namespace ros {

/**
 * Waits up to @p max_wait_time for parameter @p parameter_name to exist on the
 * ROS parameter server. Throws an `std::runtime_error` exception if the
 * parameter does not show up prior to @p max_wait_time elapsing.
 */
void WaitForParameter(const ::ros::NodeHandle& ros_node_handle,
    const std::string& parameter_name, double max_wait_time = 5.0);

/**
 * Returns a parameter from the ROS parameter server. Throws an
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
 * @returns The value of the parameter.
 *
 * @throws std::runtime_error If the parameter is not available on the ROS
 * parameter server even after waiting @p max_wait_time seconds, or if the
 * parameter is not of the correct type.
 */
template<typename T>
T GetROSParameter(const ::ros::NodeHandle &ros_node_handle,
    const std::string& parameter_name, double max_wait_time = 5.0) {
  WaitForParameter(ros_node_handle, parameter_name, max_wait_time);
  T parameter;
  if (!ros_node_handle.getParam(parameter_name, parameter)) {
    throw std::runtime_error(
      "ERROR: Failed to obtain parameter \"" + parameter_name + "\" from "
      "the ROS parameter server.");
  }

  return parameter;
}

}  // namespace ros
}  // namespace drake
