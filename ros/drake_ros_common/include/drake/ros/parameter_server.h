#pragma once

#include "ros/ros.h"

namespace drake {
namespace ros {

/**
 * Waits up to @p max_wait_time for parameter @p parameter_name to exist on the
 * ROS parameter server. Throws an `std::runtime_error` exception if the
 * parameter does not show up prior to @p max_wait_time elapsing.
 *
 * Note that this method calls `ros::Time::now()` and thus requires that either
 * a node handle be created  or `ros::start()` be called prior to this method
 * being called.
 */
void WaitForParameter(const std::string& parameter_name,
    double max_wait_time = 1.0);

/**
 * Returns a parameter from the ROS parameter server. Throws an
 *`std::runtime_error` exception if it fails to obtain the parameter.
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
T GetROSParameter(const std::string& parameter_name,
    double max_wait_time = 5.0) {
  WaitForParameter(parameter_name, max_wait_time);
  T parameter;
  if (!::ros::param::get(parameter_name, parameter)) {
    throw std::runtime_error(
      "ERROR: Failed to obtain parameter \"" + parameter_name + "\" from "
      "the ROS parameter server.");
  }

  return parameter;
}

}  // namespace ros
}  // namespace drake
