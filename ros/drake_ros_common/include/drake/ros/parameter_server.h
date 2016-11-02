#pragma once

#include <string>

#include "ros/ros.h"

namespace drake {
namespace ros {

/**
 * Waits up to @p max_wait_time for parameter @p parameter_name to exist on the
 * ROS parameter server.
 *
 * Note that this method calls `ros::Time::now()` and thus requires that either
 * a node handle be created  or `ros::start()` be called prior to this method
 * being called.
 *
 * @param[in] parameter_name The name of the parameter to obtain from the ROS
 * parameter server.
 *
 * @param[in] max_wait_time The maximum amount of wall clock time to wait for
 * the parameter to become available before returning.
 *
 * @return This method returns true if the parameter comes into existence prior
 * to @p max_wait_time and false otherwise.
 */
bool WaitForParameter(const std::string& parameter_name,
    double max_wait_time = 1.0);

/**
 * Returns a parameter from the ROS parameter server. Throws a
 *`std::runtime_error` exception if the parameter is not found after
 * @p max_wait_time or if the parameter exists but is not a type that can be
 * converted into `T`.
 *
 * @tparam T The parameter type. A full list of types supported by ROS are
 * available here: http://wiki.ros.org/Parameter%20Server#Parameter_Types.
 * Currently this method only has test coverage for the following types:
 * `double`, `int`, `bool`, and `std::string`. Notably, the following types
 * are untested: iso8601 dates, lists, and base64-encoded binary data.
 *
 * @param[in] parameter_name The name of the parameter to obtain.
 *
 * @param[in] max_wait_time The maximum wall clock time to wait for the
 * parameter to become available on the ROS parameter server.
 *
 * @returns The value of the parameter.
 *
 * @throws std::runtime_error If the parameter does not exist after
 * @p max_wait_time or it exists but is not of a compatible type.
 */
template<typename T>
T GetRosParameterOrThrow(const std::string& parameter_name,
    double max_wait_time = 5.0) {
  WaitForParameter(parameter_name, max_wait_time);
  T parameter{};
  if (!::ros::param::get(parameter_name, parameter)) {
    throw std::runtime_error(
        "ERROR: Failed to obtain parameter \"" + parameter_name + "\" from "
        "the ROS parameter server.");
  }
  return parameter;
}

/**
 * Returns a parameter from the ROS parameter server. Returns @p default_value
 * if the parameter does not exist after @p max_wait_time or if the parameter
 * exists but is not a type that can be converted into `T`.
 *
 * @tparam T The parameter type. A full list of types supported by ROS are
 * available here: http://wiki.ros.org/Parameter%20Server#Parameter_Types.
 * Currently this method only has test coverage for the following types:
 * `double`, `int`, `bool`, and `std::string`. Notably, the following types
 * are untested: iso8601 dates, lists, and base64-encoded binary data.
 *
 * @param[in] parameter_name The name of the parameter to obtain.
 *
 * @param[in] default_value The value that is returned if the parameter does not
 * exist after waiting @p max_wait_time.
 *
 * @param[in] max_wait_time The maximum wall clock time to wait for the
 * parameter to become available on the ROS parameter server.
 *
 * @returns The value of the parameter, or @p default_value if there was any
 * problem obtaining the parameter.
 */
template<typename T>
T GetRosParameterOrDefault(const std::string& parameter_name, T default_value,
    double max_wait_time = 5.0) {
  WaitForParameter(parameter_name, max_wait_time);
  T parameter = default_value;
  if (::ros::ok() && ::ros::param::has(parameter_name)) {
    ::ros::param::param<T>(parameter_name, parameter, default_value);
  }
  return parameter;
}

}  // namespace ros
}  // namespace drake
