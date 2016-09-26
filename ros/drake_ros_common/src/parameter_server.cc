#include "drake/ros/parameter_server.h"

namespace drake {
namespace ros {

void WaitForParameter(const std::string& parameter_name, double max_wait_time) {
  ::ros::Time begin_time = ::ros::Time::now();
  while (::ros::ok() && !::ros::param::has(parameter_name)
      && (::ros::Time::now() - begin_time).toSec() < max_wait_time) {
    ::ros::Duration(0.1).sleep();  // Sleeps for 1/10 of a second.
  }
}

}  // namespace ros
}  // namespace drake
