#include "drake/ros/parameter_server.h"

namespace drake {
namespace ros {

bool WaitForParameter(const std::string& parameter_name, double max_wait_time) {
  ::ros::WallTime begin_time = ::ros::WallTime::now();
  while (::ros::ok() && !::ros::param::has(parameter_name)) {
    ::ros::WallDuration elapsed_time = ::ros::WallTime::now() - begin_time;
    if (elapsed_time.toSec() >= max_wait_time)
      break;
    ::ros::WallDuration(0.1).sleep();  // Sleeps for 0.1 seconds.
  }
  return ::ros::param::has(parameter_name);
}

}  // namespace ros
}  // namespace drake
