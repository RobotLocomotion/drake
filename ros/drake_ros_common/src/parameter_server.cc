#include "drake/ros/parameter_server.h"

#include <chrono>
#include <thread>

namespace drake {
namespace ros {

typedef std::chrono::system_clock TimeClock;
typedef std::chrono::duration<double> TimeDuration;
typedef std::chrono::time_point<TimeClock, TimeDuration> TimePoint;

using std::this_thread::sleep_for;

bool WaitForParameter(const std::string& parameter_name, double max_wait_time) {
  TimePoint start_wall_time = TimeClock::now();
  while (::ros::ok() && !::ros::param::has(parameter_name)
      && (TimeClock::now() - start_wall_time).count() < max_wait_time) {
    sleep_for(TimeDuration(0.1));  // Sleeps for 1/10 of a second.
  }
  return ::ros::param::has(parameter_name);
}

}  // namespace ros
}  // namespace drake
