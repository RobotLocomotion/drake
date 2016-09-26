#include "drake/ros/parameter_server.h"

#include <chrono>
#include <thread>

namespace drake {
namespace ros {

typedef std::chrono::high_resolution_clock TimeClock;
typedef std::chrono::milliseconds TimeDuration;
typedef std::chrono::time_point<std::chrono::high_resolution_clock> TimePoint;

using std::this_thread::sleep_for;
using std::chrono::duration_cast;

bool WaitForParameter(const std::string& parameter_name, double max_wait_time) {
  TimePoint start = TimeClock::now();
  while (::ros::ok() && !::ros::param::has(parameter_name) &&
      duration_cast<std::chrono::milliseconds>(TimeClock::now() - start).count()
      < 1000 * max_wait_time) {
    sleep_for(TimeDuration(100));  // Sleeps for 100 milliseconds.
  }
  return ::ros::param::has(parameter_name);
}

}  // namespace ros
}  // namespace drake
