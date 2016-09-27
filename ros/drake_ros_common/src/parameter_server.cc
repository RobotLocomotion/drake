#include "drake/ros/parameter_server.h"

#include <chrono>
#include <thread>

namespace drake {
namespace ros {

using std::this_thread::sleep_for;
using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;
using std::chrono::time_point;

typedef high_resolution_clock TimeClock;
typedef milliseconds TimeDuration;
typedef time_point<high_resolution_clock> TimePoint;

bool WaitForParameter(const std::string& parameter_name, double max_wait_time) {
  TimePoint start = TimeClock::now();
  while (::ros::ok() && !::ros::param::has(parameter_name)) {
    auto elapsed_time = TimeClock::now() - start;
    int elapsed_ms = duration_cast<milliseconds>(elapsed_time).count();
    if (elapsed_ms >= 1000 * max_wait_time)
      break;
    sleep_for(TimeDuration(100));  // Sleeps for 100 milliseconds.
  }
  return ::ros::param::has(parameter_name);
}

}  // namespace ros
}  // namespace drake
