#include "drake/systems/ros_clock_publisher.h"

#include <cmath>

using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::steady_clock;

namespace drake {
namespace systems {

using std::make_unique;

RosClockPublisher::RosClockPublisher() {
  clock_message_.clock.sec = 0;
  clock_message_.clock.nsec = 0;
  ros::NodeHandle node_handle;
  clock_publisher_ = node_handle.advertise<rosgraph_msgs::Clock>("/clock", 1);
}

void RosClockPublisher::DoPublish(const Context<double>& context) const {
  // Aborts the transmission if the minimum transmission period hasn't been met.
  if (clock_message_.clock.sec != 0 || clock_message_.clock.nsec != 0) {
    double duration = duration_cast<milliseconds>(steady_clock::now() -
                        previous_transmit_time_).count();
    if (duration < kMinTransmitPeriod)
      return;
  }

  previous_transmit_time_ = steady_clock::now();

  double drake_time = context.get_time();
  double whole_seconds{0};
  double fractional_seconds = modf(drake_time, &whole_seconds);
  clock_message_.clock.sec = static_cast<int>(whole_seconds);
  clock_message_.clock.nsec = static_cast<int>(fractional_seconds * 1e9);
  clock_publisher_.publish(clock_message_);
}

int RosClockPublisher::get_num_subscribers() const {
  return static_cast<int>(clock_publisher_.getNumSubscribers());
}

}  // namespace systems
}  // namespace drake
