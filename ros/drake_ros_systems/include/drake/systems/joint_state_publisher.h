#pragma once

// #include <chrono>

#include "ros/ros.h"
// #include "rosgraph_msgs/Clock.h"

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system_output.h"

namespace drake {
namespace systems {

/**
 * The JointStatePublisher takes the system state as the input and publishes
 * odometry information onto a ROS topic.
 *
 * It publishes a odometry information for each robot within the rigid body
 * system. The odometry messages for each robot are published onto different
 * ROS topics that are distinguished by robot name.
 *
 * @concept{system_concept}
 *
 * The resulting system has no internal state; the publish command is throttled
 * by kMinTransmitPeriod_.
 *
 * For convenience, the input is passed directly through as an output. This
 * enables other systems to be cascaded after this system.
 */
class JointStatePublisher : public LeafSystem<double> {
 public:
  /**
   * Specifies the minimum period in wall-clock seconds between successive
   * transmissions of rosgraph_msgs/Clock messages. This is to prevent flooding
   * ROS topic `/clock`.
   */
  static constexpr double kMinTransmitPeriod{0.01};

  /**
   * Constructs a JointStatePublisher by initializing a publisher on ROS topic
   * `/clock`.
   */
  JointStatePublisher();

  /**
   * Takes the current simulation time and publishes it on ROS topic `/clock`.
   */
  void DoPublish(const Context<double>& context) const
      override;

  /**
   * This System has no output ports. Thus, DoCalcOutput() does nothing.
   */
  void DoCalcOutput(const Context<double>& context,
                    SystemOutput<double>* output) const
      override {}

  /**
   * Returns the number of subscribers to this clock publisher.
   */
  int get_num_subscribers() const;

  // TODO(liang.fok) Remove this method once we have a proper mock-ROS-topic
  // framework in place.
  /**
   * An accessor to the most recently published rosgraph_msgs/Clock message.
   */
  const rosgraph_msgs::Clock& get_clock_message() const {
    return clock_message_;
  }

 private:
  // The most recently transmitted clock message.
  mutable rosgraph_msgs::Clock clock_message_;

  // The previous time the transform messages were sent.
  mutable std::chrono::time_point<std::chrono::steady_clock>
      previous_transmit_time_;

  // The object that actually publishes messages on a ROS topic.
  ::ros::Publisher clock_publisher_;
};

}  // namespace systems
}  // namespace drake
