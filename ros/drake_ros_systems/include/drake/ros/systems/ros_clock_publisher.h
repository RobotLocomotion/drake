#pragma once

// #include <stdexcept>

// #include <Eigen/Dense>

#include "ros/ros.h"
#include "rosgraph_msgs/Clock.h"

// #include "drake/systems/System.h"
// #include "drake/systems/plants/KinematicsCache.h"
// #include "drake/systems/plants/RigidBodySystem.h"
// #include "drake/systems/plants/RigidBodyTree.h"
// #include "drake/systems/vector.h"

using drake::NullVector;
using drake::RigidBodySensor;
using drake::RigidBodySystem;
using drake::RigidBodyDepthSensor;

using Eigen::VectorXd;

namespace drake {
namespace ros {
namespace systems {

/**
 * @brief A system that publishes the current simulation time on ROS topic
 * /clock.takes the system state as the input and publishes
 * joint state information onto ROS topics.
 *
 * For convenience, the input is passed directly through as an output. This
 * enables other systems to be cascaded after this system.
 *
 * @concept{system_concept}
 */
template <template <typename> class RobotStateVector>
class RosClockPublisher {
 public:
  template <typename ScalarType>
  using StateVector = NullVector<ScalarType>;
  template <typename ScalarType>
  using OutputVector = RobotStateVector<ScalarType>;
  template <typename ScalarType>
  using InputVector = RobotStateVector<ScalarType>;

  /**
   * The constructor, which initializes the clock publisher.
   */
  RosClockPublisher() {
    // Instantiates a ROS node handle. For more information, see:
    // http://wiki.ros.org/rosgraph_msgscpp/Overview/NodeHandles.
    ::ros::NodeHandle node_handle;

    // Instantiates a ROS topic publisher for publishing clock information. For
    // more information, see: http://wiki.ros.org/Clock.
    clock_publisher_ =
        node_handle.advertise<rosgraph_msgs::Clock>("/clock", 1);
  }

  StateVector<double> dynamics(const double& t, const StateVector<double>& x,
                               const InputVector<double>& u) const {
    return StateVector<double>();
  }

  OutputVector<double> output(const double& t, const StateVector<double>& x,
                              const InputVector<double>& u) {
    // Computes the whole-second portion and the fractional-second portion of
    // the current simulatoin time @p t.
    double whole_part, fractional_part;
    fractional_part = modf(t, &whole_part);

    // Saves the time in the clock message.
    rosgraph_msgs::Clock clock_msg;
    clock_msg.clock.sec = static_cast<int>(whole_part);
    clock_msg.clock.nsec = static_cast<int>(fractional_part * 1e9);
    clock_publisher_.publish(clock_msg);

    return u;  // Passes the output through to the next system in the cascade.
  }

  bool isTimeVarying() const { return true; }
  bool isDirectFeedthrough() const { return true; }

 private:
  ::ros::Publisher clock_publisher_;
};

}  // namespace systems
}  // namespace ros
}  // namespace drake
