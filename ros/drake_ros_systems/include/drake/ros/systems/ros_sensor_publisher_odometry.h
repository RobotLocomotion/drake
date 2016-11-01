#pragma once

#include <map>
#include <string>
#include <utility>

#include <Eigen/Dense>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

#include "drake/system1/System.h"
#include "drake/systems/plants/KinematicsCache.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/system1/vector.h"

using drake::NullVector;
using drake::RigidBodySensor;
using drake::RigidBodySystem;
using drake::RigidBodyDepthSensor;

namespace drake {
namespace ros {
namespace systems {

/**
 * @brief A system that takes the system state as the input and publishes
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
template <template <typename> class RobotStateVector>
class SensorPublisherOdometry {
 private:
  // Specifies that the odometry messages should be transmitted with a minimum
  // period of 0.05 seconds.
  static constexpr double kMinTransmitPeriod_ = 0.05;

 public:
  template <typename ScalarType>
  using StateVector = NullVector<ScalarType>;
  template <typename ScalarType>
  using OutputVector = RobotStateVector<ScalarType>;
  template <typename ScalarType>
  using InputVector = RobotStateVector<ScalarType>;

  /**
   * The constructor.
   *
   * @param[in] rigid_body_system The rigid body system whose output contains
   * the odometry information.
   */
  explicit SensorPublisherOdometry(
      std::shared_ptr<RigidBodySystem> rigid_body_system)
      : rigid_body_system_(rigid_body_system) {
    // Initializes the time stamp of the previous transmission to be zero.
    previous_send_time_.sec = 0;
    previous_send_time_.nsec = 0;

    // Instantiates a ROS node handle through which we can interact with ROS.
    // For more information, see:
    // http://wiki.ros.org/roscpp/Overview/NodeHandles
    ::ros::NodeHandle nh;

    // Obtains a reference to the world link in the rigid body tree.
    const RigidBody& world = rigid_body_system->getRigidBodyTree()->world();

    // Creates a ROS topic publisher for each robot in the rigid body system.
    // A robot is defined by any link that's connected to the world via a
    // non-fixed joint.
    for (auto const& rigid_body :
         rigid_body_system->getRigidBodyTree()->bodies) {
      // Skips the current rigid body if it does not have the world as the
      // parent.
      if (!rigid_body->has_as_parent(world)) continue;

      // Skips the current rigid body if it's not connected to the world via a
      // floating joint.
      if (!rigid_body->getJoint().is_floating()) continue;

      // Creates an odometry message and publisher for the current robot if they
      // have not already been created. Stores them in odometry_publishers_ and
      // odometry_messages_.

      // The key is simply the model name since there should only be one
      // odometry message and publisher per robot.
      const std::string& key = rigid_body->get_model_name();

      if (odometry_messages_.find(key) == odometry_messages_.end()) {
        const std::string topic_name = "drake/" + key + "/odometry";

        odometry_publishers_.insert(std::pair<std::string, ::ros::Publisher>(
            key, nh.advertise<nav_msgs::Odometry>(topic_name, 1)));

        std::unique_ptr<nav_msgs::Odometry> message(new nav_msgs::Odometry());
        message->header.frame_id = RigidBodyTree::kWorldName;
        message->child_frame_id = rigid_body->get_name();

        odometry_messages_.insert(
            std::pair<std::string, std::unique_ptr<nav_msgs::Odometry>>(
                key, std::move(message)));
      } else {
        throw std::runtime_error(
            "ERROR: Rigid Body System contains multiple models named \"" + key +
            "\".");
      }
    }
  }

  StateVector<double> dynamics(const double& t, const StateVector<double>& x,
                               const InputVector<double>& u) const {
    return StateVector<double>();
  }

  OutputVector<double> output(const double& t, const StateVector<double>& x,
                              const InputVector<double>& u) {
    // Aborts if insufficient time has passed since the last transmission. This
    // is to avoid flooding the ROS topics.
    ::ros::Time current_time = ::ros::Time::now();
    if ((current_time - previous_send_time_).toSec() < kMinTransmitPeriod_)
      return u;

    previous_send_time_ = current_time;

    const std::shared_ptr<RigidBodyTree>& rigid_body_tree =
        rigid_body_system_->getRigidBodyTree();

    // The input vector u contains the entire system's state. The following
    // The following code extracts the position and velocity values from it
    // and computes the kinematic properties of the system.
    auto uvec = drake::toEigen(u);
    auto q = uvec.head(rigid_body_tree->get_num_positions());    // position
    auto v = uvec.segment(rigid_body_tree->get_num_positions(),  // velocity
                          rigid_body_tree->get_num_velocities());
    KinematicsCache<double> cache = rigid_body_tree->doKinematics(q, v);

    // Obtains a reference to the world link in the rigid body tree.
    const RigidBody& world = rigid_body_tree->world();

    // Publishes an odometry message for each rigid body that's connected via a
    // floating (non-fixed) joint to the world.
    for (auto const& rigid_body : rigid_body_tree->bodies) {
      // Skips the current rigid body if it does not have the world as the
      // parent.
      if (!rigid_body->has_as_parent(world)) continue;

      // Skips the current rigid body if it's not connected to the world via a
      // floating joint.
      if (!rigid_body->getJoint().is_floating()) continue;

      // Defines the key that can be used to obtain the publisher and message.
      // The key is simply the model name since there should only be one
      // odometry message and publisher per model.
      const std::string& key = rigid_body->get_model_name();

      // Verifies that a nav_msgs::Odometry message for the current link exists
      // in the odometry_messages_ map.
      auto message_in_map = odometry_messages_.find(key);
      if (message_in_map == odometry_messages_.end()) {
        throw std::runtime_error(
            "ERROR: SensorPublisherOdmetry: Unable to find"
            "odometry message using key " +
            key);
      }

      // Verifies that the publisher exists in the odometry_publishers_ map.
      auto publisher_in_map = odometry_publishers_.find(key);
      if (publisher_in_map == odometry_publishers_.end()) {
        throw std::runtime_error(
            "ERROR: SensorPublisherOdmetry: Unable to find"
            "odometry publisher using key " +
            key);
      }

      nav_msgs::Odometry* message = message_in_map->second.get();

      // Updates the odometry information in the odometry message.
      auto transform = rigid_body_tree->relativeTransform(
          cache, rigid_body_tree->FindBodyIndex(
              rigid_body->get_parent()->get_name()),
          rigid_body_tree->FindBodyIndex(rigid_body->get_name()));
      auto translation = transform.translation();
      auto quat = drake::math::rotmat2quat(transform.linear());

      // Saves the robot's position and orientation in the world.
      message->pose.pose.position.x = translation(0);
      message->pose.pose.position.y = translation(1);
      message->pose.pose.position.z = translation(2);

      message->pose.pose.orientation.w = quat(0);
      message->pose.pose.orientation.x = quat(1);
      message->pose.pose.orientation.y = quat(2);
      message->pose.pose.orientation.z = quat(3);

      // Saves the robot's linear and angular velocities in the world.
      auto twist = rigid_body_tree->relativeTwist(
          cache, rigid_body_tree->FindBodyIndex(
              rigid_body->get_parent()->get_name()),
          rigid_body_tree->FindBodyIndex(rigid_body->get_name()),
          rigid_body_tree->FindBodyIndex(RigidBodyTree::kWorldName));

      message->twist.twist.linear.x = twist(0);
      message->twist.twist.linear.y = twist(1);
      message->twist.twist.linear.z = twist(2);

      message->twist.twist.angular.x = twist(3);
      message->twist.twist.angular.y = twist(4);
      message->twist.twist.angular.z = twist(5);

      // Updates the time stamp in the transform message.
      message->header.stamp = current_time;

      publisher_in_map->second.publish(*message);
    }

    return u;  // Passes the output through to the next system in the cascade.
  }

  bool isTimeVarying() const { return true; }
  bool isDirectFeedthrough() const { return true; }

 private:
  std::shared_ptr<RigidBodySystem> rigid_body_system_;

  /**
   * Maintains a set of ROS topic publishers for publishing LIDAR messages.
   * The key is the name of the sensor. The value is the ROS topic publisher.
   */
  std::map<std::string, ::ros::Publisher> odometry_publishers_;

  /**
   * Maintains a set of ROS nav_msgs::Odometry messages for use by the
   * publishers. This is used to avoid having to allocate a new message
   * each time one needs to be sent.
   */
  std::map<std::string, std::unique_ptr<nav_msgs::Odometry>> odometry_messages_;

  /**
   * The previous time the LIDAR messages were sent.
   */
  ::ros::Time previous_send_time_;
};

}  // namespace systems
}  // namespace ros
}  // namespace drake
