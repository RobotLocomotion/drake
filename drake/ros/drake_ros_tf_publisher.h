#pragma once

#include <Eigen/Dense>

#include "ros/ros.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_broadcaster.h"

#include "drake/core/Vector.h"
#include "drake/systems/System.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/RigidBodySystem.h"

using Drake::NullVector;
using Drake::RigidBodySensor;
using Drake::RigidBodySystem;
using Drake::RigidBodyDepthSensor;

namespace drake {
namespace systems {
namespace plants {

/** DrakeRosTfPublisher<RobotStateVector>
 * @brief A system that takes the current state of Drake and publishes the
 * transform messages on ROS topic /tf.
 *
 * @concept{system_concept}
 *
 * The resulting system has no internal state; the publish command is executed
 * on every call to the output method.
 *
 * For convenience, the input is passed directly through as an output.
 */

template <template <typename> class RobotStateVector>
class DrakeRosTfPublisher {
 private:
  // Specifies that the transforms should be transmitted at most ten times per
  // second.
  static constexpr double kMinTransmitPeriod_ = 0.1;

 public:
  template <typename ScalarType>
  using StateVector = NullVector<ScalarType>;
  template <typename ScalarType>
  using OutputVector = RobotStateVector<ScalarType>;
  template <typename ScalarType>
  using InputVector = RobotStateVector<ScalarType>;

  static const int kQueueSize = 1;

  /**
   * The constructor. It takes a rigid body tree as an input parameter to get
   * semantic information about the input values.
   *
   * @param rigid_body_tree The rigid body tree being modeled. This parameter
   * is necessary to understand the meaning of the input data to this system.
   */
  explicit DrakeRosTfPublisher(
      const std::shared_ptr<RigidBodyTree> rigid_body_tree)
      : rigid_body_tree_(rigid_body_tree) {
    // Initializes the time stamp of the previous transmission to be zero.
    previous_send_time_.sec = 0;
    previous_send_time_.nsec = 0;

    // Instantiates a geometry_msgs::TransformStamped message for each rigid
    // body in the rigid body tree.
    for (auto &rigid_body : rigid_body_tree->bodies) {
      // Skips the world link since it by definition does not have a joint.
      if (rigid_body->name() == std::string(RigidBodyTree::kWorldLinkName))
        continue;

      // Skips links that do not have parents.
      if (!rigid_body->hasParent()) continue;

      // Obtains the current link's joint.
      const DrakeJoint &joint = rigid_body->getJoint();

      // Creates a unique key for holding the transform message.
      std::string key = rigid_body->model_name() + rigid_body->name();

      // Checks whether a transform message for the current link was already
      // added to the transform_messages_ map.
      if (transform_messages_.find(key) != transform_messages_.end())
        throw std::runtime_error(
            "ERROR: Multiple model/links named " + key +
            " found when creating a geometry_msgs::TransformStamped message!");

      // Instantiates a geometry_msgs::TransformStamped message for the
      // current rigid body.
      std::unique_ptr<geometry_msgs::TransformStamped> message(
          new geometry_msgs::TransformStamped());

      message->header.frame_id = rigid_body->parent->name();
      message->child_frame_id = rigid_body->name();

      // Initializes the transformation if the joint is fixed.
      // We can do this now since it will not change over time.
      if (joint.getNumPositions() == 0 && joint.getNumVelocities() == 0) {
        auto translation = joint.getTransformToParentBody().translation();
        auto quat = rotmat2quat(joint.getTransformToParentBody().linear());

        message->transform.translation.x = translation(0);
        message->transform.translation.y = translation(1);
        message->transform.translation.z = translation(2);

        message->transform.rotation.w = quat(0);
        message->transform.rotation.x = quat(1);
        message->transform.rotation.y = quat(2);
        message->transform.rotation.z = quat(3);
      }

      transform_messages_[key] = std::move(message);
    }

    // Instantiates a geometry_msgs::TransformStamped message for each frame
    // in the rigid body tree.
    for (auto &frame : rigid_body_tree->frames) {
      std::string key = frame->body->model_name() + frame->name;

      // Checks whether a transform message for the current frame was already
      // added to the transform_messages_ map.
      if (transform_messages_.find(key) != transform_messages_.end())
        throw std::runtime_error(
            "ERROR: Multiple models/frames named " + key +
            " found when creating a geometry_msgs::TransformStamped message!");

      // Instantiates a geometry_msgs::TransformStamped message for the
      // current frame.
      std::unique_ptr<geometry_msgs::TransformStamped> message(
          new geometry_msgs::TransformStamped());

      message->header.frame_id = frame->body->name();
      message->child_frame_id = frame->name;

      // Frames are fixed to a particular rigid body. The following code saves
      // the transformation in the frame's geometry_msgs::TransformStamped
      // message. This can be done once during initialization since it will
      // not change over time.
      auto translation = frame->transform_to_body.translation();
      auto quat = rotmat2quat(frame->transform_to_body.linear());

      message->transform.translation.x = translation(0);
      message->transform.translation.y = translation(1);
      message->transform.translation.z = translation(2);

      message->transform.rotation.w = quat(0);
      message->transform.rotation.x = quat(1);
      message->transform.rotation.y = quat(2);
      message->transform.rotation.z = quat(3);

      transform_messages_[key] = std::move(message);
    }
  }

  StateVector<double> dynamics(const double &t, const StateVector<double> &x,
                               const InputVector<double> &u) const {
    return StateVector<double>();
  }

  OutputVector<double> output(const double &t, const StateVector<double> &x,
                              const InputVector<double> &u) {
    // Checks whether enough time has elapsed since the last transmission.
    // Aborts if insufficient time has passed. This is to prevent flooding ROS
    // topic /tf.
    ros::Time current_time = ros::Time::now();
    if ((current_time - previous_send_time_).toSec() < kMinTransmitPeriod_)
      return u;

    previous_send_time_ = current_time;

    // The input vector u contains the delta model state values.
    // The following code obtains the absolute model state values.
    auto uvec = Drake::toEigen(u);
    auto q = uvec.head(rigid_body_tree_->number_of_positions());
    KinematicsCache<double> cache = rigid_body_tree_->doKinematics(q);

    for (auto &rigid_body : rigid_body_tree_->bodies) {
      // Skips the world link since the world link.
      if (rigid_body->name() == std::string(RigidBodyTree::kWorldLinkName))
        continue;

      // Skips links that do not have parents.
      if (!rigid_body->hasParent()) continue;

      // Obtains the current link's joint.
      const DrakeJoint &joint = rigid_body->getJoint();

      // Obtains a unique key for holding the transform message.
      std::string key = rigid_body->model_name() + rigid_body->name();

      // Verifies that a geometry_msgs::TransformStamped message for the current
      // link exists in the transform_messages_ map.
      auto message_in_map = transform_messages_.find(key);
      if (message_in_map == transform_messages_.end())
        throw std::runtime_error(
            "ERROR: DrakeRosTfPublisher: Unable to find"
            "transform message with key " +
            key);

      // Obtains a pointer to the geometry_msgs::TransformStamped message for
      // the current link.
      geometry_msgs::TransformStamped *message = message_in_map->second.get();

      // Updates the transform only if the joint is not fixed.
      if (joint.getNumPositions() != 0 || joint.getNumVelocities() != 0) {
        auto transform = rigid_body_tree_->relativeTransform(
            cache, rigid_body_tree_->findLinkId(rigid_body->parent->name()),
            rigid_body_tree_->findLinkId(rigid_body->name()));
        auto translation = transform.translation();
        auto quat = rotmat2quat(transform.linear());

        message->transform.translation.x = translation(0);
        message->transform.translation.y = translation(1);
        message->transform.translation.z = translation(2);

        message->transform.rotation.w = quat(0);
        message->transform.rotation.x = quat(1);
        message->transform.rotation.y = quat(2);
        message->transform.rotation.z = quat(3);
      }

      // Updates the time stamp in the transform message.
      message->header.stamp = current_time;

      // Publishes the transform message onto ROS topic /tf.
      tf_broadcaster_.sendTransform(*message);
    }

    // Publishes the transform for each frame in the rigid body tree.
    for (auto &frame : rigid_body_tree_->frames) {
      std::string key = frame->body->model_name() + frame->name;

      // Verifies that a geometry_msgs::TransformStamped message for the current
      // link exists in the transform_messages_ map.
      auto message_in_map = transform_messages_.find(key);
      if (message_in_map == transform_messages_.end())
        throw std::runtime_error(
            "ERROR: DrakeRosTfPublisher: Unable to find"
            "transform message with key " +
            key);

      // Obtains a pointer to the geometry_msgs::TransformStamped message for
      // the current link.
      geometry_msgs::TransformStamped *message = message_in_map->second.get();

      // Updates the message with the latest time stamp. There's no need to
      // update anything else since frames do not move relative to their
      // rigid body.
      message->header.stamp = current_time;

      // Publishes the transform message.
      tf_broadcaster_.sendTransform(*message);
    }

    return u;  // Passes the output through to the next system in the cascade.
  }

  bool isTimeVarying() const { return true; }
  bool isDirectFeedthrough() const { return true; }

 private:
  /**
   * The rigid body tree being used by Drake's rigid body dynamics engine.
   */
  const std::shared_ptr<RigidBodyTree> rigid_body_tree_;

  /**
   * Publishes the transform messages that specify the positions and
   * orientations of every rigid body and frame in the rigid body tree. This is
   * done on ROS topic /tf.
   */
  tf::TransformBroadcaster tf_broadcaster_;

  /**
   * Maintains a set of ROS geometry_msgs::TransformStamped messages, one for
   * each link and frame in the rigid body tree. This is used to avoid having to
   * allocate a new message each time one needs to be sent. The key is the
   * name of the model concatinated with the name of the rigid body or frame.
   */
  std::map<std::string, std::unique_ptr<geometry_msgs::TransformStamped>>
      transform_messages_;

  /**
   * The previous time the transform messages were sent.
   */
  ros::Time previous_send_time_;
};

}  // end namespace plants
}  // end namespace systems
}  // end namespace drake
