#pragma once

#include <map>
#include <string>

#include <Eigen/Dense>

#include "ros/ros.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_broadcaster.h"

#include "drake/math/rotation_matrix.h"
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
  // Specifies the minimum period in seconds between successive transmissions of
  // of tf transforms. This is to prevent flooding the tf ROS topic.
  static constexpr double kMinTransmitPeriod_ = 0.01;

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
      : rigid_body_tree_(rigid_body_tree), enable_tf_publisher_(true) {
    // Queries the ROS parameter server for a boolean parameter in
    // "/drake/enable_tf_publisher". This parameter is used to control whether
    // this class publishes /tf messages.
    {
      int num_get_attempts = 0;
      bool continue_query = true;
      while (continue_query &&
             !::ros::param::get("/drake/enable_tf_publisher",
                                enable_tf_publisher_)) {
        if (++num_get_attempts > 10) {
          ROS_WARN(
              "Failed to get parameter /drake/enable_tf_publisher. "
              "Assuming publisher is enabled.");
          continue_query = false;
        }
      }

      if (enable_tf_publisher_) {
        ROS_INFO("Enabling TF publisher!");
      } else {
        ROS_INFO("Disabling TF publisher!");
      }
    }

    // Initializes the time stamp of the previous transmission to be zero.
    previous_send_time_.sec = 0;
    previous_send_time_.nsec = 0;

    // Instantiates a geometry_msgs::TransformStamped message for each rigid
    // body in the rigid body tree.
    for (auto const& rigid_body : rigid_body_tree->bodies) {
      // Skips the current rigid body if it should be skipped.
      if (!PublishTfForRigidBody(rigid_body.get())) continue;

      // Creates a unique key for holding the transform message.
      std::string key = rigid_body->get_model_name() + rigid_body->get_name();

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

      message->header.frame_id = rigid_body->get_parent()->get_name();
      message->child_frame_id = rigid_body->get_name();

      // Obtains the current link's joint.
      const DrakeJoint& joint = rigid_body->getJoint();

      // Initializes the transformation if the joint is fixed.
      // We can do this now since it will not change over time.
      if (joint.get_num_positions() == 0 && joint.get_num_velocities() == 0) {
        auto translation = joint.get_transform_to_parent_body().translation();
        auto quat = drake::math::rotmat2quat(
            joint.get_transform_to_parent_body().linear());

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
    for (auto const& frame : rigid_body_tree->frames) {
      std::string key = frame->get_rigid_body().get_model_name() +
          frame->get_name();

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

      message->header.frame_id = frame->get_rigid_body().get_name();
      message->child_frame_id = frame->get_name();

      // Frames are fixed to a particular rigid body. The following code saves
      // the transformation in the frame's geometry_msgs::TransformStamped
      // message. This can be done once during initialization since it will
      // not change over time.
      auto translation = frame->get_transform_to_body().translation();
      auto quat = drake::math::rotmat2quat(
          frame->get_transform_to_body().linear());

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

    // Aborts publishing tf messages if enable_tf_publisher_ is true.
    if (!enable_tf_publisher_) return u;

    // The input vector u contains the entire system's state.
    // The following code extracts the position values from it
    // and computes the kinematic properties of the system.
    auto uvec = drake::toEigen(u);
    auto q = uvec.head(rigid_body_tree_->get_num_positions());
    KinematicsCache<double> cache = rigid_body_tree_->doKinematics(q);

    // Publishes the transform for each rigid body in the rigid body tree.
    for (auto const& rigid_body : rigid_body_tree_->bodies) {
      // Skips the current rigid body if it should be skipped.
      if (!PublishTfForRigidBody(rigid_body.get())) continue;

      // Obtains a unique key for holding the transform message.
      std::string key = rigid_body->get_model_name() + rigid_body->get_name();

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
      geometry_msgs::TransformStamped* message = message_in_map->second.get();

      // Obtains the current link's joint.
      const DrakeJoint& joint = rigid_body->getJoint();

      // Updates the transform only if the joint is not fixed.
      if (joint.get_num_positions() != 0 || joint.get_num_velocities() != 0) {
        auto transform = rigid_body_tree_->relativeTransform(
            cache,
            rigid_body_tree_->FindBodyIndex(
                rigid_body->get_parent()->get_name()),
            rigid_body_tree_->FindBodyIndex(rigid_body->get_name()));
        auto translation = transform.translation();
        auto quat = drake::math::rotmat2quat(transform.linear());

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
    for (auto const& frame : rigid_body_tree_->frames) {
      std::string key = frame->get_rigid_body().get_model_name() +
          frame->get_name();

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
      geometry_msgs::TransformStamped* message = message_in_map->second.get();

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
  // Determines whether a transform should be published for the specified
  // rigid body. A rigid body should be skipped if it is the world link or if
  // it is connected to the world via a fixed joint.
  bool PublishTfForRigidBody(const RigidBody* rigid_body) {
    // Skips rigid bodies without a mobilizer joint. This includes the RigidBody
    // that represents the world.
    if (!rigid_body->has_parent_body()) return false;
    return true;
  }

  // The rigid body tree being used by Drake's rigid body dynamics engine.
  const std::shared_ptr<RigidBodyTree> rigid_body_tree_;

  // Publishes the transform messages that specify the positions and
  // orientations of every rigid body and frame in the rigid body tree. This is
  // done on ROS topic /tf.
  tf::TransformBroadcaster tf_broadcaster_;

  // Maintains a set of ROS geometry_msgs::TransformStamped messages, one for
  // each link and frame in the rigid body tree. This is used to avoid having to
  // allocate a new message each time one needs to be sent. The key is the
  // name of the model concatinated with the name of the rigid body or frame.
  std::map<std::string, std::unique_ptr<geometry_msgs::TransformStamped>>
      transform_messages_;

  // The previous time the transform messages were sent.
  ::ros::Time previous_send_time_;

  // Determines whether tf messages should be published.
  bool enable_tf_publisher_;
};

}  // namespace systems
}  // namespace ros
}  // namespace drake
