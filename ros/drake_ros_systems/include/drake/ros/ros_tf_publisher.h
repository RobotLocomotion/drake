#pragma once

#include <Eigen/Dense>

#include "ros/ros.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_broadcaster.h"

#include "drake/math/rotation_matrix.h"
#include "drake/ros/parameter_server.h"
#include "drake/systems/System.h"
#include "drake/systems/plants/KinematicsCache.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/systems/vector.h"


using drake::NullVector;
using drake::RigidBodySensor;
using drake::RigidBodySystem;
using drake::RigidBodyDepthSensor;

namespace drake {
namespace ros {

/**
 * Publishes transforms for every rigid body in a `RigidBodyTree`. The transform
 * names are prefixed by the model instance names.
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
   * It checks the ROS parameter server for a boolean parameter called
   * "enable_tf_publisher". If this parameter exists and is false, this
   * class disables itself. Otherwise, this class is enabled.
   *
   * @param[in] rigid_body_tree Contains the rigid bodies whose transforms are
   * to be published. This reference must remain valid for the lifetime of this
   * object.
   *
   * @param[in] model_instance_names A mapping from model instance IDs to model
   * instance names. These names are used to prefix the transform names, which
   * is necessary for RViz to simultaneously visualize multiple robots. This
   * reference must remain valid for the lifetime of this object.
   */
  explicit DrakeRosTfPublisher(
      const std::shared_ptr<RigidBodyTree> rigid_body_tree,
      const std::map<int, std::string>& model_instance_names) :
          rigid_body_tree_(rigid_body_tree),
          model_instance_names_(model_instance_names) {
    ::ros::NodeHandle node_handle;
    // Queries the ROS parameter server for a boolean parameter called
    // "enable_tf_publisher". This parameter is used to control whether
    // this class publishes /tf messages.
    // std::string parameter_name("enable_tf_publisher");
    std::string parameter_name("enable_tf_publisher");
    enable_tf_publisher_ = GetROSParameter<bool>(node_handle, parameter_name);

    if (enable_tf_publisher_) {
      ROS_INFO("Enabling TF publisher!");
    } else {
      ROS_INFO("Disabling TF publisher!");
    }

    // Initializes the time stamp of the previous transmission to be zero.
    previous_send_time_.sec = 0;
    previous_send_time_.nsec = 0;

    // Instantiates a geometry_msgs::TransformStamped message for each rigid
    // body in the rigid body tree.
    for (auto const& rigid_body : rigid_body_tree->bodies) {
      // Skips the current rigid body if it should be skipped.
      if (!ShouldPublishTfForRigidBody(rigid_body.get())) continue;

      // Derives the key for storing the geometry_msgs::TransformStamped for
      // the current rigid body in transform_messages_.
      std::string key;
      DeriveKey(rigid_body, &key);

      // Checks whether a transform message for the current link was already
      // added to the transform_messages_ map.
      if (transform_messages_.find(key) != transform_messages_.end()) {
        throw std::runtime_error(
            "ERROR: Duplicate key \"" + key + " encountered when creating a "
            "geometry_msgs::TransformStamped message for rigid body \"" +
            rigid_body->get_name() + "\".");
      }

      // Instantiates a geometry_msgs::TransformStamped message for the
      // current rigid body.
      std::unique_ptr<geometry_msgs::TransformStamped> message(
          new geometry_msgs::TransformStamped());

      // Determines the name of this rigid body tree's parent's frame in the
      // /tf tree.
      std::string parent_frame_name;
      DeriveTfFrameName(*(rigid_body->get_parent()), model_instance_names,
          &parent_frame_name);
      message->header.frame_id = parent_frame_name;

      // Determines the name of this rigid body's frame in the /tf tree.
      std::string frame_name;
      DeriveTfFrameName(*(rigid_body.get()), model_instance_names,
          &frame_name);
      message->child_frame_id = frame_name;

      // Obtains the current link's joint.
      const DrakeJoint& joint = rigid_body->getJoint();

      // Initializes the transformation if the joint is fixed.
      // We can do this now since it will not change over time.
      if (joint.getNumPositions() == 0 && joint.getNumVelocities() == 0) {
        auto translation = joint.getTransformToParentBody().translation();
        auto quat =
            drake::math::rotmat2quat(joint.getTransformToParentBody().linear());

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

      // Derives the key for storing the geometry_msgs::TransformStamped for
      // the current frame in transform_messages_.
      std::string key;
      DeriveKey(frame, &key);

      // Checks whether a transform message for the current frame was already
      // added to the transform_messages_ map.
      if (transform_messages_.find(key) != transform_messages_.end()) {
        throw std::runtime_error(
            "ERROR: Duplicate key \"" + key + "\" encountered when creating a "
            "geometry_msgs::TransformStamped message.");
      }

      // Instantiates a geometry_msgs::TransformStamped message for the
      // current frame.
      std::unique_ptr<geometry_msgs::TransformStamped> message(
          new geometry_msgs::TransformStamped());

      // Determines the name of this rigid body tree's parent's frame in the
      // /tf tree.
      std::string parent_frame_name;
      DeriveTfFrameName(frame->get_rigid_body(), model_instance_names,
          &parent_frame_name);
      message->header.frame_id = parent_frame_name;

      // Determines the name of this rigid body's frame in the /tf tree.
      std::string frame_name;
      DeriveTfFrameName(*(frame.get()), model_instance_names,
          &frame_name);
      message->child_frame_id = frame_name;

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
    // Aborts publishing tf messages if enable_tf_publisher_ is false.
    if (!enable_tf_publisher_) return u;

    // Aborts if insufficient time has passed since the last transmission. This
    // is to avoid flooding the ROS topics.
    ::ros::Time current_time = ::ros::Time::now();
    if ((current_time - previous_send_time_).toSec() < kMinTransmitPeriod_)
      return u;

    // Updates the previous send time.
    previous_send_time_ = current_time;

    // The input vector u contains the entire system's state.
    // The following code extracts the position values from it
    // and computes the kinematic properties of the system.
    auto uvec = drake::toEigen(u);
    auto q = uvec.head(rigid_body_tree_->number_of_positions());
    KinematicsCache<double> cache = rigid_body_tree_->doKinematics(q);

    // Publishes the transform for each rigid body in the rigid body tree.
    for (auto const& rigid_body : rigid_body_tree_->bodies) {
      // Skips the current rigid body if it should be skipped.
      if (!ShouldPublishTfForRigidBody(rigid_body.get())) continue;

      std::string key;
      DeriveKey(rigid_body, &key);

      // Verifies that a geometry_msgs::TransformStamped message for the current
      // link exists in the transform_messages_ map.
      auto message_in_map = transform_messages_.find(key);
      if (message_in_map == transform_messages_.end()) {
        throw std::runtime_error(
            "ERROR: DrakeRosTfPublisher: Unable to obtain transform message "
            "for rigid body \"" + rigid_body->get_name() + "\" using key \"" +
            key + "\"");
      }

      // Obtains a pointer to the geometry_msgs::TransformStamped message for
      // the current link.
      geometry_msgs::TransformStamped* message = message_in_map->second.get();

      // Obtains the current link's joint.
      const DrakeJoint& joint = rigid_body->getJoint();

      // Updates the transform only if the joint is not fixed.
      if (joint.getNumPositions() != 0 || joint.getNumVelocities() != 0) {
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
      std::string key;
      DeriveKey(frame, &key);

      // Verifies that a geometry_msgs::TransformStamped message for the current
      // link exists in the transform_messages_ map.
      auto message_in_map = transform_messages_.find(key);
      if (message_in_map == transform_messages_.end()) {
        throw std::runtime_error(
            "ERROR: DrakeRosTfPublisher: Unable to obtain transform message "
            "for frame \"" + frame->get_name() + "\" using key \"" +
            key + "\"");
      }

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
  // Determines whether a transform should be published for @p rigid_body. A
  // rigid body should be skipped if it is the world link or does not have a
  // parent link.
  bool ShouldPublishTfForRigidBody(const RigidBody* rigid_body) {
    // Skips parent-less links. This includes the world.
    return rigid_body->hasParent();
  }

  // Derives a key for obtaining the transform message for @p rigid_body
  // from transform_messages_. The key is a concatenation of the string
  // "Rigidbody_", the rigid body's name, and the ID of the model instance to
  // which the rigid body belongs.
  void DeriveKey(const std::unique_ptr<RigidBody>& rigid_body,
      std::string* key) {
    *key = "RigidBody_" + rigid_body->get_name() +
        std::to_string(rigid_body->get_model_instance_id());
  }

  // Derives a key for obtaining the transform message for @p frame
  // from transform_messages_. The key is a concatenation of the string
  // "Frame_", the frame's name, and the ID of the model instance to which the
  // frame belongs.
  void DeriveKey(const std::shared_ptr<RigidBodyFrame>& frame,
      std::string* key) {
    *key = "Frame_" + frame->get_name() +
        std::to_string(frame->get_model_instance_id());
  }

  // Determines the name of the frame belonging to @p rigid_body in the /tf
  // tree. If @p rigid_body is the world, use the world's name. Otherwise,
  // prefix the name of @p rigid_body with its model instance name. This is
  // necessary for the /tf tree to support multiple models.
  //
  // @param[in] rigid_body The rigid body whose /tf tree frame name is being
  // derived.
  //
  // @param[in] model_instance_name A mapping from model instance IDs to model
  // instance names. The instance names are used to prefix the frame name so
  // that multiple models can be supported.
  //
  // @param[out] frame_name A pointer to where the frame name should be stored.
  void DeriveTfFrameName(const RigidBody& rigid_body,
      const std::map<int, std::string>& model_instance_names,
      std::string* frame_name) {

    // Obtains the rigid body's name.
    std::string name = rigid_body.get_name();

    if (name == RigidBodyTree::kWorldLinkName) {
      // If the rigid body is the world, just use the world's name. Since there
      // is only one world, there is no need for a prefix.
       *frame_name = name;
    } else {
      // Obtains the rigid body's model instance ID.
      int model_instance_id = rigid_body.get_model_instance_id();

      // Verifies that the model instance ID has a model instance name.
      // Throws an exception if it does not have an instance name.
      if (model_instance_names.find(model_instance_id) ==
          model_instance_names.end()) {
        throw std::runtime_error(
            "ERROR: DrakeRosTfPublisher: Model instance with ID " +
            std::to_string(model_instance_id) + " does not have a name.");
      }

      // Prefixes the rigid body's name with the model instance name.
      // This is the name of the rigid body's frame in the /tf tree.
      *frame_name =
          model_instance_names.at(model_instance_id) + "/" + name;
    }
  }

  // Determines the name of the frame belonging to @p frame in the /tf
  // tree. This is done by prefixing the name of @p frame with its model
  // instance name. A prefix is necessary for the /tf tree to support multiple
  // models.
  //
  // @param[in] frame The rigid body frame whose /tf tree frame name is being
  // derived.
  //
  // @param[in] model_instance_name A mapping from model instance IDs to model
  // instance names. The instance names are used to prefix the frame name so
  // that multiple models can be supported.
  //
  // @param[out] frame_name A pointer to where the frame name should be stored.
  void DeriveTfFrameName(const RigidBodyFrame& frame,
      const std::map<int, std::string>& model_instance_names,
      std::string* frame_name) {

    // Obtains the frame's name.
    std::string name = frame.get_name();

    // Obtains the frame's model instance ID.
    int model_instance_id = frame.get_model_instance_id();

    // Verifies that the model instance ID has a model instance name.
    // Throws an exception if it does not have an instance name.
    if (model_instance_names.find(model_instance_id) ==
        model_instance_names.end()) {
      throw std::runtime_error(
          "ERROR: DrakeRosTfPublisher: Model instance with ID " +
          std::to_string(model_instance_id) + " does not have a name.");
    }

    // Prefixes the rigid body frame's name with the model instance name.
    // This is the name of the rigid body's frame in the /tf tree.
    *frame_name =
        model_instance_names.at(model_instance_id) + "/" + name;
  }

  // The rigid body tree being used by Drake's rigid body dynamics engine.
  const std::shared_ptr<RigidBodyTree> rigid_body_tree_;

  // Maps model instance IDs to model instance names. This is used to prefix
  // the transform names.
  const std::map<int, std::string>& model_instance_names_;

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
  bool enable_tf_publisher_{true};
};

}  // end namespace ros
}  // end namespace drake
