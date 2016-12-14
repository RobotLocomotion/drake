#include "drake/systems/ros_tf_publisher.h"

namespace drake {
namespace systems {

using std::make_unique;

RosTfPublisher::RosTfPublisher(const RigidBodyTree<double>& tree)
    : tree_(tree) {
  const int vector_size =
      tree.get_num_positions() + tree.get_num_velocities();
  this->DeclareInputPort(kVectorValued, vector_size);
  LoadEnableParameter();
  Init();
}

void RosTfPublisher::Init() {
  // Initializes the time stamp of the previous transmission to be zero.
  previous_send_time_.sec = 0;
  previous_send_time_.nsec = 0;

  // Instantiates a geometry_msgs::TransformStamped message for each rigid
  // body in the rigid body tree that has a parent body.
  for (const auto& rigid_body : tree_.bodies) {
    if (!rigid_body->has_parent_body()) continue;
    std::string key = GetKey(*rigid_body);

    if (transform_messages_.find(key) != transform_messages_.end()) {
      throw std::runtime_error(
          "ERROR: Duplicate key \"" + key + "\" encountered when creating a "
          "geometry_msgs::TransformStamped message!");
    }

    auto message = make_unique<geometry_msgs::TransformStamped>();
    message->header.frame_id = rigid_body->get_parent()->get_name();
    message->child_frame_id = rigid_body->get_name();

    const DrakeJoint& joint = rigid_body->getJoint();

    // If the joint is fixed, initialize its tansformation.
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
  for (auto const& frame : tree_.frames) {
    std::string key = GetKey(*frame);

    if (transform_messages_.find(key) != transform_messages_.end()) {
      throw std::runtime_error(
          "ERROR: Duplicate key \"" + key + "\" encountered when creating a "
          "geometry_msgs::TransformStamped message!");
    }

    auto message = make_unique<geometry_msgs::TransformStamped>();
    message->header.frame_id = frame->get_rigid_body().get_name();
    message->child_frame_id = frame->get_name();

    // Frames are fixed to a particular RigidBody. The following code saves
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

void RosTfPublisher::DoPublish(const Context<double>& context) const {
  if (!enable_tf_publisher_) return;

  // Aborts if less than kMinTransmitPeriod_ has elapsed since the last
  // transmission to avoid flooding the ROS topic.
  ::ros::Time current_time = ::ros::Time::now();
  if ((current_time - previous_send_time_).toSec() < kMinTransmitPeriod)
    return;
  previous_send_time_ = current_time;

  Eigen::VectorXd u = this->EvalEigenVectorInput(context, 0);
  auto q = u.head(tree_.get_num_positions());
  KinematicsCache<double> cache = tree_.doKinematics(q);

  // Publishes the transforms for the bodies in the tree.
  for (auto const& body : tree_.bodies) {
    if (!body->has_parent_body()) continue;
    std::string key = GetKey(*body);

    // Verifies that a message for the current body exists.
    auto message_in_map = transform_messages_.find(key);
    if (message_in_map == transform_messages_.end()) {
      throw std::runtime_error(
          "ERROR: RosTfPublisher: Unable to find transform message using "
          "key \"" + key + "\".");
    }

    geometry_msgs::TransformStamped* message = message_in_map->second.get();

    // Obtains the current body's joint.
    const DrakeJoint& joint = body->getJoint();

    // Updates the transform if the joint is not fixed.
    if (joint.get_num_positions() != 0 || joint.get_num_velocities() != 0) {
      auto transform = tree_.relativeTransform(
          cache,
          tree_.FindBodyIndex(
              body->get_parent()->get_name()),
          tree_.FindBodyIndex(body->get_name()));
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

    // We use ROS time instead of the Context time since the messages are being
    // consumed by ROS nodes. The two notions of time will be in sync once
    // #4344 is resolved.
    message->header.stamp = current_time;
    tf_broadcaster_.sendTransform(*message);
  }

  // Publishes the transform for each frame in the tree.
  for (auto const& frame : tree_.frames) {
    std::string key = GetKey(*frame);

    // Verifies that a message for the current frame exists.
    auto message_in_map = transform_messages_.find(key);
    if (message_in_map == transform_messages_.end()) {
      throw std::runtime_error(
          "ERROR: RosTfPublisher: Unable to find transform message using "
          "key \"" + key + "\".");
    }

    geometry_msgs::TransformStamped* message = message_in_map->second.get();
    message->header.stamp = current_time;
    tf_broadcaster_.sendTransform(*message);
  }
}

void RosTfPublisher::LoadEnableParameter() {
  const int kMaxNumTries = 10;
  int num_get_attempts{0};
  bool continue_query{true};
  while (continue_query &&
         !::ros::param::get("/drake/enable_tf_publisher",
                            enable_tf_publisher_)) {
    if (++num_get_attempts >= kMaxNumTries) {
      ROS_WARN(
          "Failed to get parameter /drake/enable_tf_publisher. "
          "Assuming publisher is enabled.");
      continue_query = false;
    }
  }

  if (!enable_tf_publisher_) {
    ROS_INFO("RosTfPublisher: Disabling TF publisher!");
  }
}

const std::map<std::string, std::unique_ptr<geometry_msgs::TransformStamped>>&
    RosTfPublisher::get_transform_messages() const {
  return transform_messages_;
}

std::string RosTfPublisher::GetKey(const RigidBody<double>& rigid_body) const {
  return rigid_body.get_model_name() +
      std::to_string(rigid_body.get_model_instance_id()) +
      rigid_body.get_name();
}

std::string RosTfPublisher::GetKey(const RigidBodyFrame<double>& frame) const {
  return frame.get_rigid_body().get_model_name() +
      std::to_string(frame.get_rigid_body().get_model_instance_id()) +
      frame.get_name();
}

}  // namespace systems
}  // namespace drake
