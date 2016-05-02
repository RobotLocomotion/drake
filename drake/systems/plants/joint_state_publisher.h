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

/** JointStatePublisher<RobotStateVector>
 * @brief A system that takes the system state as the input and then publishes
 * sensor_msgs::JointState messages on ROS topic "joint_states".
 *
 * @concept{system_concept}
 *
 * The resulting system has no internal state; the publish command is executed
 * on every call to the output method.
 *
 * For convenience, the input is passed directly through as an output.
 */

template <template <typename> class RobotStateVector>
class JointStatePublisher {
 public:
  template <typename ScalarType>
  using StateVector = NullVector<ScalarType>;
  template <typename ScalarType>
  using OutputVector = RobotStateVector<ScalarType>;
  template <typename ScalarType>
  using InputVector = RobotStateVector<ScalarType>;

  static const int kQueueSize = 1;

  /**
   * The constructor. It takes a rigid body system as an input parameter to get
   * semantic information about the output from the rigid body system.
   *
   * Ideally, the output of the rigid body system should be self-descriptive.
   * See: https://github.com/RobotLocomotion/drake/issues/2152
   */
  JointStatePublisher(std::shared_ptr<RigidBodySystem> rigid_body_system)
      : rigid_body_system_(rigid_body_system) {

    auto rigid_body_tree = rigid_body_system->getRigidBodyTree();
    for (auto & rigid_body : rigid_body_tree->bodies) {

      // Skips the world link since the world link does not have a joint.
      if (rigid_body->GetName() == "world") continue;

      // Skips links that do not have parents.
      if (!rigid_body->hasParent()) continue;

      // Obtains the current link's joint.
      const DrakeJoint & joint = rigid_body->getJoint();

      // Skips links that are attached to the rigid body tree via fixed joints.
      if (joint.getNumPositions() == 0 && joint.getNumVelocities() == 0)
        continue;

      // Creates a unique key for holding the transform message.
      std::string key = rigid_body->GetModelName() + rigid_body->GetName();

      // Verifies that an odometry message for the current link has not already
      // been added to the transform_messages_ map.
      if (transform_messages_.find(key) == transform_messages_.end()) {

        // Instantiates a geometry_msgs::TransformStamped message for the
        // current model.
        std::unique_ptr<geometry_msgs::TransformStamped> message(
          new geometry_msgs::TransformStamped());

        message->header.frame_id = rigid_body->parent->GetName();
        message->child_frame_id = rigid_body->GetName();

        transform_messages_[key] = std::move(message);
      } else {
        throw std::runtime_error(
            "ERROR: Multiple models named " + rigid_body->GetModelName() +
            " found when creating a geometry_msgs::TransformStamped message!");
      }
    }
  }

  StateVector<double> dynamics(const double &t, const StateVector<double> &x,
                               const InputVector<double> &u) const {
    return StateVector<double>();
  }

  OutputVector<double> output(const double &t, const StateVector<double> &x,
                              const InputVector<double> &u) const {

    auto rigid_body_tree = rigid_body_system_->getRigidBodyTree();

    // The input vector, u, contains the delta model state values.
    // The following code obtains the absolute values.
    auto uvec = Drake::toEigen(u);
    auto q = uvec.head(rigid_body_tree->num_positions);
    KinematicsCache<double> cache = rigid_body_tree->doKinematics(q);

    for (auto & rigid_body : rigid_body_tree->bodies) {

      // Skips the world link since the world link.
      if (rigid_body->GetName() == "world") continue;

      // Skips links that do not have parents.
      if (!rigid_body->hasParent()) continue;

      // Obtains the current link's joint.
      const DrakeJoint & joint = rigid_body->getJoint();

      // Skips links that are attached to the rigid body tree via fixed joints.
      if (joint.getNumPositions() == 0 && joint.getNumVelocities() == 0)
        continue;

      // Obtains a unique key for holding the transform message.
      std::string key = rigid_body->GetModelName() + rigid_body->GetName();

      // Verifies that an odometry message for the current link has not already
      // been added to the transform_messages_ map.
      auto message_in_map = transform_messages_.find(key);
      if (message_in_map == transform_messages_.end())
        throw std::runtime_error("ERROR: JointStatePublisher: Unable to find"
          "transform message with key " + key);

      geometry_msgs::TransformStamped* message = message_in_map->second.get();

      auto transform = rigid_body_tree->relativeTransform(cache,
        rigid_body_tree->findLinkId(rigid_body->parent->GetName()),
        rigid_body_tree->findLinkId(rigid_body->GetName()));
      auto translation = transform.translation();
      auto quat = rotmat2quat(transform.linear());

      // Updates the message with the latest joint state.
      message->header.stamp = ros::Time::now();

      message->transform.translation.x = translation(0);
      message->transform.translation.y = translation(1);
      message->transform.translation.z = translation(2);

      message->transform.rotation.w = quat(0);
      message->transform.rotation.x = quat(1);
      message->transform.rotation.y = quat(2);
      message->transform.rotation.z = quat(3);

      // Publishes the newly updated odometry message.
      //
      // Note that we need to const_cast<...> a pointer to the transform
      // broadcaster because tf::TransformBroadcaster::sendTransform()
      // is not const.
      //
      // Otherwise, we would be able to execute:
      //
      //   tf_broadcaster_.sendTransform(*message);
      //
      // The workaround being used is described here:
      //
      //  http://stackoverflow.com/questions/8325400/how-to-call-a-non-const-method-from-a-const-method
      const_cast<tf::TransformBroadcaster*>(&tf_broadcaster_)->sendTransform(*message);
    }

    return u;  // Passes the output through to the next system in the cascade.
  }

  bool isTimeVarying() const { return true; }
  bool isDirectFeedthrough() const { return true; }

 private:
  std::shared_ptr<RigidBodySystem> rigid_body_system_;

  /**
   * Publishes the transform messages that specify the position of the model's
   * root node and the world.
   */
  tf::TransformBroadcaster tf_broadcaster_;

  /**
   * Maintains a set of ROS geometry_msgs::TransformStamped messages, one for
   * each link in the rigid body system. This is used to avoid having to
   * allocate a new message each time one needs to be sent. The key is the
   * name of the link.
   */
  std::map<std::string, std::unique_ptr<geometry_msgs::TransformStamped>>
      transform_messages_;
};

}  // end namespace plants
}  // end namespace systems
}  // end namespace drake
