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
      : rigid_body_system_(rigid_body_system),
        joint_state_message_(new sensor_msgs::JointState()) {
    // Instantiates a ROS node handle, which is necessary to interact with ROS.
    // For more information, see:
    // http://wiki.ros.org/roscpp/Overview/NodeHandles
    ros::NodeHandle nh;

    joint_state_publisher_ = nh.advertise<sensor_msgs::JointState>(
      "joint_states", kQueueSize);

    std::cout << "JointStatePublisher: Initializing joint state message." << std::endl;

    auto rigid_body_tree = rigid_body_system->getRigidBodyTree();
    for (auto & rigid_body : rigid_body_tree->bodies) {

      // Skips the world link since the world link does not have a joint.
      if (rigid_body->GetName() == "world") continue;

      const DrakeJoint & joint = rigid_body->getJoint();

      // Skips fixed joints.
      if (joint.getNumPositions() == 0 && joint.getNumVelocities() == 0)
        continue;

      // Identifies whether the joint is a base joint. Base joint states are
      // conveyed to ROS using a dedicated tf::TransformBroadcaster. Non-base
      // joint states are transmitted via a sensor_msgs::JointState message.
      if (joint.getName() != "base") {
        for (int ii = 0; ii < joint.getNumPositions(); ii++) {
          joint_state_message_->name.push_back(joint.getPositionName(ii));
        }
      } else {

        // The following if statement verifies that an odometry message for the
        // current model has not already been added to the odometry_message_
        // map.
        if (odometry_messages_.find(rigid_body->GetModelName()) ==
          odometry_messages_.end()) {

          // Instantiates a geometry_msgs::TransformStamped message for the
          // current model.

          std::unique_ptr<geometry_msgs::TransformStamped> message(
            new geometry_msgs::TransformStamped());

          // TODO: Initialize the message.
          message->header.frame_id = "world";
          message->child_frame_id = rigid_body->GetName();

          odometry_messages_[rigid_body->GetModelName()] = std::move(message);
        } else {
          throw std::runtime_error(
              "ERROR: Multiple models named " + rigid_body->GetModelName() +
              " found when creating a geometry_msgs::TransformStamped message!");
        }

        // // std::stringstream ss;
        // // ss << "JointStatePublisher: Link \"" << rigid_body->GetName()
        // //   << "\" has " << joint.getNumPositions() << " positions and "
        // //   << joint.getNumVelocities() << " velocities." << std::endl;
        // // for (int ii = 0; ii < joint.getNumPositions(); ii++) {
        // //   ss << ii << ": " << joint.getPositionName(ii);
        // //   if (ii < joint.getNumPositions() - 1)
        // //     ss << std::endl;
        // // }
        // // std::cout << ss.str() << std::endl;

        // // Determines whether the joint is a QuaternionFloatingJoint by
        // // checking if the joint has 7 positions and 6 velocities.
        // // If it is, the following code sets the number of positions to be 6
        // // (x, y, z, r, p, y) instead of 7 (x, y, z, w, z, y, z) since
        // // sensor_msgs::JointState messages uses Euler angles for the floating
        // // joint. It also sets the names of the bank joints to use "roll",
        // // "pitch", and "yaw".
        // if (joint.getNumPositions() == 7 && joint.getNumVelocities() == 6) {
        //   joint_state_message_.name.push_back(joint.getName() + "_x");
        //   joint_state_message_.name.push_back(joint.getName() + "_y");
        //   joint_state_message_.name.push_back(joint.getName() + "_z");
        //   joint_state_message_.name.push_back(joint.getName() + "_r");
        //   joint_state_message_.name.push_back(joint.getName() + "_p");
        //   joint_state_message_.name.push_back(joint.getName() + "_y");
        // } else {
        //   for (int ii = 0; ii < joint.getNumPositions(); ii++) {
        //     joint_state_message_.name.push_back(joint.getPositionName(ii));
        //   }
        // }
      }
    }

    joint_state_message_->position.resize(joint_state_message_->name.size());
    joint_state_message_->velocity.resize(joint_state_message_->name.size());
    joint_state_message_->effort.resize(joint_state_message_->name.size());

    std::stringstream ss;
    ss << "Joint names:\n";
    for (int ii = 0; ii < joint_state_message_->name.size(); ii++) {
      ss << ii << ". " << joint_state_message_->name[ii];
      if (ii < joint_state_message_->name.size() - 1)
        ss << "\n";
    }
    std::cout << ss.str() << std::endl;
  }

  StateVector<double> dynamics(const double &t, const StateVector<double> &x,
                               const InputVector<double> &u) const {
    return StateVector<double>();
  }

  OutputVector<double> output(const double &t, const StateVector<double> &x,
                              const InputVector<double> &u) const {

    // Remembers where in the joint state message we are currently saving state.
    size_t joint_state_message_index = 0;

    // Remembers where in the input vector we are currently obtaining state.
    size_t input_vector_index = 0;

    // Saves the joint positions.
    auto rigid_body_tree = rigid_body_system_->getRigidBodyTree();
    for (auto & rigid_body : rigid_body_tree->bodies) {

      // Skips the world link since it does not have a joint.
      if (rigid_body->GetName() == "world") continue;

      const DrakeJoint & joint = rigid_body->getJoint();

      // Skips fixed joints.
      if (joint.getNumPositions() == 0 && joint.getNumVelocities() == 0)
        continue;

      if (joint.getName() != "base") {
        for (int ii = 0; ii < joint.getNumPositions(); ii++) {
          joint_state_message_->position[joint_state_message_index] =
            u[input_vector_index];
          joint_state_message_->velocity[joint_state_message_index] =
            u[rigid_body_tree->num_positions + input_vector_index];

          joint_state_message_index++;
          input_vector_index++;
        }
      } else {
        if (joint.getNumPositions() == 7 && joint.getNumVelocities() == 6) {

          // Searches for and obtains the odometry message for the current
          // base joint.
          auto message_in_map = odometry_messages_.find(rigid_body->GetModelName());
          if (message_in_map == odometry_messages_.end())
            throw std::runtime_error(
              "ERROR: JointStatePublisher::output: Could not find Odometry "
              "ROS message for rigid body \"" + rigid_body->GetModelName() +
              "\".");

          geometry_msgs::TransformStamped* message = message_in_map->second.get();

          std::cout << "Drake: JointStatePublisher: Publishing world-to-"
            << rigid_body->GetName() << " transform...\n"
            << "  - input vector: " << u.transpose() << "\n"
            << "  - input_vector_index: " << input_vector_index
            << std::endl;

          // Updates the message with the latest joint state.
          message->header.stamp = ros::Time::now();

          message->transform.translation.x = u[input_vector_index++];
          message->transform.translation.y = u[input_vector_index++];
          message->transform.translation.z = u[input_vector_index++];

          message->transform.rotation.w = u[input_vector_index++];
          message->transform.rotation.x = u[input_vector_index++];
          message->transform.rotation.y = u[input_vector_index++];
          message->transform.rotation.z = u[input_vector_index++];

          std::cout
            << "  - translation.x = " << message->transform.translation.x << "\n"
            << "  - translation.y = " << message->transform.translation.y << "\n"
            << "  - translation.z = " << message->transform.translation.z << "\n"
            << "  - rotation.w = " << message->transform.rotation.w << "\n"
            << "  - rotation.x = " << message->transform.rotation.x << "\n"
            << "  - rotation.y = " << message->transform.rotation.y << "\n"
            << "  - rotation.z = " << message->transform.rotation.z
            << std::endl;

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

        } else {
          std::stringstream error_msg;
          error_msg << "ERROR: JointStatePublisher::output: "
            << "Unable to handle non-quaternion base joints." << std::endl
            << "  - joint name: " << joint.getName() << std::endl
            << "  - # position DOFs: " << joint.getNumPositions() << std::endl
            << "  - # velocity DOFs: " << joint.getNumVelocities() << std::endl;
          throw std::runtime_error(error_msg.str());
        }
      }
    }

    // std::cout << "Drake: JointStatePublisher: Publishingjoint state message..." << std::endl;
    joint_state_publisher_.publish(*joint_state_message_.get());
    return u;  // Passes the output through to the next system in the cascade.
  }

  bool isTimeVarying() const { return true; }
  bool isDirectFeedthrough() const { return true; }

 private:
  std::shared_ptr<RigidBodySystem> rigid_body_system_;

  /**
   * The ROS topic publisher for publishing joint state messages.
   */
  ros::Publisher joint_state_publisher_;

  /**
   * The joint state message to publish. This is used to avoid having to
   * allocate a new message each time one needs to be sent.
   *
   * The std::unique_ptr indirection is needed to be able to modify
   * the joint state messag within method output(), which is const.
   */
  std::unique_ptr<sensor_msgs::JointState> joint_state_message_;

  /**
   * Publishes the transform messages that specify the position of the model's
   * root node and the world.
   */
  tf::TransformBroadcaster tf_broadcaster_;

  /**
   * Maintains a set of ROS geometry_msgs::TransformStamped messages, one for
   * each model in the world. This is used to avoid having to allocate a new
   * message each time one needs to be sent. The key is the name of the model.
   */
  std::map<std::string, std::unique_ptr<geometry_msgs::TransformStamped>>
      odometry_messages_;
};

}  // end namespace plants
}  // end namespace systems
}  // end namespace drake
