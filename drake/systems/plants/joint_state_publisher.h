#pragma once

#include <Eigen/Dense>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

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
    // Instantiates a ROS node handle, which is necessary to interact with ROS.
    // For more information, see:
    // http://wiki.ros.org/roscpp/Overview/NodeHandles
    ros::NodeHandle nh;

    joint_state_publisher_ = nh.advertise<sensor_msgs::JointState>(
      "joint_states", kQueueSize);

    std::cout << "JointStatePublisher: Initializing joint state message." << std::endl;

    auto rigid_body_tree = rigid_body_system->getRigidBodyTree();
    for (auto & rigid_body : rigid_body_tree->bodies) {
      if (rigid_body->GetName() != "world") {
        std::cout << "JointStatePublisher: Processing link: " << rigid_body->GetName() << std::endl;
        const DrakeJoint & joint = rigid_body->getJoint();

        std::stringstream ss;
        ss << "JointStatePublisher: Link \"" << rigid_body->GetName()
          << "\" has " << joint.getNumPositions() << " positions and "
          << joint.getNumVelocities() << " velocities." << std::endl;
        for (int ii = 0; ii < joint.getNumPositions(); ii++) {
          ss << ii << ": " << joint.getPositionName(ii);
          if (ii < joint.getNumPositions() - 1)
            ss << std::endl;
        }
        std::cout << ss.str() << std::endl;

        // Determines whether the joint is a QuaternionFloatingJoint by
        // checking whether the joint has 7 positions and 6 velocities.
        // If it is, it sets the number of positions to be 6 (x, y, z, r, p, y)
        // instead of 7 (x, y, z, w, z, y, z) since sensor_msgs::JointState
        // message uses Euler angles for the floating joint. It also sets
        // the names of the bank joints to use "roll", "pitch", and "yaw".
        if (joint.getNumPositions() == 7 && joint.getNumVelocities() == 6) {
          joint_state_message_.name.push_back(joint.getName() + "_x");
          joint_state_message_.name.push_back(joint.getName() + "_y");
          joint_state_message_.name.push_back(joint.getName() + "_z");
          joint_state_message_.name.push_back(joint.getName() + "_r");
          joint_state_message_.name.push_back(joint.getName() + "_p");
          joint_state_message_.name.push_back(joint.getName() + "_y");
        } else {
          for (int ii = 0; ii < joint.getNumPositions(); ii++) {
            joint_state_message_.name.push_back(joint.getPositionName(ii));
          }
        }
      }
    }

    std::stringstream ss;
    ss << "Joint names:\n";
    for (int ii = 0; ii < joint_state_message_.name.size(); ii++) {
      ss << ii << ". " << joint_state_message_.name[ii];
      if (ii < joint_state_message_.name.size() - 1)
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

    // To convert from quaternion to RPY, see: drake/util/drakeGeometryUtil.h
    //   Methods: quat2rotmat(), rotmat2rpy()

    // const std::vector<std::shared_ptr<RigidBodySensor>> &sensor_vector =
    //     rigid_body_system_->GetSensors();

    // // This variable is for tracking where in the output of rigid body system
    // // we are currently processing. We initialize it to be the number of states
    // // of the rigid body system to start past the joint state information.
    // size_t output_index = rigid_body_system_->getNumStates();

    // // Iterates through each sensor in the rigid body system. If the sensor is
    // // a LIDAR sensor, store the range measurements in a ROS message and publish
    // // it on the appropriate ROS topic.
    // for (auto &sensor : sensor_vector) {

    //   if (output_index + sensor->getNumOutputs() >
    //       rigid_body_system_->getNumOutputs()) {
    //     std::stringstream buff;
    //     buff << "ERROR: Sum of output index " << output_index
    //          << " and number of outputs of sensor " << sensor->get_name()
    //          << " (" << sensor->getNumOutputs()
    //          << ") exceeds the total number of outputs of the rigid body "
    //             "system ("
    //          << rigid_body_system_->getNumOutputs();
    //     throw std::runtime_error(buff.str());
    //   }

    //   RigidBodyDepthSensor *depth_sensor =
    //       dynamic_cast<RigidBodyDepthSensor *>(sensor.get());

    //   if (depth_sensor != nullptr) {
    //     size_t sensor_data_index_ = output_index;

    //     auto message_in_map = lidar_messages_.find(depth_sensor->get_name());
    //     if (message_in_map == lidar_messages_.end())
    //       throw std::runtime_error(
    //           "Could not find ROS message for LIDAR sensor " +
    //           depth_sensor->get_name());

    //     sensor_msgs::LaserScan *message = message_in_map->second.get();

    //     // Saves the new range measurements in the ROS message.
    //     for (size_t ii = 0; ii < depth_sensor->getNumOutputs(); ii++) {
    //       message->ranges[ii] = u[sensor_data_index_++];
    //     }

    //     // Publishes the ROS message containing the new range measurements.
    //     auto publisher_in_map =
    //         lidar_publishers_.find(depth_sensor->get_name());
    //     if (publisher_in_map == lidar_publishers_.end())
    //       throw std::runtime_error(
    //           "ERROR: Failed to find ROS topic publisher for LIDAR sensor " +
    //           depth_sensor->get_name());

    //     publisher_in_map->second.publish(*message);
    //   }

    //   // Shifts the output index variable forward by one sensor.
    //   output_index += sensor->getNumOutputs();
    // }

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
   */
  sensor_msgs::JointState joint_state_message_;
};

}  // end namespace plants
}  // end namespace systems
}  // end namespace drake
