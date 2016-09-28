#pragma once

#include <Eigen/Dense>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

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
namespace systems {

/**
 * Holds the objects and data needed to extract and publish odometry information
 * of a particular model instance.
 */
struct OdometrySensorStruct {
  // The index of the base body of the model instance that this struct is
  // representing.
  int base_body_index;

  // The ROS topic publisher for publishing the LIDAR data.
  ::ros::Publisher publisher;

  // This holds the model instance's odometry information and is periodically
  // published.
  std::unique_ptr<nav_msgs::Odometry> message;
};

/**
 * Takes the system state and publishes odometry information onto ROS topics.
 *
 * It publishes a odometry information for each model within the rigid body
 * system that connected to the world via a floating joint. The odometry
 * messages for each model instance are published onto different
 * ROS topics that are distinguished by model instance name.
 *
 * The resulting system has no internal state; the publish command is throttled
 * by kMinTransmitPeriod_.
 *
 * For convenience, the input is passed directly through as an output. This
 * enables other systems to be cascaded after this system.
 */
template <template <typename> class RobotStateVector>
class RosSensorPublisherOdometry {
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
   * The constructor that creates one ROS topic, publisher, and message for each
   * model instance in @p rigid_body_system.
   *
   * @param[in] rigid_body_system The rigid body system whose output contains
   * the odometry information.
   *
   * @param[in] model_instance_name_table A mapping from model instance IDs to
   * model instance names. The instance names are used to name space the ROS
   * topics on which the `sensor_msgs::JointState` messages are published. This
   * is necessary since multiple model instances may be simultaneously
   * simulated.
   */
  explicit RosSensorPublisherOdometry(
      std::shared_ptr<RigidBodySystem> rigid_body_system,
      const std::map<int, std::string>& model_instance_name_table)
      : rigid_body_system_(rigid_body_system) {
    // Initializes the time stamp of the previous transmission to be zero.
    previous_send_time_.sec = 0;
    previous_send_time_.nsec = 0;

    // Instantiates a ROS node handle through which we can interact with ROS.
    // For more information, see:
    // http://wiki.ros.org/roscpp/Overview/NodeHandles
    ::ros::NodeHandle nh;

    const std::shared_ptr<RigidBodyTree>& tree =
        rigid_body_system->getRigidBodyTree();

    // Obtains a reference to the world link in the rigid body tree.
    const RigidBody& world = tree->world();

    // Creates a ROS topic publisher and a message for each model instance in
    // `model_instance_table`.
    for (const auto& entry : model_instance_name_table) {
      // Obtains the current model instance's ID, instance name, and list of
      // rigid bodies.
      int model_instance_id = entry.first;
      std::string model_instance_name = entry.second;

      // Instantiates a ModelStateStruct for the current model instance.
      std::unique_ptr<OdometrySensorStruct> model_instance_struct(
          new OdometrySensorStruct());

      const std::string topic_name = model_instance_name + "/odometry";
      model_instance_struct->publisher =
          nh.advertise<nav_msgs::Odometry>(topic_name, 1);

      model_instance_struct->message.reset(new nav_msgs::Odometry());
      model_instance_struct->message->header.frame_id =
          RigidBodyTree::kWorldName;

      std::vector<int> base_bodies = tree->FindBaseBodies(model_instance_id);
      if (base_bodies.size() != 1) {
        throw std::runtime_error("RosSensorPublisherOdometry: ERROR: model "
            "instance with ID " + std::to_string(model_instance_id) + " has "
            "multiple base bodies. Not sure which one to use.");
      }

      model_instance_struct->base_body_index = base_bodies.at(0);

      model_instance_struct->message->child_frame_id = model_instance_name + "/"
          + tree->get_body(model_instance_struct->base_body_index).get_name();

      model_instance_list_.push_back(std::move(model_instance_struct));
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

    const std::shared_ptr<RigidBodyTree>& tree =
        rigid_body_system_->getRigidBodyTree();

    // The input vector u contains the entire system's state. The following
    // The following code extracts the position and velocity values from it
    // and computes the kinematic properties of the system.
    auto uvec = drake::toEigen(u);
    auto q = uvec.head(tree->get_num_positions());    // position
    auto v = uvec.segment(tree->get_num_positions(),  // velocity
                          tree->get_num_velocities());
    KinematicsCache<double> cache = tree->doKinematics(q, v);

    // Publishes an odometry message for each model instance in
    // model_instance_list_.
    for (auto& model_instance_struct : model_instance_list_) {
      nav_msgs::Odometry* message = model_instance_struct->message.get();
      const RigidBody& body =
          tree->get_body(model_instance_struct->base_body_index);
      // Updates the odometry information in the odometry message.
      auto transform = tree->relativeTransform(
          cache,
          RigidBodyTree::kWorldBodyIndex,
          model_instance_struct->base_body_index);
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
      auto twist = tree->relativeTwist(
          cache,
          RigidBodyTree::kWorldBodyIndex,
          model_instance_struct->base_body_index,
          RigidBodyTree::kWorldBodyIndex);

      message->twist.twist.linear.x = twist(0);
      message->twist.twist.linear.y = twist(1);
      message->twist.twist.linear.z = twist(2);

      message->twist.twist.angular.x = twist(3);
      message->twist.twist.angular.y = twist(4);
      message->twist.twist.angular.z = twist(5);

      // Updates the time stamp in the transform message.
      message->header.stamp = current_time;

      model_instance_struct->publisher.publish(*message);
    }

    return u;  // Passes the output through to the next system in the cascade.
  }

  bool isTimeVarying() const { return true; }
  bool isDirectFeedthrough() const { return true; }

 private:
  std::shared_ptr<RigidBodySystem> rigid_body_system_;

  // A set of ModelStateStruct structs, one for each model instance.
  std::vector<std::unique_ptr<OdometrySensorStruct>> model_instance_list_;

  // The previous time the odometry messages were sent.
  ::ros::Time previous_send_time_;
};

}  // namespace systems
}  // namespace ros
}  // namespace drake
