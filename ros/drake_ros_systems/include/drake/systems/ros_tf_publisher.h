#pragma once

#include <map>
#include <memory>
#include <string>

#include "geometry_msgs/TransformStamped.h"
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"

#include "drake/multibody/rigid_body.h"
#include "drake/multibody/rigid_body_frame.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/output_port_value.h"

namespace drake {
namespace systems {

/**
 * Publishes ROS TF messages for visualizing a RigidBodyTree. It is designed to
 * take as input a RigidBodyTree's generalized state, which is typically
 * ouputted by a RigidBodyPlant.
 */
class RosTfPublisher : public LeafSystem<double> {
 public:
  /**
   * Specifies the minimum period in seconds between successive transmissions of
   * of ROS tf transform messages. This is to prevent flooding the tf ROS topic.
   */
  static constexpr double kMinTransmitPeriod{0.01};

  /**
   * Constructs a ROS tf publisher that publishes the poses of the bodies in
   * @p rigid_body_tree.
   */
  explicit RosTfPublisher(const RigidBodyTree<double>& rigid_body_tree);

  /**
   * Takes the current state of the RigidBodyTree and publishes ROS tf messages
   * for the bodies in the RigidBodyTree.
   */
  void DoPublish(const Context<double>& context) const
      override;

  /**
   * This System has no output ports. Thus, DoCalcOutput() does nothing.
   */
  void DoCalcOutput(const Context<double>& context,
                    SystemOutput<double>* output) const
      override {}

  // TODO(liang.fok) Remove this method once we have a proper mock-ROS-topic
  // framework in place.
  /**
   * An accessor to the transform messages that were transmitted.
   */
  const std::map<std::string, std::unique_ptr<geometry_msgs::TransformStamped>>&
      get_transform_messages() const;

 private:
  // Initializes the transform_messages_ table.
  void Init();

  // Queries the ROS parameter server for "/drake/enable_tf_publisher", which is
  // a boolean value that determines whether this class publishes /tf messages.
  void LoadEnableParameter();

  // Returns the key for obtaining the transform message in transform_messages_
  // that is dedicated to hold the pose of @p rigid_body.
  std::string GetKey(const RigidBody<double>& rigid_body) const;

  // Returns the key for obtaining the transform message in transform_messages_
  // that is dedicated to hold the pose of @p frame.
  std::string GetKey(const RigidBodyFrame<double>& frame) const;

  // The RigidBodyTree containing the bodies for whom transforms are being
  // published.
  const RigidBodyTree<double>& tree_;

  // Publishes the transform messages that specify the positions and
  // orientations of every rigid body and frame in the rigid body tree. This is
  // done on ROS topic /tf.
  mutable tf::TransformBroadcaster tf_broadcaster_;

  // A table of ROS geometry_msgs::TransformStamped messages, one for each link
  // and frame in the rigid body tree. This is used to avoid allocating a new
  // message each time for each transmission. The key is the concatination of
  // the model name, model instance ID, and rigid body or frame name.
  std::map<std::string, std::unique_ptr<geometry_msgs::TransformStamped>>
      transform_messages_;

  // The previous time the transform messages were sent.
  mutable ::ros::Time previous_send_time_;

  // Determines whether tf messages should be published.
  bool enable_tf_publisher_{true};
};

}  // namespace systems
}  // namespace drake
