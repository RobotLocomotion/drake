#pragma once

#include <map>
#include <memory>
#include <string>

#include "geometry_msgs/TransformStamped.h"
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system_output.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace systems {

/**
 * Publishes ROS TF messages for visualizing a RigidBodyTree. It is designed to
 * take as input a RigidBodyTree's generalized state, which is typically
 * ouputted by a RigidBodyPlant.
 *
 * @tparam T The vector element type, which must be a valid Eigen scalar.
 *
 * Instantiated templates for the following scalar types @p T are provided:
 * - double
 */
template <typename T>
class RosTfPublisher : public LeafSystem<T> {
 public:
  // Specifies the minimum period in seconds between successive transmissions of
  // of tf transforms. This is to prevent flooding the tf ROS topic.
  static constexpr double kMinTransmitPeriod_{0.01};

  /**
   * Constructs a ROS tf publisher for a given `RigidBodyTree
   */
  explicit RosTfPublisher(const RigidBodyTree<T>& rigid_body_tree);

  /**
   * Takes the current state of the RigidBodyTree and publishes ROS tf messages
   * for the bodies in the RigidBodyTree.
   */
  void DoPublish(const Context<double>& context) const
      override;

  /**
   * This System has no output ports so EvalOutput() does nothing.
   */
  void EvalOutput(const Context<double>& context,
                  SystemOutput<double>* output) const
      override {}

 private:
  // Initializes the transform_messages_ table.
  void Init();

  // Queries the ROS parameter server for "/drake/enable_tf_publisher", which is
  // a boolean value that determines whether this class publishes /tf messages.
  void LoadEnableParameter();

  // The RigidBodyTree containing the bodies for whom transforms are being
  // published.
  const RigidBodyTree<T>& tree_;

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
