#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

namespace drake {
namespace examples {
namespace cars {
namespace {

class PriusToGolfcartTFConverter {
 public:
  PriusToGolfcartTFConverter() :
      start_time_(ros::Time::now()) {
  }

  void obtain_and_send_transform(const std::string drake_parent,
      const std::string drake_child, const std::string golfcart_parent,
      const std::string golfcart_child) {
    tf::StampedTransform transform;
    try {
      listener_.lookupTransform(drake_parent, drake_child, ros::Time(0), transform);
      transform.frame_id_ = golfcart_parent;
      transform.child_frame_id_ = golfcart_child;
      transform.stamp_ = ros::Time::now();
      broadcaster_.sendTransform(transform);
    } catch (tf::TransformException ex) {
      if ((ros::Time::now() - start_time_).toSec() > kPublishErrorThreshold) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
      }
    }
  }

 private:
  const static double kPublishErrorThreshold = 2.0;
  ros::Time start_time_;

  tf::TransformListener listener_;
  tf::TransformBroadcaster broadcaster_;
};

int do_main(int argc, char* argv[]) {

  // Defines the frequency in Hz of publishing golfcart transforms.
  const double kCycleFrequency = 10.0;

  // Initializes ROS.
  ros::init(argc, argv, "golfcart_tf_publisher");

  // Sets the verbosity level to DEBUG.
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
      ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
  }

  // Instantiates some useful local variables.
  ros::NodeHandle node;
  PriusToGolfcartTFConverter tf_converter;
  tf::TransformBroadcaster broadcaster;

  // Cycles at kCycleFrequency Hz.
  ros::Rate rate(kCycleFrequency);
  while (node.ok()) {
    // ROS_DEBUG("Hello %s", "World");

    // Broadcasts an identity transform from "world" to "golfcartdj/map".
    broadcaster.sendTransform(tf::StampedTransform(tf::Transform::getIdentity(),
      ros::Time::now(), "world", "golfcartdj/map"));

    // Broadcasts a transform from "golfcartdj/map" to "golfcartdj/odom".
    // Assumes the transform is equal to Drake's "world" to "chassis_floor".
    tf_converter.obtain_and_send_transform(
      std::string("world"),
      std::string("chassis_floor"),
      std::string("golfcartdj/map"),
      std::string("golfcartdj/odom"));

    // Broadcasts a transform from "golfcartdj/odom" to "golfcartdj/base_link".
    // Assumes this transform is identity.
    broadcaster.sendTransform(tf::StampedTransform(tf::Transform::getIdentity(),
      ros::Time::now(), "golfcartdj/odom", "golfcartdj/base_link"));

    // Broadcasts a transform from "golfcartdj/base_link" to "body".
    // Assumes this transform is identity.
    broadcaster.sendTransform(tf::StampedTransform(tf::Transform::getIdentity(),
      ros::Time::now(), "golfcartdj/base_link", "body"));

    // Broadcasts a transform from "golfcartdj/base_link" to "front_lidar".
    // Assumes the transform is equal to Drake's "chassis_floor" to
    // "front_laser".
    tf_converter.obtain_and_send_transform(
      std::string("chassis_floor"),
      std::string("front_laser"),
      std::string("golfcartdj/base_link"),
      std::string("front_lidar"));

    // Broadcasts a transform from "golfcartdj/base_link" to "front_top_lidar".
    // Assumes the transform is equal to Drake's "chassis_floor" to "top_laser".
    tf_converter.obtain_and_send_transform(
      std::string("chassis_floor"),
      std::string("top_laser"),
      std::string("golfcartdj/base_link"),
      std::string("front_top_lidar"));

    // Broadcasts a transform from "golfcartdj/base_link" to "rear_right_lidar".
    // Assumes the transform is equal to Drake's "chassis_floor" to
    // "rear_right_laser".
    tf_converter.obtain_and_send_transform(
      std::string("chassis_floor"),
      std::string("rear_right_laser"),
      std::string("golfcartdj/base_link"),
      std::string("rear_right_lidar"));

    // Broadcasts a transform from "golfcartdj/base_link" to "rear_left_lidar".
    // Assumes the transform is equal to Drake's "chassis_floor" to
    // "rear_left_laser".
    tf_converter.obtain_and_send_transform(
      std::string("chassis_floor"),
      std::string("rear_right_laser"),
      std::string("golfcartdj/base_link"),
      std::string("rear_left_lidar"));

    rate.sleep();
  }
  return 0;
}

}  // namespace
}  // namespace cars
}  // namespace examples
}  // namespace drake


int main(int argc, char* argv[]) {
  return drake::examples::cars::do_main(argc, argv);
}
