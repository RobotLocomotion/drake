#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

int main(int argc, char** argv){
  // Defines the frequency in Hz of publishing golfcart transforms.
  const double kCycleFrequency = 10.0;

  // Initializes ROS.
  ros::init(argc, argv, "golfcart_tf_publisher");

  // Sets the verbosity level to DEBUG.
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
      ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
  }

  // Instantiates a ROS node and a tf transform listener.
  ros::NodeHandle node;
  tf::TransformListener listener;
  tf::TransformBroadcaster broadcaster;

  // Cycles at kCycleFrequency Hz.
  ros::Rate rate(kCycleFrequency);
  while (node.ok()) {
    // ROS_DEBUG("Hello %s", "World");

    // Broadcasts a transform from "world" to "golfcartdj/map".
    broadcaster.sendTransform(tf::StampedTransform(tf::Transform::getIdentity(),
      ros::Time::now(), "world", "golfcartdj/map"));

    // Broadcasts a transform from "golfcartdj/map" to "golfcartdj/odom".
    // Assumes the transform is equal to Drake's "world" to "chassis_floor".
    tf::StampedTransform T_world_to_chassis_floor;
    try {
      listener.lookupTransform("world", "chassis_floor", ros::Time(0),
        T_world_to_chassis_floor);
      T_world_to_chassis_floor.frame_id_ = "golfcartdj/map";
      T_world_to_chassis_floor.child_frame_id_ = "golfcartdj/odom";
      T_world_to_chassis_floor.stamp_ = ros::Time::now();
      broadcaster.sendTransform(T_world_to_chassis_floor);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

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
    tf::StampedTransform T_base_link_to_front_lidar;
    try {
      listener.lookupTransform("chassis_floor", "front_laser", ros::Time(0),
        T_base_link_to_front_lidar);
      T_base_link_to_front_lidar.frame_id_ = "golfcartdj/base_link";
      T_base_link_to_front_lidar.child_frame_id_ = "front_lidar";
      T_base_link_to_front_lidar.stamp_ = ros::Time::now();
      broadcaster.sendTransform(T_base_link_to_front_lidar);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    // Broadcasts a transform from "golfcartdj/base_link" to "front_top_lidar".
    // Assumes the transform is equal to Drake's "chassis_floor" to "top_laser".
    tf::StampedTransform T_base_link_to_top_lidar;
    try {
      listener.lookupTransform("chassis_floor", "top_laser", ros::Time(0),
        T_base_link_to_top_lidar);
      T_base_link_to_top_lidar.frame_id_ = "golfcartdj/base_link";
      T_base_link_to_top_lidar.child_frame_id_ = "front_top_lidar";
      T_base_link_to_top_lidar.stamp_ = ros::Time::now();
      broadcaster.sendTransform(T_base_link_to_top_lidar);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    // Broadcasts a transform from "golfcartdj/base_link" to "rear_right_lidar".
    // Assumes the transform is equal to Drake's "chassis_floor" to
    // "rear_right_laser".
    tf::StampedTransform T_base_link_to_rear_right_lidar;
    try {
      listener.lookupTransform("chassis_floor", "rear_right_laser",
        ros::Time(0), T_base_link_to_rear_right_lidar);
      T_base_link_to_rear_right_lidar.frame_id_ = "golfcartdj/base_link";
      T_base_link_to_rear_right_lidar.child_frame_id_ = "rear_right_lidar";
      T_base_link_to_rear_right_lidar.stamp_ = ros::Time::now();
      broadcaster.sendTransform(T_base_link_to_rear_right_lidar);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    // Broadcasts a transform from "golfcartdj/base_link" to "rear_left_lidar".
    // Assumes the transform is equal to Drake's "chassis_floor" to
    // "rear_left_laser".
    tf::StampedTransform T_base_link_to_rear_left_lidar;
    try {
      listener.lookupTransform("chassis_floor", "rear_left_laser", ros::Time(0),
        T_base_link_to_rear_left_lidar);
      T_base_link_to_rear_left_lidar.frame_id_ = "golfcartdj/base_link";
      T_base_link_to_rear_left_lidar.child_frame_id_ = "rear_left_lidar";
      T_base_link_to_rear_left_lidar.stamp_ = ros::Time::now();
      broadcaster.sendTransform(T_base_link_to_rear_left_lidar);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    rate.sleep();
  }
  return 0;
};