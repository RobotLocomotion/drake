#include <thread>

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/LaserScan.h>

namespace drake {
namespace examples {
namespace cars {
namespace {

class PriusToGolfCartTFConverter {
 public:
  PriusToGolfCartTFConverter() :
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
  constexpr static double kPublishErrorThreshold = 2.0;
  ros::Time start_time_;

  tf::TransformListener listener_;
  tf::TransformBroadcaster broadcaster_;
};

class PriusToGolfCartSensorConverter {
 public:
  PriusToGolfCartSensorConverter() {
  }

  void operator()() {
    ros::NodeHandle node;

    // Declares the publishers for the golf cart sensor data.
    pub_front_lidar = node.advertise<sensor_msgs::LaserScan>("front_lidar", 1000);
    pub_front_top_lidar = node.advertise<sensor_msgs::LaserScan>("front_top_lidar", 1000);
    pub_rear_right_lidar = node.advertise<sensor_msgs::LaserScan>("rear_right_lidar", 1000);;
    pub_rear_left_lidar = node.advertise<sensor_msgs::LaserScan>("rear_left_lidar", 1000);

    // Subscribes to the ROS topics containing Prius sensor data
    sub_front_laser = node.subscribe(
      "/drake/prius_1/lidar/front_laser", 1,
      & PriusToGolfCartSensorConverter::callback_front_laser, this);
    sub_top_laser = node.subscribe(
      "/drake/prius_1/lidar/top_laser", 1,
      & PriusToGolfCartSensorConverter::callback_top_laser, this);
    sub_rear_left_laser = node.subscribe(
      "/drake/prius_1/lidar/rear_left_laser",
       1, & PriusToGolfCartSensorConverter::callback_rear_left_laser, this);
    sub_rear_right_laser = node.subscribe(
      "/drake/prius_1/lidar/rear_right_laser", 1,
      & PriusToGolfCartSensorConverter::callback_rear_right_laser, this);

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();
  }

  void callback_front_laser(const sensor_msgs::LaserScanPtr& msg) {
    sensor_msgs::LaserScan message = *(msg.get());
    message.header.frame_id = "front_lidar";
    pub_front_lidar.publish(message);
  }

  void callback_top_laser(const sensor_msgs::LaserScanPtr& msg) {
    sensor_msgs::LaserScan message = *(msg.get());
    message.header.frame_id = "front_top_lidar";
    pub_front_top_lidar.publish(message);
  }

  void callback_rear_left_laser(const sensor_msgs::LaserScanPtr& msg) {
    sensor_msgs::LaserScan message = *(msg.get());
    message.header.frame_id = "rear_left_lidar";
    pub_rear_left_lidar.publish(message);
  }

  void callback_rear_right_laser(const sensor_msgs::LaserScanPtr& msg) {
    sensor_msgs::LaserScan message = *(msg.get());
    message.header.frame_id = "rear_right_lidar";
    pub_rear_right_lidar.publish(message);
  }

 private:
  // Declares the golf cart ROS topic publishers
  ros::Publisher pub_front_lidar;
  ros::Publisher pub_front_top_lidar;
  ros::Publisher pub_rear_right_lidar;
  ros::Publisher pub_rear_left_lidar;

  // Declares the Prius sensor ROS topic listeners.
  ros::Subscriber sub_front_laser;
  ros::Subscriber sub_top_laser;
  ros::Subscriber sub_rear_left_laser;
  ros::Subscriber sub_rear_right_laser;
};

int do_main(int argc, char* argv[]) {
  // Defines the frequency in Hz of publishing golfcart transforms.
  const double kCycleFrequency = 10.0;

  // Initializes ROS.
  ros::init(argc, argv, "golfcart_adapter");

  // Sets the verbosity level to DEBUG.
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
      ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
  }

  // Instantiates some useful local variables.
  ros::NodeHandle node;
  PriusToGolfCartTFConverter tf_converter;
  tf::TransformBroadcaster broadcaster;
  PriusToGolfCartSensorConverter msg_converter;

  // Starts a thread for handling sensor message conversion.
  std::thread msg_converter_thread(msg_converter);

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
      std::string("rear_left_laser"),
      std::string("golfcartdj/base_link"),
      std::string("rear_left_lidar"));

    rate.sleep();
  }

  msg_converter_thread.join();

  return 0;
}

}  // namespace
}  // namespace cars
}  // namespace examples
}  // namespace drake


int main(int argc, char* argv[]) {
  return drake::examples::cars::do_main(argc, argv);
}
