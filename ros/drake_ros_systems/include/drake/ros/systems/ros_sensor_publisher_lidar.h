#pragma once

#include <map>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "drake/system1/System.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/system1/vector.h"

using drake::NullVector;
using drake::RigidBodySensor;
using drake::RigidBodySystem;
using drake::RigidBodyDepthSensor;

namespace drake {
namespace ros {
namespace systems {

/**
 * @brief A system that takes the system state as the input,
 * saves LIDAR data in to a sensor_msgs::LaserScan message,
 * and then publishes the message onto a ROS topic.
 *
 * @concept{system_concept}
 *
 * The resulting system has no internal state; the publish command is throttled
 * by kMinTransmitPeriod_.
 *
 * For convenience, the input is passed directly through as an output. This
 * enables other systems to be cascaded after this system.
 */
template <template <typename> class RobotStateVector>
class SensorPublisherLidar {
 private:
  // Specifies that the LIDAR messages should be transmitted with a minimum
  // period of 0.05 seconds.
  //
  // TODO(liangfok): Modify sensor model to include scan frequency and time
  // between scan measurements. See:
  // https://github.com/RobotLocomotion/drake/issues/2210
  static constexpr double kMinTransmitPeriod_ = 0.05;

 public:
  template <typename ScalarType>
  using StateVector = NullVector<ScalarType>;
  template <typename ScalarType>
  using OutputVector = RobotStateVector<ScalarType>;
  template <typename ScalarType>
  using InputVector = RobotStateVector<ScalarType>;

  /**
   * The constructor. It takes a rigid body system as an input parameter to get
   * semantic information about the output from the rigid body system,
   * specifically about the sensor state.
   *
   * Ideally, the output of the rigid body system should be self-descriptive.
   * See: https://github.com/RobotLocomotion/drake/issues/2152
   *
   * @param[in] rigid_body_system The rigid body system whose output contains
   * the odometry information.
   */
  explicit SensorPublisherLidar(
      std::shared_ptr<RigidBodySystem> rigid_body_system)
      : rigid_body_system_(rigid_body_system) {
    // Initializes the time stamp of the previous transmission to be zero.
    previous_send_time_.sec = 0;
    previous_send_time_.nsec = 0;

    // Instantiates a ROS node handle through which we can interact with ROS.
    // For more information, see:
    // http://wiki.ros.org/roscpp/Overview/NodeHandles
    ::ros::NodeHandle nh;

    // Creates a ROS topic publisher for each LIDAR sensor in the rigid body
    // system.
    for (auto sensor : rigid_body_system->GetSensors()) {
      // Attempts to cast the RigidBodySensor pointer to a RigidBodyDepthSensor
      // pointer. This actually does two things simultaneously. First it
      // determines whether the pointer in fact points to a
      // RigidBodyDepthSensor. Second, if true, it also povides a pointer to the
      // RigidBodyDepthSensor that can be used later in this method.
      const RigidBodyDepthSensor* depth_sensor =
          dynamic_cast<const RigidBodyDepthSensor*>(sensor);

      if (depth_sensor != nullptr) {
        // If the dynamic cast is successful, the sensor is in fact a
        // RigidBodyDepthSensor. The following code creates a ROS topic
        // publisher that will be used to publish the data contained in the
        // sensor's output. The ROS topic is
        // "drake/lidar/[name of robot]/[name of sensor]/".
        // It then creates a sensor_msgs::LaserScan message for each publisher.
        const std::string model_name = depth_sensor->get_model_name();
        const std::string key = model_name + "_" + depth_sensor->get_name();

        // Creates the ROS topic publisher for the current LIDAR sensor.
        if (lidar_publishers_.find(key) == lidar_publishers_.end()) {
          const std::string topic_name =
              "drake/" + model_name + "/lidar/" + depth_sensor->get_name();

          lidar_publishers_.insert(std::pair<std::string, ::ros::Publisher>(
              key, nh.advertise<sensor_msgs::LaserScan>(topic_name, 1)));
        } else {
          throw std::runtime_error(
              "ERROR: Multiple sensors with name " + depth_sensor->get_name() +
              " found when creating a ROS topic publisher for the sensor!");
        }

        // Creates the ROS message for the current LIDAR sensor.
        if (lidar_messages_.find(depth_sensor->get_name()) ==
            lidar_messages_.end()) {
          std::unique_ptr<sensor_msgs::LaserScan> message(
              new sensor_msgs::LaserScan());
          message->header.frame_id = depth_sensor->get_name();

          // The rigid body depth sensor scans either horizontally or
          // vertically.
          bool is_horizontal_scanner = depth_sensor->is_horizontal_scanner();
          bool is_vertical_scanner = depth_sensor->is_vertical_scanner();

          if (is_horizontal_scanner && is_vertical_scanner)
            throw std::runtime_error(
                "ERROR: Rigid body depth sensor " + depth_sensor->get_name() +
                " has both horizontal and vertical dimensions. Expecting it to "
                "only scan within a 2D plane!");

          if (is_horizontal_scanner) {
            message->angle_min = depth_sensor->min_yaw();
            message->angle_max = depth_sensor->max_yaw();
            message->angle_increment =
                (depth_sensor->max_yaw() - depth_sensor->min_yaw()) /
                depth_sensor->num_pixel_cols();
          } else {
            message->angle_min = depth_sensor->min_pitch();
            message->angle_max = depth_sensor->max_pitch();
            message->angle_increment =
                (depth_sensor->max_pitch() - depth_sensor->min_pitch()) /
                depth_sensor->num_pixel_rows();
          }

          // Since the RigidBodyDepthSensor does not include this information
          // in the output and it is difficult to obtain this information
          // from other variables within this method's scope, set this value
          // to be zero.
          //
          // See:
          // https://github.com/RobotLocomotion/drake/issues/2210
          message->time_increment = 0;
          message->scan_time = 0;

          message->range_min = depth_sensor->min_range();
          message->range_max = depth_sensor->max_range();
          message->ranges.resize(depth_sensor->getNumOutputs());
          message->intensities.resize(depth_sensor->getNumOutputs());

          lidar_messages_.insert(
              std::pair<std::string, std::unique_ptr<sensor_msgs::LaserScan>>(
                  key, std::move(message)));
        } else {
          throw std::runtime_error(
              "ERROR: Multiple sensors with name " + depth_sensor->get_name() +
              " found when creating a sensor_msgs::LaserScan message!");
        }
      }
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

    std::vector<const RigidBodySensor*> sensor_vector =
        rigid_body_system_->GetSensors();

    // This variable is for tracking where in the output of rigid body system
    // we are currently processing. We initialize it to be the number of states
    // of the rigid body system to start past the joint state information.
    size_t output_index = rigid_body_system_->getNumStates();

    // Iterates through each sensor in the rigid body system. If the sensor is
    // a LIDAR sensor, store the range measurements in a ROS message and publish
    // it on the appropriate ROS topic.
    for (const RigidBodySensor* sensor : sensor_vector) {
      if (output_index + sensor->getNumOutputs() >
          rigid_body_system_->getNumOutputs()) {
        std::stringstream buff;
        buff << "ERROR: Sum of output index " << output_index
             << " and number of outputs of sensor " << sensor->get_name()
             << " (" << sensor->getNumOutputs()
             << ") exceeds the total number of outputs of the rigid body "
                "system (" << rigid_body_system_->getNumOutputs();
        throw std::runtime_error(buff.str());
      }

      // TODO(liang.fok): Update API to not require type dispatch!
      // See: https://github.com/RobotLocomotion/drake/issues/2802.
      const RigidBodyDepthSensor* depth_sensor =
          dynamic_cast<const RigidBodyDepthSensor*>(sensor);

      if (depth_sensor != nullptr) {
        size_t sensor_data_index_ = output_index;

        const std::string& model_name = depth_sensor->get_model_name();
        const std::string key = model_name + "_" + depth_sensor->get_name();

        auto message_in_map = lidar_messages_.find(key);
        if (message_in_map == lidar_messages_.end())
          throw std::runtime_error(
              "Could not find ROS message for LIDAR sensor " +
              depth_sensor->get_name() + " in robot " + model_name + ".");

        sensor_msgs::LaserScan* message = message_in_map->second.get();

        // Saves the new range measurements in the ROS message.
        for (size_t ii = 0; ii < depth_sensor->getNumOutputs(); ii++) {
          message->ranges[ii] = u[sensor_data_index_++];
        }

        // Publishes the ROS message containing the new range measurements.
        auto publisher_in_map = lidar_publishers_.find(key);
        if (publisher_in_map == lidar_publishers_.end())
          throw std::runtime_error(
              "ERROR: Failed to find ROS topic publisher for LIDAR sensor " +
              depth_sensor->get_name() + " in robot " + model_name + ".");

        publisher_in_map->second.publish(*message);
      }

      // Shifts the output index variable forward by one sensor.
      output_index += sensor->getNumOutputs();
    }

    return u;  // Passes the output through to the next system in the cascade.
  }

  bool isTimeVarying() const { return true; }
  bool isDirectFeedthrough() const { return true; }

 private:
  std::shared_ptr<RigidBodySystem> rigid_body_system_;

  /**
   * Maintains a set of ROS topic publishers for publishing LIDAR messages.
   * The key is the name of the sensor. The value is the ROS topic publisher.
   */
  std::map<std::string, ::ros::Publisher> lidar_publishers_;

  /**
   * Maintains a set of ROS sensor_msgs::LaserScan messages for use by the
   * publishers. This is used to avoid having to allocate a new message
   * each time one needs to be sent.
   */
  std::map<std::string, std::unique_ptr<sensor_msgs::LaserScan>>
      lidar_messages_;

  /**
   * The previous time the LIDAR messages were sent.
   */
  ::ros::Time previous_send_time_;
};

}  // namespace systems
}  // namespace ros
}  // namespace drake
