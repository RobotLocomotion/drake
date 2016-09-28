#pragma once

#include <Eigen/Dense>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "drake/systems/System.h"
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
 * Holds the objects and data needed to extract and publish joint state
 * information for a particular model instance.
 */
struct LidarSensorStruct {
  // The ROS topic publisher for publishing the LIDAR data.
  ::ros::Publisher publisher;

  // This holds LIDAR sensor's measurements and is periodically published.
  std::unique_ptr<sensor_msgs::LaserScan> message;
};

/**
 * Takes the system's state vector and publishes `sensor_msgs::LaserScan`
 * messages on ROS topics. The ROS topics are namespaced by the model instance
 * names.
 *
 * The resulting system has no internal state; the publish command is throttled
 * by kMinTransmitPeriod_.
 *
 * For convenience, the input is passed directly through as an output. This
 * enables other systems to be cascaded after this system.
 */
template <template <typename> class RobotStateVector>
class RosSensorPublisherLidar {
 private:
  // Specifies the minimum period at which to publish LIDAR messages.
  static constexpr double kMinTransmitPeriod_ = 0.05;

 public:
  template <typename ScalarType>
  using StateVector = NullVector<ScalarType>;
  template <typename ScalarType>
  using OutputVector = RobotStateVector<ScalarType>;
  template <typename ScalarType>
  using InputVector = RobotStateVector<ScalarType>;

  /**
   * The constructor, which initializes all internal data structures necessary
   * to publish LIDAR state information for each model instance in @p
   * rigid_body_system.
   *
   * @param[in] rigid_body_system The rigid body system whose output contains
   * the LIDAR information.
   *
   * @param[in] model_instance_names A mapping from model instance IDs to model
   * instance names. These names are used to prefix the names of the frames in
   * the `sensor_msgs::LaserScan` messages, which is necessary for RViz to
   * simultaneously visualize the sensors belonging to multiple models.
   */
  explicit RosSensorPublisherLidar(
      std::shared_ptr<RigidBodySystem> rigid_body_system,
      const std::map<int, std::string>& model_instance_names)
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
      // Attempts to cast the pointer to a `RigidBodySensor` to be a pointer to
      // a `RigidBodyDepthSensor`. This actually does two things simultaneously.
      // First it determines whether the pointer in fact points to a
      // `RigidBodyDepthSensor`. Second, if true, it also povides a pointer to
      // the `RigidBodyDepthSensor`.
      const RigidBodyDepthSensor* depth_sensor =
          dynamic_cast<const RigidBodyDepthSensor*>(sensor);

      if (depth_sensor != nullptr) {
        // If the dynamic cast is successful, the sensor is a
        // `RigidBodyDepthSensor`. The following code creates a ROS topic
        // publisher that will be used to publish the data contained in the
        // sensor's output. The ROS topic is:
        //
        //     [model instance name]/lidar/[sensor name]/.
        //
        // It then creates a `sensor_msgs::LaserScan` message for each
        // publisher.
        const std::string model_name = depth_sensor->get_model_name();
        int model_instance_id = depth_sensor->get_frame()
            .get_model_instance_id();

        // Verifies that the model instance name can be determined.
        // Throws an error if it cannot.
        if (model_instance_names.find(model_instance_id) ==
            model_instance_names.end()) {
          throw std::runtime_error("RosSensorPublisherLidar: ERROR: Unable to "
              "determine model instance name of model with instance ID " +
              std::to_string(model_instance_id) + ".");
        }

        // Instantiates a LidarSensorStruct for the current sensor and
        // initializes various member variables within it.
        std::unique_ptr<LidarSensorStruct> sensor_struct(
            new LidarSensorStruct());

        const std::string model_instance_name =
            model_instance_names.at(model_instance_id);

        const std::string sensor_name = depth_sensor->get_name();

        const std::string topic_name = model_instance_name + "/lidar/" +
            sensor_name;

        // ROS_INFO_STREAM("Creating LIDAR sensor publisher on topic "
        //     << topic_name);
        sensor_struct->publisher =
            nh.advertise<sensor_msgs::LaserScan>(topic_name, 1);

        sensor_struct->message.reset(new sensor_msgs::LaserScan());
        sensor_struct->message->header.frame_id = model_instance_name +
            "/" + sensor_name;

        // The rigid body depth sensor scans either horizontally or
        // vertically.
        bool is_horizontal_scanner = depth_sensor->is_horizontal_scanner();
        bool is_vertical_scanner = depth_sensor->is_vertical_scanner();

        // Throws an error if the sensor scans both horizontally and vertically.
        if (is_horizontal_scanner && is_vertical_scanner) {
          throw std::runtime_error(
              "ERROR: Rigid body depth sensor " + depth_sensor->get_name() +
              " has both horizontal and vertical dimensions. Expecting it to "
              "only scan within a 2D plane!");
        }

        if (is_horizontal_scanner) {
          sensor_struct->message->angle_min = depth_sensor->min_yaw();
          sensor_struct->message->angle_max = depth_sensor->max_yaw();
          sensor_struct->message->angle_increment =
              (depth_sensor->max_yaw() - depth_sensor->min_yaw()) /
              depth_sensor->num_pixel_cols();
        } else {
          sensor_struct->message->angle_min = depth_sensor->min_pitch();
          sensor_struct->message->angle_max = depth_sensor->max_pitch();
          sensor_struct->message->angle_increment =
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
        sensor_struct->message->time_increment = 0;
        sensor_struct->message->scan_time = 0;

        sensor_struct->message->range_min = depth_sensor->min_range();
        sensor_struct->message->range_max = depth_sensor->max_range();
        sensor_struct->message->ranges.resize(depth_sensor->getNumOutputs());
        sensor_struct->message->intensities.resize(
            depth_sensor->getNumOutputs());

        lidar_table_.push_back(std::move(sensor_struct));
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
    // of the rigid body system to start immediately after the joint states in
    // in the state vector.
    size_t output_index = rigid_body_system_->getNumStates();

    // For keeping track of which LidarSesnorStruct we are processing in the
    // for loop below.
    int lidar_table_index = 0;

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
        std::unique_ptr<LidarSensorStruct>& sensor_struct =
            lidar_table_[lidar_table_index++];

        // Saves the new range measurements in the ROS message.
        for (size_t ii = 0; ii < depth_sensor->getNumOutputs(); ii++) {
          sensor_struct->message->ranges[ii] = u[sensor_data_index_++];
        }

        sensor_struct->publisher.publish(*(sensor_struct->message.get()));
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
   * Maintains a set of ROS sensor_msgs::LaserScan messages for use by the
   * publishers. This is used to avoid having to allocate a new message
   * each time one needs to be sent.
   */
  std::vector<std::unique_ptr<LidarSensorStruct>> lidar_table_;

  /**
   * The previous time the LIDAR messages were sent.
   */
  ::ros::Time previous_send_time_;
};

}  // namespace systems
}  // namespace ros
}  // namespace drake
