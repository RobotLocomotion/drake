#pragma once

#include <Eigen/Dense>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

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

/** SensorVisualizerLidar<RobotStateVector>
 * @brief A system that takes the system state as the input,
 * saves LIDAR data in to a sensor_msgs::LaserScan message,
 * and then publish the message onto a ROS topic.
 *
 * @concept{system_concept}
 *
 * The resulting system has no internal state; the publish command is executed
 * on every call to the output method.
 *
 * For convenience, the input is passed directly through as an output.
 */

template <template <typename> class RobotStateVector>
class SensorVisualizerLidar {
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
   */
  SensorVisualizerLidar(std::shared_ptr<RigidBodySystem> rigid_body_system)
      : rigid_body_system_(rigid_body_system) {
    // Instantiates a ROS node handle, which is necessary to interact with ROS.
    // For more information, see:
    // http://wiki.ros.org/roscpp/Overview/NodeHandles
    ros::NodeHandle nh;

    // Creates a ROS topic publisher for each LIDAR sensor in the rigid body
    // system.
    for (auto &sensor : rigid_body_system->GetSensors()) {
      // Attempts to cast the RigidBodySensor pointer to a RigidBodyDepthSensor
      // pointer. This actually does two things simultaneously. First it
      // determines whether the pointer in fact points to a
      // RigidBodyDepthSensor. Second, if true, it also povides a pointer to the
      // RigidBodyDepthSensor that can be used later in this method.
      RigidBodyDepthSensor *depth_sensor =
          dynamic_cast<RigidBodyDepthSensor *>(sensor.get());

      if (depth_sensor != nullptr) {
        // If the dynamic cast was successful, that means the sensor is in fact
        // a RigidBodyDepth sensor. The following code creates a ROS topic
        // publisher that will be used to publish the data contained in the
        // sensor's output. It then creates a sensor_msgs::LaserScan message
        // for each publisher.

        std::cout << "**** Sensor " << depth_sensor->get_name()
                  << " is a LIDAR sensor!" << std::endl;

        if (lidar_publishers_.find(depth_sensor->get_name()) ==
            lidar_publishers_.end()) {
          std::string topic_name = "drake/lidar/" + depth_sensor->get_name();
          lidar_publishers_.insert(std::pair<std::string, ros::Publisher>(
              depth_sensor->get_name(),
              nh.advertise<sensor_msgs::LaserScan>(topic_name, 1)));
        } else {
          throw std::runtime_error(
              "ERROR: Multiple sensors with name " + depth_sensor->get_name() +
              " found when creating a ROS topic publisher for the sensor!");
        }

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
            message->angle_min = depth_sensor->get_min_yaw();
            message->angle_max = depth_sensor->get_max_yaw();
            message->angle_increment =
                (depth_sensor->get_max_yaw() - depth_sensor->get_min_yaw()) /
                depth_sensor->get_num_pixel_cols();
          } else {
            message->angle_min = depth_sensor->get_min_pitch();
            message->angle_max = depth_sensor->get_max_pitch();
            message->angle_increment = (depth_sensor->get_max_pitch() -
                                        depth_sensor->get_min_pitch()) /
                                       depth_sensor->get_num_pixel_rows();
          }

          // TODO(liangfok): Update sensor output to include time-between
          // measurement information.
          message->time_increment = 0;

          // TODO(liangfok): Update sensor output to include time-between-scan
          // information.
          message->scan_time = 0;

          message->range_min = depth_sensor->get_min_range();
          message->range_max = depth_sensor->get_max_range();
          message->ranges.resize(depth_sensor->getNumOutputs());
          message->intensities.resize(depth_sensor->getNumOutputs());

          lidar_messages_.insert(
              std::pair<std::string, std::unique_ptr<sensor_msgs::LaserScan>>(
                  depth_sensor->get_name(), std::move(message)));
        } else {
          throw std::runtime_error(
              "ERROR: Multiple sensors with name " + depth_sensor->get_name() +
              " found when creating a sensor_msgs::LaserScan message!");
        }
      }
    }
  }

  StateVector<double> dynamics(const double &t, const StateVector<double> &x,
                               const InputVector<double> &u) const {
    return StateVector<double>();
  }

  OutputVector<double> output(const double &t, const StateVector<double> &x,
                              const InputVector<double> &u) const {
    // Checks whether it's time to shutdown
    if (!ros::ok()) {
      ros::shutdown();
    }

    const std::vector<std::shared_ptr<RigidBodySensor>> &sensor_vector =
        rigid_body_system_->GetSensors();

    for (auto &sensor : sensor_vector) {
      std::cout << "SensorVisualizerLidar: output: Processing sensor "
                << sensor->get_name() << std::endl;
    }

    // std::cout << "SensorVisualizerLidar::output: Method called!\n"
    //           << "  - size of state vector: " << x.rows() << " x " <<
    //           x.cols() << "\n"
    //           << "  - size of input vector: " << u.rows() << " x " <<
    //           u.cols() << "\n"
    //           << "  - state vector: " << x.transpose() << "\n"
    //           << "  - input vector: " << u.transpose()
    //           << std::endl;

    return u;  // pass the output through
  }

  bool isTimeVarying() const { return true; }
  bool isDirectFeedthrough() const { return true; }

 private:
  std::shared_ptr<RigidBodySystem> rigid_body_system_;

  /**
   * Maintains a set of ROS topic publishers for publishing LIDAR messages.
   * The key is the name of the sensor. The value is the ROS topic publisher.
   */
  std::map<std::string, ros::Publisher> lidar_publishers_;

  /**
   * Maintains a set of ROS sensor_msgs::LaserScan messages for use by the
   * publishers. This is used to avoid having to allocate a new message
   * each time one needs to be sent.
   */
  std::map<std::string, std::unique_ptr<sensor_msgs::LaserScan>>
      lidar_messages_;
};

}  // end namespace plants
}  // end namespace systems
}  // end namespace drake
