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
      RigidBodyDepthSensor *rbds =
          dynamic_cast<RigidBodyDepthSensor *>(sensor.get());

      // If the dynamic cast was successful, the following code creates a ROS
      // topic publisher for publishing the data contained in the sensor's
      // output.
      if (rbds != nullptr) {
        std::cout << "Sensor " << rbds->get_name() << " is a LIDAR sensor!"
                  << std::endl;
        if (lidar_publishers_.find(rbds->get_name()) ==
            lidar_publishers_.end()) {
          std::string topic_name = "drake/lidar/" + rbds->get_name();
          lidar_publishers_.insert(std::pair<std::string, ros::Publisher>(
              rbds->get_name(),
              nh.advertise<sensor_msgs::LaserScan>(topic_name, 1)));
        } else {
          throw std::runtime_error(
              "ERROR: Multiple sensors of the same name detected!");
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
};

}  // end namespace plants
}  // end namespace systems
}  // end namespace drake
