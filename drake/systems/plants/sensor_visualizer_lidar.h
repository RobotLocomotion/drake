#pragma once

#include <Eigen/Dense>
#include "drake/systems/System.h"
#include "drake/systems/plants/RigidBodyTree.h"

namespace Drake {

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

  SensorVisualizerLidar(std::shared_ptr<RigidBodySystem> rigid_body_system)
      : rigid_body_system_(rigid_body_system) {
  }

  StateVector<double> dynamics(const double &t, const StateVector<double> &x,
                               const InputVector<double> &u) const {
    return StateVector<double>();
  }

  OutputVector<double> output(const double &t, const StateVector<double> &x,
                              const InputVector<double> &u) const {

    std::cout << "SensorVisualizerLidar::output: Method called!\n"
              << "  - size of state vector: " << x.rows() << " x " << x.cols() << "\n"
              << "  - size of input vector: " << u.rows() << " x " << u.cols() << "\n"
              << "  - state vector: " << x.transpose() << "\n"
              << "  - input vector: " << u.transpose()
              << std::endl;

    return u;  // pass the output through
  }

  bool isTimeVarying() const { return true; }
  bool isDirectFeedthrough() const { return true; }

 private:
  std::shared_ptr<RigidBodySystem> rigid_body_system_;
};

}  // end namespace Drake
