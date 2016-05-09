#pragma once

// #include <algorithm>
// #include <cmath>
#include <string>

#include <Eigen/Geometry>

#include "drake/drakeCar_export.h"
// #include "drake/examples/Cars/gen/driving_command.h"
#include "lcmtypes/drake/lcmt_driving_control_cmd_t.hpp"

template <typename ScalarType = double>
class DrivingCommand {
 public:
  typedef drake::lcmt_driving_control_cmd_t LCMMessageType;
  static std::string channel() { return "DRIVING_COMMAND"; }

  DrivingCommand(void) : throttle(0), brake(0), steering_angle(0) {}

  template <typename Derived>
  DrivingCommand(  // NOLINT(runtime/explicit) per Drake::Vector.
      const Eigen::MatrixBase<Derived>& x)
      : steering_angle(x(0)), throttle(x(1)), brake(x(2)) {}

  template <typename Derived>
  DrivingCommand& operator=(const Eigen::MatrixBase<Derived>& x) {
    steering_angle = x(0);
    throttle = x(1);
    brake = x(2);
    return *this;
  }

  friend Eigen::Vector3d toEigen(const DrivingCommand<ScalarType>& vec) {
    Eigen::Vector3d x;
    x << vec.steering_angle, vec.throttle, vec.brake;
    return x;
  }

  friend std::string getCoordinateName(const DrivingCommand<ScalarType>& vec,
                                       unsigned int index) {
    switch (index) {
      case 0:
        return "steering_angle";
      case 1:
        return "throttle";
      case 2:
        return "brake";
    }
    return "error";
  }
  static const int RowsAtCompileTime = 3;

  ScalarType steering_angle;
  ScalarType throttle;
  ScalarType brake;
};

/**
 * A toString method for DrivingCommand.
 */
template <typename ScalarType = double>
std::ostream& operator<<(std::ostream& os,
                         const DrivingCommand<ScalarType>& dc) {
  return os << "[steering_angle = " << dc.steering_angle
            << ", throttle = " << dc.throttle << ", brake = " << dc.brake
            << "]";
}

bool decode(const drake::lcmt_driving_control_cmd_t& msg, double& t,
            DrivingCommand<double>& x);

namespace drake {

}  // namespace drake
