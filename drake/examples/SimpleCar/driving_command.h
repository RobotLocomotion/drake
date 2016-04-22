// Copyright 2016 Robot Locomotion Group @ CSAIL. All rights reserved.
// This file is generated by a script.  Do not edit!
// See drake/examples/SimpleCar/lcm_vector_gen.py.
#pragma once

#include <stdexcept>
#include <string>
#include <Eigen/Core>

#include "lcmtypes/drake/lcmt_driving_command_t.hpp"

namespace Drake {

template <typename ScalarType = double>
class DrivingCommand {  // models the Drake::LCMVector concept
 public:
  typedef drake::lcmt_driving_command_t LCMMessageType;
  static std::string channel() { return "DRIVING_COMMAND"; }
  static const int RowsAtCompileTime = 3;
  typedef Eigen::Matrix<ScalarType, RowsAtCompileTime, 1> EigenType;

  DrivingCommand() {}

  template <typename Derived>
  // NOLINTNEXTLINE(runtime/explicit)
  DrivingCommand(const Eigen::MatrixBase<Derived>& initial)
      : steering_angle(initial(0)),  // BR
        throttle(initial(1)),        // BR
        brake(initial(2)) {}

  template <typename Derived>
  DrivingCommand& operator=(const Eigen::MatrixBase<Derived>& rhs) {
    steering_angle = rhs(0);
    throttle = rhs(1);
    brake = rhs(2);
    return *this;
  }

  friend EigenType toEigen(const DrivingCommand<ScalarType>& vec) {
    EigenType result;
    result << vec.steering_angle, vec.throttle, vec.brake;
    return result;
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
    throw std::domain_error("unknown coordinate index");
  }

  ScalarType steering_angle = 0;
  ScalarType throttle = 0;
  ScalarType brake = 0;
};

template <typename ScalarType>
bool encode(const double& t, const DrivingCommand<ScalarType>& wrap,
            // NOLINTNEXTLINE(runtime/references)
            drake::lcmt_driving_command_t& msg) {
  msg.timestamp = static_cast<int64_t>(t * 1000);
  msg.steering_angle = wrap.steering_angle;
  msg.throttle = wrap.throttle;
  msg.brake = wrap.brake;
  return true;
}

template <typename ScalarType>
bool decode(const drake::lcmt_driving_command_t& msg,
            // NOLINTNEXTLINE(runtime/references)
            double& t,
            // NOLINTNEXTLINE(runtime/references)
            DrivingCommand<ScalarType>& wrap) {
  t = static_cast<double>(msg.timestamp) / 1000.0;
  wrap.steering_angle = msg.steering_angle;
  wrap.throttle = msg.throttle;
  wrap.brake = msg.brake;
  return true;
}

}  // namespace Drake
