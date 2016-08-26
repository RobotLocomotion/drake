#pragma once

// TODO(liang.fok): This file would normally generated by
// drake/examples/Cars/lcm_vector_gen.py. However, I had to make custom
// modifications to support multiple models.

#include <stdexcept>
#include <string>

#include <Eigen/Core>

#include "drake/drakeCars_export.h"
#include "drake/examples/Cars/gen/driving_command.h"
#include "drake/systems/framework/basic_state_and_output_vector.h"
#include "lcmtypes/drake/lcmt_driving_command_t.hpp"

namespace drake {

/// Specializes BasicStateAndOutputVector with specific getters and setters.
template <typename T>
class MultiDrivingCommand : public systems::BasicStateAndOutputVector<T> {
 public:
  // An abbreviation for our row index constants.
  typedef DrivingCommandIndices K;

  // Defines the number of vehicles in the system.
  // TODO(liang.fok) Generalize this to support an arbitrary number of vehicles.
  const int kNumVehicles = 5;

  /// Default constructor.  Sets all rows to zero.
  // TODO(liang.fok): Replace 15 with K::kNumCoordinates * kNumVehicles
  MultiDrivingCommand() : systems::BasicStateAndOutputVector<T>(15) {
    this->SetFromVector(VectorX<T>::Zero(15));
  }

  /// @name Getters and Setters
  //@{
  const T steering_angle(int model_instance_index) const {
    return this->GetAtIndex(
        model_instance_index * K::kNumCoordinates + K::kSteeringAngle);
  }
  void set_steering_angle(int model_instance_index, const T& steering_angle) {
    this->SetAtIndex(
        model_instance_index * K::kNumCoordinates + K::kSteeringAngle,
        steering_angle);
  }
  const T throttle(int model_instance_index) const {
    return this->GetAtIndex(model_instance_index * K::kNumCoordinates +
        K::kThrottle);
  }
  void set_throttle(int model_instance_index, const T& throttle) {
    this->SetAtIndex(
        model_instance_index * K::kNumCoordinates + K::kThrottle,
        throttle);
  }
  const T brake(int model_instance_index) const {
    return this->GetAtIndex(
        model_instance_index * K::kNumCoordinates + K::kBrake);
  }
  void set_brake(int model_instance_index, const T& brake) {
    this->SetAtIndex(model_instance_index * K::kNumCoordinates + K::kBrake,
        brake);
  }
  //@}

  /// @name Implement the LCMVector concept
  //@{
  typedef drake::lcmt_driving_command_t LCMMessageType;
  static std::string channel() { return "DRIVING_COMMAND"; }
  //@}
};


template <typename ScalarType>
bool encode(int model_instance_index, const double& t,
            const DrivingCommand<ScalarType>& wrap,
            // NOLINTNEXTLINE(runtime/references)
            drake::lcmt_driving_command_t& msg) {
  msg.timestamp = static_cast<int64_t>(t * 1000);
  msg.steering_angle = wrap.steering_angle(model_instance_index);
  msg.throttle = wrap.throttle(model_instance_index);
  msg.brake = wrap.brake(model_instance_index);
  return true;
}

template <typename ScalarType>
bool decode(int model_instance_index, const drake::lcmt_driving_command_t& msg,
            // NOLINTNEXTLINE(runtime/references)
            double& t,
            // NOLINTNEXTLINE(runtime/references)
            DrivingCommand<ScalarType>& wrap) {
  t = static_cast<double>(msg.timestamp) / 1000.0;
  wrap.set_steering_angle(model_instance_index, msg.steering_angle);
  wrap.set_throttle(model_instance_index, msg.throttle);
  wrap.set_brake(model_instance_index, msg.brake);
  return true;
}

}  // namespace drake
