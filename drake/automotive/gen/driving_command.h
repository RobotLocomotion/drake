#pragma once

// GENERATED FILE DO NOT EDIT
// See drake/automotive/lcm_vector_gen.py.

#include <stdexcept>
#include <string>

#include <Eigen/Core>

#include "drake/drakeAutomotive_export.h"
#include "drake/systems/framework/basic_vector.h"

// TODO(liang.fok) Remove this line after System 1.0-based systems are removed.
// See #3553.
#include "lcmtypes/drake/lcmt_driving_command_t.hpp"

namespace drake {
namespace automotive {

/// Describes the row indices of a DrivingCommand.
struct DRAKEAUTOMOTIVE_EXPORT DrivingCommandIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 3;

  // The index of each individual coordinate.
  static const int kSteeringAngle = 0;
  static const int kThrottle = 1;
  static const int kBrake = 2;
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class DrivingCommand : public systems::BasicVector<T> {
 public:
  // An abbreviation for our row index constants.
  typedef DrivingCommandIndices K;

  /// Default constructor.  Sets all rows to zero.
  DrivingCommand() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  /// @name Getters and Setters
  //@{
  const T steering_angle() const { return this->GetAtIndex(K::kSteeringAngle); }
  void set_steering_angle(const T& steering_angle) {
    this->SetAtIndex(K::kSteeringAngle, steering_angle);
  }
  const T throttle() const { return this->GetAtIndex(K::kThrottle); }
  void set_throttle(const T& throttle) {
    this->SetAtIndex(K::kThrottle, throttle);
  }
  const T brake() const { return this->GetAtIndex(K::kBrake); }
  void set_brake(const T& brake) { this->SetAtIndex(K::kBrake, brake); }
  //@}

  // TODO(liang.fok) Remove the following once System 1.0-based systems are
  // removed. See #3553.
  /// @name Implement the LCMVector concept
  //@{
  typedef drake::lcmt_driving_command_t LCMMessageType;
  static std::string channel() { return "DRIVING_COMMAND"; }
  //@}
};

}  // namespace automotive
}  // namespace drake
