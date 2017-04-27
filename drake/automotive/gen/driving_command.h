#pragma once

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "drake/common/never_destroyed.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace automotive {

/// Describes the row indices of a DrivingCommand.
struct DrivingCommandIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 2;

  // The index of each individual coordinate.
  static const int kSteeringAngle = 0;
  static const int kAcceleration = 1;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `DrivingCommandIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class DrivingCommand : public systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef DrivingCommandIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c steering_angle defaults to 0.0 in units of rad.
  /// @arg @c acceleration defaults to 0.0 in units of m/s^2.
  DrivingCommand() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_steering_angle(0.0);
    this->set_acceleration(0.0);
  }

  DrivingCommand<T>* DoClone() const override { return new DrivingCommand; }

  /// @name Getters and Setters
  //@{
  /// The desired steering angle of a virtual center wheel, positive results in
  /// the vehicle turning left.
  /// @note @c steering_angle is expressed in units of rad.
  const T& steering_angle() const {
    return this->GetAtIndex(K::kSteeringAngle);
  }
  void set_steering_angle(const T& steering_angle) {
    this->SetAtIndex(K::kSteeringAngle, steering_angle);
  }
  /// The signed acceleration, positive means speed up; negative means slow
  /// down, but should not move in reverse.
  /// @note @c acceleration is expressed in units of m/s^2.
  const T& acceleration() const { return this->GetAtIndex(K::kAcceleration); }
  void set_acceleration(const T& acceleration) {
    this->SetAtIndex(K::kAcceleration, acceleration);
  }
  //@}

  /// See DrivingCommandIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return DrivingCommandIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  decltype(T() < T()) IsValid() const {
    using std::isnan;
    auto result = (T(0) == T(0));
    result = result && !isnan(steering_angle());
    result = result && !isnan(acceleration());
    return result;
  }
};

}  // namespace automotive
}  // namespace drake
