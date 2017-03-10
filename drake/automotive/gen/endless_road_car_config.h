#pragma once

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <cmath>
#include <stdexcept>
#include <string>

#include <Eigen/Core>

#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace automotive {

/// Describes the row indices of a EndlessRoadCarConfig.
struct EndlessRoadCarConfigIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 5;

  // The index of each individual coordinate.
  static const int kWheelbase = 0;
  static const int kMaxAbsSteeringAngle = 1;
  static const int kMaxVelocity = 2;
  static const int kMaxAcceleration = 3;
  static const int kMaxDeceleration = 4;
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class EndlessRoadCarConfig : public systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef EndlessRoadCarConfigIndices K;

  /// Default constructor.  Sets all rows to zero.
  EndlessRoadCarConfig() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  EndlessRoadCarConfig<T>* DoClone() const override {
    return new EndlessRoadCarConfig;
  }

  /// @name Getters and Setters
  //@{
  /// distance between left/right wheels [positive value, meters]
  const T& wheelbase() const { return this->GetAtIndex(K::kWheelbase); }
  void set_wheelbase(const T& wheelbase) {
    this->SetAtIndex(K::kWheelbase, wheelbase);
  }
  /// maximum absolute front wheel angle (from forward) [positive value,
  /// radians]
  const T& max_abs_steering_angle() const {
    return this->GetAtIndex(K::kMaxAbsSteeringAngle);
  }
  void set_max_abs_steering_angle(const T& max_abs_steering_angle) {
    this->SetAtIndex(K::kMaxAbsSteeringAngle, max_abs_steering_angle);
  }
  /// maximum allowed forward velocity [positive value, m/s]
  const T& max_velocity() const { return this->GetAtIndex(K::kMaxVelocity); }
  void set_max_velocity(const T& max_velocity) {
    this->SetAtIndex(K::kMaxVelocity, max_velocity);
  }
  /// maximum forward acceleration [positive value, m/s^2]
  const T& max_acceleration() const {
    return this->GetAtIndex(K::kMaxAcceleration);
  }
  void set_max_acceleration(const T& max_acceleration) {
    this->SetAtIndex(K::kMaxAcceleration, max_acceleration);
  }
  /// maximum forward deceleration [positive value, m/s^2]
  const T& max_deceleration() const {
    return this->GetAtIndex(K::kMaxDeceleration);
  }
  void set_max_deceleration(const T& max_deceleration) {
    this->SetAtIndex(K::kMaxDeceleration, max_deceleration);
  }
  //@}

  /// Returns whether the current values of this vector are well-formed.
  decltype(T() < T()) IsValid() const {
    using std::isnan;
    auto result = (T(0) == T(0));
    result = result && !isnan(wheelbase());
    result = result && !isnan(max_abs_steering_angle());
    result = result && !isnan(max_velocity());
    result = result && !isnan(max_acceleration());
    result = result && !isnan(max_deceleration());
    return result;
  }
};

}  // namespace automotive
}  // namespace drake
