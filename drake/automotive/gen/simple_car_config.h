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

/// Describes the row indices of a SimpleCarConfig.
struct SimpleCarConfigIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 6;

  // The index of each individual coordinate.
  static const int kWheelbase = 0;
  static const int kTrack = 1;
  static const int kMaxAbsSteeringAngle = 2;
  static const int kMaxVelocity = 3;
  static const int kMaxAcceleration = 4;
  static const int kVelocityLimitKp = 5;
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class SimpleCarConfig : public systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef SimpleCarConfigIndices K;

  /// Default constructor.  Sets all rows to zero.
  SimpleCarConfig() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  SimpleCarConfig<T>* DoClone() const override { return new SimpleCarConfig; }

  /// @name Getters and Setters
  //@{
  /// The distance between the front and rear axles of the vehicle, in meters;
  /// this element must be positive.
  const T& wheelbase() const { return this->GetAtIndex(K::kWheelbase); }
  void set_wheelbase(const T& wheelbase) {
    this->SetAtIndex(K::kWheelbase, wheelbase);
  }
  /// The distance between the center of two wheels on the same axle, in meters;
  /// this element must be positive.
  const T& track() const { return this->GetAtIndex(K::kTrack); }
  void set_track(const T& track) { this->SetAtIndex(K::kTrack, track); }
  /// The limit on the driving_command.steering angle input (the desired
  /// steering angle of a virtual center wheel), in radians; this element must
  /// be positive, and is applied symmetrically to both left- and right-turn
  /// limits.
  const T& max_abs_steering_angle() const {
    return this->GetAtIndex(K::kMaxAbsSteeringAngle);
  }
  void set_max_abs_steering_angle(const T& max_abs_steering_angle) {
    this->SetAtIndex(K::kMaxAbsSteeringAngle, max_abs_steering_angle);
  }
  /// The limit on the car's forward speed, in meters per second; this element
  /// must be positive.
  const T& max_velocity() const { return this->GetAtIndex(K::kMaxVelocity); }
  void set_max_velocity(const T& max_velocity) {
    this->SetAtIndex(K::kMaxVelocity, max_velocity);
  }
  /// The limit on the car's acceleration and deceleration, in meters per second
  /// per second; this element must be positive.
  const T& max_acceleration() const {
    return this->GetAtIndex(K::kMaxAcceleration);
  }
  void set_max_acceleration(const T& max_acceleration) {
    this->SetAtIndex(K::kMaxAcceleration, max_acceleration);
  }
  /// The smoothing constant for min/max velocity limits, in Hz; this element
  /// must be positive.
  const T& velocity_limit_kp() const {
    return this->GetAtIndex(K::kVelocityLimitKp);
  }
  void set_velocity_limit_kp(const T& velocity_limit_kp) {
    this->SetAtIndex(K::kVelocityLimitKp, velocity_limit_kp);
  }
  //@}

  /// Returns whether the current values of this vector are well-formed.
  decltype(T() < T()) IsValid() const {
    using std::isnan;
    auto result = (T(0) == T(0));
    result = result && !isnan(wheelbase());
    result = result && !isnan(track());
    result = result && !isnan(max_abs_steering_angle());
    result = result && !isnan(max_velocity());
    result = result && !isnan(max_acceleration());
    result = result && !isnan(velocity_limit_kp());
    return result;
  }
};

}  // namespace automotive
}  // namespace drake
