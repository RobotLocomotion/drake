#pragma once

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <stdexcept>
#include <string>

#include <Eigen/Core>

#include "drake/common/drake_export.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace automotive {

/// Describes the row indices of a SimpleCarConfig.
struct DRAKE_EXPORT SimpleCarConfigIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 7;

  // The index of each individual coordinate.
  static const int kWheelbase = 0;
  static const int kTrack = 1;
  static const int kMaxAbsSteeringAngle = 2;
  static const int kMaxVelocity = 3;
  static const int kMaxAcceleration = 4;
  static const int kVelocityLookaheadTime = 5;
  static const int kVelocityKp = 6;
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class SimpleCarConfig : public systems::BasicVector<T> {
 public:
  // An abbreviation for our row index constants.
  typedef SimpleCarConfigIndices K;

  /// Default constructor.  Sets all rows to zero.
  SimpleCarConfig() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  /// @name Getters and Setters
  //@{
  // wheelbase
  const T wheelbase() const { return this->GetAtIndex(K::kWheelbase); }
  void set_wheelbase(const T& wheelbase) {
    this->SetAtIndex(K::kWheelbase, wheelbase);
  }
  // track
  const T track() const { return this->GetAtIndex(K::kTrack); }
  void set_track(const T& track) { this->SetAtIndex(K::kTrack, track); }
  // max_abs_steering_angle
  const T max_abs_steering_angle() const {
    return this->GetAtIndex(K::kMaxAbsSteeringAngle);
  }
  void set_max_abs_steering_angle(const T& max_abs_steering_angle) {
    this->SetAtIndex(K::kMaxAbsSteeringAngle, max_abs_steering_angle);
  }
  // max_velocity
  const T max_velocity() const { return this->GetAtIndex(K::kMaxVelocity); }
  void set_max_velocity(const T& max_velocity) {
    this->SetAtIndex(K::kMaxVelocity, max_velocity);
  }
  // max_acceleration
  const T max_acceleration() const {
    return this->GetAtIndex(K::kMaxAcceleration);
  }
  void set_max_acceleration(const T& max_acceleration) {
    this->SetAtIndex(K::kMaxAcceleration, max_acceleration);
  }
  // velocity_lookahead_time
  const T velocity_lookahead_time() const {
    return this->GetAtIndex(K::kVelocityLookaheadTime);
  }
  void set_velocity_lookahead_time(const T& velocity_lookahead_time) {
    this->SetAtIndex(K::kVelocityLookaheadTime, velocity_lookahead_time);
  }
  // velocity_kp
  const T velocity_kp() const { return this->GetAtIndex(K::kVelocityKp); }
  void set_velocity_kp(const T& velocity_kp) {
    this->SetAtIndex(K::kVelocityKp, velocity_kp);
  }
  //@}
};

}  // namespace automotive
}  // namespace drake
