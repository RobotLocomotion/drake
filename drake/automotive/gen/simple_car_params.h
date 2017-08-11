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

/// Describes the row indices of a SimpleCarParams.
struct SimpleCarParamsIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 6;

  // The index of each individual coordinate.
  static const int kWheelbase = 0;
  static const int kTrack = 1;
  static const int kMaxAbsSteeringAngle = 2;
  static const int kMaxVelocity = 3;
  static const int kMaxAcceleration = 4;
  static const int kVelocityLimitKp = 5;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `SimpleCarParamsIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class SimpleCarParams : public systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef SimpleCarParamsIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c wheelbase defaults to 2.700 m.
  /// @arg @c track defaults to 1.521 m.
  /// @arg @c max_abs_steering_angle defaults to 0.471 rad.
  /// @arg @c max_velocity defaults to 45.0 m/s.
  /// @arg @c max_acceleration defaults to 4.0 m/s^2.
  /// @arg @c velocity_limit_kp defaults to 10.0 Hz.
  SimpleCarParams() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_wheelbase(2.700);
    this->set_track(1.521);
    this->set_max_abs_steering_angle(0.471);
    this->set_max_velocity(45.0);
    this->set_max_acceleration(4.0);
    this->set_velocity_limit_kp(10.0);
  }

  SimpleCarParams<T>* DoClone() const override { return new SimpleCarParams; }

  /// @name Getters and Setters
  //@{
  /// The distance between the front and rear axles of the vehicle.
  /// @note @c wheelbase is expressed in units of m.
  /// @note @c wheelbase has a limited domain of [0.0, +Inf].
  const T& wheelbase() const { return this->GetAtIndex(K::kWheelbase); }
  void set_wheelbase(const T& wheelbase) {
    this->SetAtIndex(K::kWheelbase, wheelbase);
  }
  /// The distance between the center of two wheels on the same axle.
  /// @note @c track is expressed in units of m.
  /// @note @c track has a limited domain of [0.0, +Inf].
  const T& track() const { return this->GetAtIndex(K::kTrack); }
  void set_track(const T& track) { this->SetAtIndex(K::kTrack, track); }
  /// The limit on the driving_command.steering angle input (the desired
  /// steering angle of a virtual center wheel); this element is applied
  /// symmetrically to both left- and right-turn limits.
  /// @note @c max_abs_steering_angle is expressed in units of rad.
  /// @note @c max_abs_steering_angle has a limited domain of [0.0, +Inf].
  const T& max_abs_steering_angle() const {
    return this->GetAtIndex(K::kMaxAbsSteeringAngle);
  }
  void set_max_abs_steering_angle(const T& max_abs_steering_angle) {
    this->SetAtIndex(K::kMaxAbsSteeringAngle, max_abs_steering_angle);
  }
  /// The limit on the car's forward speed.
  /// @note @c max_velocity is expressed in units of m/s.
  /// @note @c max_velocity has a limited domain of [0.0, +Inf].
  const T& max_velocity() const { return this->GetAtIndex(K::kMaxVelocity); }
  void set_max_velocity(const T& max_velocity) {
    this->SetAtIndex(K::kMaxVelocity, max_velocity);
  }
  /// The limit on the car's acceleration and deceleration.
  /// @note @c max_acceleration is expressed in units of m/s^2.
  /// @note @c max_acceleration has a limited domain of [0.0, +Inf].
  const T& max_acceleration() const {
    return this->GetAtIndex(K::kMaxAcceleration);
  }
  void set_max_acceleration(const T& max_acceleration) {
    this->SetAtIndex(K::kMaxAcceleration, max_acceleration);
  }
  /// The smoothing constant for min/max velocity limits.
  /// @note @c velocity_limit_kp is expressed in units of Hz.
  /// @note @c velocity_limit_kp has a limited domain of [0.0, +Inf].
  const T& velocity_limit_kp() const {
    return this->GetAtIndex(K::kVelocityLimitKp);
  }
  void set_velocity_limit_kp(const T& velocity_limit_kp) {
    this->SetAtIndex(K::kVelocityLimitKp, velocity_limit_kp);
  }
  //@}

  /// See SimpleCarParamsIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return SimpleCarParamsIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  decltype(T() < T()) IsValid() const {
    using std::isnan;
    auto result = (T(0) == T(0));
    result = result && !isnan(wheelbase());
    result = result && (wheelbase() >= T(0.0));
    result = result && !isnan(track());
    result = result && (track() >= T(0.0));
    result = result && !isnan(max_abs_steering_angle());
    result = result && (max_abs_steering_angle() >= T(0.0));
    result = result && !isnan(max_velocity());
    result = result && (max_velocity() >= T(0.0));
    result = result && !isnan(max_acceleration());
    result = result && (max_acceleration() >= T(0.0));
    result = result && !isnan(velocity_limit_kp());
    result = result && (velocity_limit_kp() >= T(0.0));
    return result;
  }
};

}  // namespace automotive
}  // namespace drake
