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

/// Describes the row indices of a TrajectoryCarParams.
struct TrajectoryCarParamsIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 2;

  // The index of each individual coordinate.
  static const int kMaxSpeed = 0;
  static const int kSpeedLimitKp = 1;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `TrajectoryCarParamsIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class TrajectoryCarParams : public systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef TrajectoryCarParamsIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c max_speed defaults to 45.0 in units of m/s.
  /// @arg @c speed_limit_kp defaults to 10.0 in units of Hz.
  TrajectoryCarParams() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_max_speed(45.0);
    this->set_speed_limit_kp(10.0);
  }

  TrajectoryCarParams<T>* DoClone() const override {
    return new TrajectoryCarParams;
  }

  /// @name Getters and Setters
  //@{
  /// The limit on the car's forward speed.
  /// @note @c max_speed is expressed in units of m/s.
  /// @note @c max_speed has a limited domain of [0.0, +Inf].
  const T& max_speed() const { return this->GetAtIndex(K::kMaxSpeed); }
  void set_max_speed(const T& max_speed) {
    this->SetAtIndex(K::kMaxSpeed, max_speed);
  }
  /// The smoothing constant for min/max speed limits.
  /// @note @c speed_limit_kp is expressed in units of Hz.
  /// @note @c speed_limit_kp has a limited domain of [0.0, +Inf].
  const T& speed_limit_kp() const { return this->GetAtIndex(K::kSpeedLimitKp); }
  void set_speed_limit_kp(const T& speed_limit_kp) {
    this->SetAtIndex(K::kSpeedLimitKp, speed_limit_kp);
  }
  //@}

  /// See TrajectoryCarParamsIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return TrajectoryCarParamsIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  decltype(T() < T()) IsValid() const {
    using std::isnan;
    auto result = (T(0) == T(0));
    result = result && !isnan(max_speed());
    result = result && (max_speed() >= T(0.0));
    result = result && !isnan(speed_limit_kp());
    result = result && (speed_limit_kp() >= T(0.0));
    return result;
  }
};

}  // namespace automotive
}  // namespace drake
