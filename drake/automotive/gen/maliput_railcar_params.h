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

/// Describes the row indices of a MaliputRailcarParams.
struct MaliputRailcarParamsIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 4;

  // The index of each individual coordinate.
  static const int kR = 0;
  static const int kH = 1;
  static const int kMaxSpeed = 2;
  static const int kVelocityLimitKp = 3;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `MaliputRailcarParamsIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class MaliputRailcarParams : public systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef MaliputRailcarParamsIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c r defaults to 0.0 m.
  /// @arg @c h defaults to 0.0 m.
  /// @arg @c max_speed defaults to 45.0 m/s.
  /// @arg @c velocity_limit_kp defaults to 10.0 Hz.
  MaliputRailcarParams() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_r(0.0);
    this->set_h(0.0);
    this->set_max_speed(45.0);
    this->set_velocity_limit_kp(10.0);
  }

  MaliputRailcarParams<T>* DoClone() const override {
    return new MaliputRailcarParams;
  }

  /// @name Getters and Setters
  //@{
  /// The vehicle's position on the lane's r-axis.
  /// @note @c r is expressed in units of m.
  const T& r() const { return this->GetAtIndex(K::kR); }
  void set_r(const T& r) { this->SetAtIndex(K::kR, r); }
  /// The vehicle's height above the lane's surface.
  /// @note @c h is expressed in units of m.
  const T& h() const { return this->GetAtIndex(K::kH); }
  void set_h(const T& h) { this->SetAtIndex(K::kH, h); }
  /// The limit on the vehicle's forward speed, in meters per second; this
  /// element must be positive.
  /// @note @c max_speed is expressed in units of m/s.
  const T& max_speed() const { return this->GetAtIndex(K::kMaxSpeed); }
  void set_max_speed(const T& max_speed) {
    this->SetAtIndex(K::kMaxSpeed, max_speed);
  }
  /// The smoothing constant for min/max velocity limits; this element must be
  /// positive.
  /// @note @c velocity_limit_kp is expressed in units of Hz.
  const T& velocity_limit_kp() const {
    return this->GetAtIndex(K::kVelocityLimitKp);
  }
  void set_velocity_limit_kp(const T& velocity_limit_kp) {
    this->SetAtIndex(K::kVelocityLimitKp, velocity_limit_kp);
  }
  //@}

  /// See MaliputRailcarParamsIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return MaliputRailcarParamsIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  decltype(T() < T()) IsValid() const {
    using std::isnan;
    auto result = (T(0) == T(0));
    result = result && !isnan(r());
    result = result && !isnan(h());
    result = result && !isnan(max_speed());
    result = result && !isnan(velocity_limit_kp());
    return result;
  }
};

}  // namespace automotive
}  // namespace drake
