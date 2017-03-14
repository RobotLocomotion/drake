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

/// Describes the row indices of a MaliputRailcarConfig.
struct MaliputRailcarConfigIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 5;

  // The index of each individual coordinate.
  static const int kR = 0;
  static const int kH = 1;
  static const int kInitialSpeed = 2;
  static const int kMaxSpeed = 3;
  static const int kVelocityLimitKp = 4;
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class MaliputRailcarConfig : public systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef MaliputRailcarConfigIndices K;

  /// Default constructor.  Sets all rows to zero.
  MaliputRailcarConfig() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  MaliputRailcarConfig<T>* DoClone() const override {
    return new MaliputRailcarConfig;
  }

  /// @name Getters and Setters
  //@{
  /// The vehicle's position on the lane's r-axis.
  const T& r() const { return this->GetAtIndex(K::kR); }
  void set_r(const T& r) { this->SetAtIndex(K::kR, r); }
  /// The vehicle's height above the lane's surface.
  const T& h() const { return this->GetAtIndex(K::kH); }
  void set_h(const T& h) { this->SetAtIndex(K::kH, h); }
  /// The initial speed of the vehicle. This must be positive.
  const T& initial_speed() const { return this->GetAtIndex(K::kInitialSpeed); }
  void set_initial_speed(const T& initial_speed) {
    this->SetAtIndex(K::kInitialSpeed, initial_speed);
  }
  /// The limit on the vehicle's forward speed, in meters per second; this
  /// element must be positive.
  const T& max_speed() const { return this->GetAtIndex(K::kMaxSpeed); }
  void set_max_speed(const T& max_speed) {
    this->SetAtIndex(K::kMaxSpeed, max_speed);
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
    result = result && !isnan(r());
    result = result && !isnan(h());
    result = result && !isnan(initial_speed());
    result = result && !isnan(max_speed());
    result = result && !isnan(velocity_limit_kp());
    return result;
  }
};

}  // namespace automotive
}  // namespace drake
