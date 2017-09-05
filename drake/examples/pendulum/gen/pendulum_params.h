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
namespace examples {
namespace pendulum {

/// Describes the row indices of a PendulumParams.
struct PendulumParamsIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 4;

  // The index of each individual coordinate.
  static const int kMass = 0;
  static const int kLength = 1;
  static const int kDamping = 2;
  static const int kGravity = 3;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `PendulumParamsIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class PendulumParams : public systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef PendulumParamsIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c mass defaults to 1.0 kg.
  /// @arg @c length defaults to 0.5 m.
  /// @arg @c damping defaults to 0.1 kg m^2/s.
  /// @arg @c gravity defaults to 9.81 m/s^2.
  PendulumParams() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_mass(1.0);
    this->set_length(0.5);
    this->set_damping(0.1);
    this->set_gravity(9.81);
  }

  PendulumParams<T>* DoClone() const override { return new PendulumParams; }

  /// @name Getters and Setters
  //@{
  /// The simple pendulum has a single point mass at the end of the arm.
  /// @note @c mass is expressed in units of kg.
  /// @note @c mass has a limited domain of [0.0, +Inf].
  const T& mass() const { return this->GetAtIndex(K::kMass); }
  void set_mass(const T& mass) { this->SetAtIndex(K::kMass, mass); }
  /// The length of the pendulum arm.
  /// @note @c length is expressed in units of m.
  /// @note @c length has a limited domain of [0.0, +Inf].
  const T& length() const { return this->GetAtIndex(K::kLength); }
  void set_length(const T& length) { this->SetAtIndex(K::kLength, length); }
  /// The damping friction coefficient relating angular velocity to torque.
  /// @note @c damping is expressed in units of kg m^2/s.
  /// @note @c damping has a limited domain of [0.0, +Inf].
  const T& damping() const { return this->GetAtIndex(K::kDamping); }
  void set_damping(const T& damping) { this->SetAtIndex(K::kDamping, damping); }
  /// An approximate value for gravitational acceleration.
  /// @note @c gravity is expressed in units of m/s^2.
  /// @note @c gravity has a limited domain of [0.0, +Inf].
  const T& gravity() const { return this->GetAtIndex(K::kGravity); }
  void set_gravity(const T& gravity) { this->SetAtIndex(K::kGravity, gravity); }
  //@}

  /// See PendulumParamsIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return PendulumParamsIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  decltype(T() < T()) IsValid() const {
    using std::isnan;
    auto result = (T(0) == T(0));
    result = result && !isnan(mass());
    result = result && (mass() >= T(0.0));
    result = result && !isnan(length());
    result = result && (length() >= T(0.0));
    result = result && !isnan(damping());
    result = result && (damping() >= T(0.0));
    result = result && !isnan(gravity());
    result = result && (gravity() >= T(0.0));
    return result;
  }
};

}  // namespace pendulum
}  // namespace examples
}  // namespace drake
