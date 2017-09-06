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

/// Describes the row indices of a PendulumState.
struct PendulumStateIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 2;

  // The index of each individual coordinate.
  static const int kTheta = 0;
  static const int kThetadot = 1;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `PendulumStateIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class PendulumState : public systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef PendulumStateIndices K;

  /// Default constructor.  Sets all rows to zero.
  PendulumState() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  PendulumState<T>* DoClone() const override { return new PendulumState; }

  /// @name Getters and Setters
  //@{
  /// theta
  const T& theta() const { return this->GetAtIndex(K::kTheta); }
  void set_theta(const T& theta) { this->SetAtIndex(K::kTheta, theta); }
  /// thetadot
  const T& thetadot() const { return this->GetAtIndex(K::kThetadot); }
  void set_thetadot(const T& thetadot) {
    this->SetAtIndex(K::kThetadot, thetadot);
  }
  //@}

  /// See PendulumStateIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return PendulumStateIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  decltype(T() < T()) IsValid() const {
    using std::isnan;
    auto result = (T(0) == T(0));
    result = result && !isnan(theta());
    result = result && !isnan(thetadot());
    return result;
  }
};

}  // namespace pendulum
}  // namespace examples
}  // namespace drake
