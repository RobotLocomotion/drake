#pragma once

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <stdexcept>
#include <string>

#include <Eigen/Core>

#include "drake/common/drake_export.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace examples {
namespace pendulum {

/// Describes the row indices of a PendulumStateVector.
struct DRAKE_EXPORT PendulumStateVectorIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 2;

  // The index of each individual coordinate.
  static const int kTheta = 0;
  static const int kThetadot = 1;
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class PendulumStateVector : public systems::BasicVector<T> {
 public:
  // An abbreviation for our row index constants.
  typedef PendulumStateVectorIndices K;

  /// Default constructor.  Sets all rows to zero.
  PendulumStateVector() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  /// @name Getters and Setters
  //@{
  const T theta() const { return this->GetAtIndex(K::kTheta); }
  void set_theta(const T& theta) { this->SetAtIndex(K::kTheta, theta); }
  const T thetadot() const { return this->GetAtIndex(K::kThetadot); }
  void set_thetadot(const T& thetadot) {
    this->SetAtIndex(K::kThetadot, thetadot);
  }
  //@}
};

}  // namespace pendulum
}  // namespace examples
}  // namespace drake
