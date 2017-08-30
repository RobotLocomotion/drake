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

/// Describes the row indices of a PendulumInput.
struct PendulumInputIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 1;

  // The index of each individual coordinate.
  static const int kTau = 0;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `PendulumInputIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class PendulumInput : public systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef PendulumInputIndices K;

  /// Default constructor.  Sets all rows to zero.
  PendulumInput() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  PendulumInput<T>* DoClone() const override { return new PendulumInput; }

  /// @name Getters and Setters
  //@{
  /// tau
  const T& tau() const { return this->GetAtIndex(K::kTau); }
  void set_tau(const T& tau) { this->SetAtIndex(K::kTau, tau); }
  //@}

  /// See PendulumInputIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return PendulumInputIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  decltype(T() < T()) IsValid() const {
    using std::isnan;
    auto result = (T(0) == T(0));
    result = result && !isnan(tau());
    return result;
  }
};

}  // namespace pendulum
}  // namespace examples
}  // namespace drake
