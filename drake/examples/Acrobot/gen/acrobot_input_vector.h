#pragma once

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <stdexcept>
#include <string>

#include <Eigen/Core>

#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace examples {
namespace acrobot {

/// Describes the row indices of a AcrobotInputVector.
struct AcrobotInputVectorIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 1;

  // The index of each individual coordinate.
  static const int kTau = 0;
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class AcrobotInputVector : public systems::BasicVector<T> {
 public:
  // An abbreviation for our row index constants.
  typedef AcrobotInputVectorIndices K;

  /// Default constructor.  Sets all rows to zero.
  AcrobotInputVector() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  /// @name Getters and Setters
  //@{
  // tau
  const T& tau() const { return this->GetAtIndex(K::kTau); }
  void set_tau(const T& tau) { this->SetAtIndex(K::kTau, tau); }
  //@}
};

}  // namespace acrobot
}  // namespace examples
}  // namespace drake
