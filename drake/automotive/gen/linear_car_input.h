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

/// Describes the row indices of a LinearCarInput.
struct DRAKE_EXPORT LinearCarInputIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 1;

  // The index of each individual coordinate.
  static const int kVdot = 0;
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class LinearCarInput : public systems::BasicVector<T> {
 public:
  // An abbreviation for our row index constants.
  typedef LinearCarInputIndices K;

  /// Default constructor.  Sets all rows to zero.
  LinearCarInput() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  /// @name Getters and Setters
  //@{
  // vdot
  const T vdot() const { return this->GetAtIndex(K::kVdot); }
  void set_vdot(const T& vdot) { this->SetAtIndex(K::kVdot, vdot); }
  //@}
};

}  // namespace automotive
}  // namespace drake
