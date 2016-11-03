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

/// Describes the row indices of a IdmPlannerInput.
struct DRAKE_EXPORT IdmPlannerInputIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 2;

  // The index of each individual coordinate.
  static const int kX = 0;
  static const int kV = 1;
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class IdmPlannerInput : public systems::BasicVector<T> {
 public:
  // An abbreviation for our row index constants.
  typedef IdmPlannerInputIndices K;

  /// Default constructor.  Sets all rows to zero.
  IdmPlannerInput() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  /// @name Getters and Setters
  //@{
  // x
  const T x() const { return this->GetAtIndex(K::kX); }
  void set_x(const T& x) { this->SetAtIndex(K::kX, x); }
  // v
  const T v() const { return this->GetAtIndex(K::kV); }
  void set_v(const T& v) { this->SetAtIndex(K::kV, v); }
  //@}
};

}  // namespace automotive
}  // namespace drake
