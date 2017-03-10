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

/// Describes the row indices of a MaliputRailcarState.
struct MaliputRailcarStateIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 2;

  // The index of each individual coordinate.
  static const int kS = 0;
  static const int kSpeed = 1;
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class MaliputRailcarState : public systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef MaliputRailcarStateIndices K;

  /// Default constructor.  Sets all rows to zero.
  MaliputRailcarState() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  MaliputRailcarState<T>* DoClone() const override {
    return new MaliputRailcarState;
  }

  /// @name Getters and Setters
  //@{
  /// The s-coordinate of the vehicle in lane-space.
  const T& s() const { return this->GetAtIndex(K::kS); }
  void set_s(const T& s) { this->SetAtIndex(K::kS, s); }
  /// The speed of the vehicle in physical space.
  const T& speed() const { return this->GetAtIndex(K::kSpeed); }
  void set_speed(const T& speed) { this->SetAtIndex(K::kSpeed, speed); }
  //@}

  /// Returns whether the current values of this vector are well-formed.
  decltype(T() < T()) IsValid() const {
    using std::isnan;
    auto result = (T(0) == T(0));
    result = result && !isnan(s());
    result = result && !isnan(speed());
    return result;
  }
};

}  // namespace automotive
}  // namespace drake
