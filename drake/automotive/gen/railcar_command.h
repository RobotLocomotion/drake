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

/// Describes the row indices of a RailcarCommand.
struct RailcarCommandIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 2;

  // The index of each individual coordinate.
  static const int kThrottle = 0;
  static const int kBrake = 1;
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class RailcarCommand : public systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef RailcarCommandIndices K;

  /// Default constructor.  Sets all rows to zero.
  RailcarCommand() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  RailcarCommand<T>* DoClone() const override { return new RailcarCommand; }

  /// @name Getters and Setters
  //@{
  /// The normalized desired acceleration (in the range [0.0..1.0]).
  const T& throttle() const { return this->GetAtIndex(K::kThrottle); }
  void set_throttle(const T& throttle) {
    this->SetAtIndex(K::kThrottle, throttle);
  }
  /// The normalized desired deceleration (in the range [0.0..1.0]).
  const T& brake() const { return this->GetAtIndex(K::kBrake); }
  void set_brake(const T& brake) { this->SetAtIndex(K::kBrake, brake); }
  //@}

  /// Returns whether the current values of this vector are well-formed.
  decltype(T() < T()) IsValid() const {
    using std::isnan;
    auto result = (T(0) == T(0));
    result = result && !isnan(throttle());
    result = result && !isnan(brake());
    return result;
  }
};

}  // namespace automotive
}  // namespace drake
