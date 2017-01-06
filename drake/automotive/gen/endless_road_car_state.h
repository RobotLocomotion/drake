#pragma once

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <stdexcept>
#include <string>

#include <Eigen/Core>

#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace automotive {

/// Describes the row indices of a EndlessRoadCarState.
struct EndlessRoadCarStateIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 4;

  // The index of each individual coordinate.
  static const int kS = 0;
  static const int kR = 1;
  static const int kHeading = 2;
  static const int kSpeed = 3;
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class EndlessRoadCarState : public systems::BasicVector<T> {
 public:
  // An abbreviation for our row index constants.
  typedef EndlessRoadCarStateIndices K;

  /// Default constructor.  Sets all rows to zero.
  EndlessRoadCarState() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  /// @name Getters and Setters
  //@{
  /// s
  const T& s() const { return this->GetAtIndex(K::kS); }
  void set_s(const T& s) { this->SetAtIndex(K::kS, s); }
  /// r
  const T& r() const { return this->GetAtIndex(K::kR); }
  void set_r(const T& r) { this->SetAtIndex(K::kR, r); }
  /// heading
  const T& heading() const { return this->GetAtIndex(K::kHeading); }
  void set_heading(const T& heading) { this->SetAtIndex(K::kHeading, heading); }
  /// speed
  const T& speed() const { return this->GetAtIndex(K::kSpeed); }
  void set_speed(const T& speed) { this->SetAtIndex(K::kSpeed, speed); }
  //@}
};

}  // namespace automotive
}  // namespace drake
