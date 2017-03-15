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
  /// An abbreviation for our row index constants.
  typedef EndlessRoadCarStateIndices K;

  /// Default constructor.  Sets all rows to zero.
  EndlessRoadCarState() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  EndlessRoadCarState<T>* DoClone() const override {
    return new EndlessRoadCarState;
  }

  /// @name Getters and Setters
  //@{
  /// s (longitudinal) component of LANE-space position [meters]
  const T& s() const { return this->GetAtIndex(K::kS); }
  void set_s(const T& s) { this->SetAtIndex(K::kS, s); }
  /// r (lateral) component of LANE-space position [meters]
  const T& r() const { return this->GetAtIndex(K::kR); }
  void set_r(const T& r) { this->SetAtIndex(K::kR, r); }
  /// heading in the (s,r) plane [radians, 0 == +s direction]
  const T& heading() const { return this->GetAtIndex(K::kHeading); }
  void set_heading(const T& heading) { this->SetAtIndex(K::kHeading, heading); }
  /// speed in the heading direction in the (s,r) plane [m/s]
  const T& speed() const { return this->GetAtIndex(K::kSpeed); }
  void set_speed(const T& speed) { this->SetAtIndex(K::kSpeed, speed); }
  //@}

  /// Returns whether the current values of this vector are well-formed.
  decltype(T() < T()) IsValid() const {
    using std::isnan;
    auto result = (T(0) == T(0));
    result = result && !isnan(s());
    result = result && !isnan(r());
    result = result && !isnan(heading());
    result = result && !isnan(speed());
    return result;
  }
};

}  // namespace automotive
}  // namespace drake
