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

/// Describes the row indices of a SimpleCarState.
struct SimpleCarStateIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 4;

  // The index of each individual coordinate.
  static const int kX = 0;
  static const int kY = 1;
  static const int kHeading = 2;
  static const int kVelocity = 3;
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class SimpleCarState : public systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef SimpleCarStateIndices K;

  /// Default constructor.  Sets all rows to zero.
  SimpleCarState() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  SimpleCarState<T>* DoClone() const override { return new SimpleCarState; }

  /// @name Getters and Setters
  //@{
  /// x
  const T& x() const { return this->GetAtIndex(K::kX); }
  void set_x(const T& x) { this->SetAtIndex(K::kX, x); }
  /// y
  const T& y() const { return this->GetAtIndex(K::kY); }
  void set_y(const T& y) { this->SetAtIndex(K::kY, y); }
  /// heading
  const T& heading() const { return this->GetAtIndex(K::kHeading); }
  void set_heading(const T& heading) { this->SetAtIndex(K::kHeading, heading); }
  /// velocity
  const T& velocity() const { return this->GetAtIndex(K::kVelocity); }
  void set_velocity(const T& velocity) {
    this->SetAtIndex(K::kVelocity, velocity);
  }
  //@}

  /// Returns whether the current values of this vector are well-formed.
  decltype(T() < T()) IsValid() const {
    using std::isnan;
    auto result = (T(0) == T(0));
    result = result && !isnan(x());
    result = result && !isnan(y());
    result = result && !isnan(heading());
    result = result && !isnan(velocity());
    return result;
  }
};

}  // namespace automotive
}  // namespace drake
