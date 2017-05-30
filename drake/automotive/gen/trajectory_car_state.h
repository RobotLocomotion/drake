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
namespace automotive {

/// Describes the row indices of a TrajectoryCarState.
struct TrajectoryCarStateIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 2;

  // The index of each individual coordinate.
  static const int kPosition = 0;
  static const int kSpeed = 1;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `TrajectoryCarStateIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class TrajectoryCarState : public systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef TrajectoryCarStateIndices K;

  /// Default constructor.  Sets all rows to zero.
  TrajectoryCarState() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  TrajectoryCarState<T>* DoClone() const override {
    return new TrajectoryCarState;
  }

  /// @name Getters and Setters
  //@{
  /// The along-curve position of the vehicle.
  const T& position() const { return this->GetAtIndex(K::kPosition); }
  void set_position(const T& position) {
    this->SetAtIndex(K::kPosition, position);
  }
  /// The along-curve speed of the vehicle.
  const T& speed() const { return this->GetAtIndex(K::kSpeed); }
  void set_speed(const T& speed) { this->SetAtIndex(K::kSpeed, speed); }
  //@}

  /// See TrajectoryCarStateIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return TrajectoryCarStateIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  decltype(T() < T()) IsValid() const {
    using std::isnan;
    auto result = (T(0) == T(0));
    result = result && !isnan(position());
    result = result && !isnan(speed());
    return result;
  }
};

}  // namespace automotive
}  // namespace drake
