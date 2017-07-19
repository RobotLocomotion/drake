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
namespace bouncing_ball {

/// Describes the row indices of a BouncingBallVector.
struct BouncingBallVectorIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 2;

  // The index of each individual coordinate.
static const int kZ = 0;
static const int kZdot = 1;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `BouncingBallVectorIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class BouncingBallVector : public systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef BouncingBallVectorIndices K;

  /// Default constructor.  Sets all rows to zero.
  BouncingBallVector() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  BouncingBallVector<T>* DoClone() const override {
    return new BouncingBallVector;
  }

  /// @name Getters and Setters
  //@{
  /// z
  const T& z() const { return this->GetAtIndex(K::kZ); }
  void set_z(const T& z) {
    this->SetAtIndex(K::kZ, z);
  }
  /// zdot
  const T& zdot() const { return this->GetAtIndex(K::kZdot); }
  void set_zdot(const T& zdot) {
    this->SetAtIndex(K::kZdot, zdot);
  }
  //@}

  /// See BouncingBallVectorIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return BouncingBallVectorIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  decltype(T() < T()) IsValid() const {
    using std::isnan;
    auto result = (T(0) == T(0));
    result = result && !isnan(z());
    result = result && !isnan(zdot());
    return result;
  }
};

}  // namespace bouncing_ball
}  // namespace examples
}  // namespace drake
