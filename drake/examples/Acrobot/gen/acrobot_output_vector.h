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

/// Describes the row indices of a AcrobotOutputVector.
struct AcrobotOutputVectorIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 2;

  // The index of each individual coordinate.
  static const int kTheta1 = 0;
  static const int kTheta2 = 1;
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class AcrobotOutputVector : public systems::BasicVector<T> {
 public:
  // An abbreviation for our row index constants.
  typedef AcrobotOutputVectorIndices K;

  /// Default constructor.  Sets all rows to zero.
  AcrobotOutputVector() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  /// @name Getters and Setters
  //@{
  // theta1
  const T& theta1() const { return this->GetAtIndex(K::kTheta1); }
  void set_theta1(const T& theta1) { this->SetAtIndex(K::kTheta1, theta1); }
  // theta2
  const T& theta2() const { return this->GetAtIndex(K::kTheta2); }
  void set_theta2(const T& theta2) { this->SetAtIndex(K::kTheta2, theta2); }
  //@}
};

}  // namespace acrobot
}  // namespace examples
}  // namespace drake
