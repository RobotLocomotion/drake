#pragma once

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <cmath>
#include <stdexcept>
#include <string>

#include <Eigen/Core>

#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace examples {
namespace acrobot {

/// Describes the row indices of a AcrobotStateVector.
struct AcrobotStateVectorIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 4;

  // The index of each individual coordinate.
  static const int kTheta1 = 0;
  static const int kTheta2 = 1;
  static const int kTheta1dot = 2;
  static const int kTheta2dot = 3;
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class AcrobotStateVector : public systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef AcrobotStateVectorIndices K;

  /// Default constructor.  Sets all rows to zero.
  AcrobotStateVector() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  AcrobotStateVector<T>* DoClone() const override {
    return new AcrobotStateVector;
  }

  /// @name Getters and Setters
  //@{
  /// theta1
  const T& theta1() const { return this->GetAtIndex(K::kTheta1); }
  void set_theta1(const T& theta1) { this->SetAtIndex(K::kTheta1, theta1); }
  /// theta2
  const T& theta2() const { return this->GetAtIndex(K::kTheta2); }
  void set_theta2(const T& theta2) { this->SetAtIndex(K::kTheta2, theta2); }
  /// theta1dot
  const T& theta1dot() const { return this->GetAtIndex(K::kTheta1dot); }
  void set_theta1dot(const T& theta1dot) {
    this->SetAtIndex(K::kTheta1dot, theta1dot);
  }
  /// theta2dot
  const T& theta2dot() const { return this->GetAtIndex(K::kTheta2dot); }
  void set_theta2dot(const T& theta2dot) {
    this->SetAtIndex(K::kTheta2dot, theta2dot);
  }
  //@}

  /// Returns whether the current values of this vector are well-formed.
  decltype(T() < T()) IsValid() const {
    using std::isnan;
    auto result = (T(0) == T(0));
    result = result && !isnan(theta1());
    result = result && !isnan(theta2());
    result = result && !isnan(theta1dot());
    result = result && !isnan(theta2dot());
    return result;
  }
};

}  // namespace acrobot
}  // namespace examples
}  // namespace drake
