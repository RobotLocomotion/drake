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

/// Describes the row indices of a BicycleCarParameters.
struct BicycleCarParametersIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 6;

  // The index of each individual coordinate.
  static const int kMass = 0;
  static const int kLf = 1;
  static const int kLr = 2;
  static const int kIz = 3;
  static const int kCf = 4;
  static const int kCr = 5;
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class BicycleCarParameters : public systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef BicycleCarParametersIndices K;

  /// Default constructor.  Sets all rows to zero.
  BicycleCarParameters() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  BicycleCarParameters<T>* DoClone() const override {
    return new BicycleCarParameters;
  }

  /// @name Getters and Setters
  //@{
  /// mass
  const T& mass() const { return this->GetAtIndex(K::kMass); }
  void set_mass(const T& mass) { this->SetAtIndex(K::kMass, mass); }
  /// distance from the center of mass to the front axle
  const T& lf() const { return this->GetAtIndex(K::kLf); }
  void set_lf(const T& lf) { this->SetAtIndex(K::kLf, lf); }
  /// distance from the center of mass to the rear axle
  const T& lr() const { return this->GetAtIndex(K::kLr); }
  void set_lr(const T& lr) { this->SetAtIndex(K::kLr, lr); }
  /// moment of inertia about the yaw-axis
  const T& Iz() const { return this->GetAtIndex(K::kIz); }
  void set_Iz(const T& Iz) { this->SetAtIndex(K::kIz, Iz); }
  /// cornering stiffness (front)
  const T& Cf() const { return this->GetAtIndex(K::kCf); }
  void set_Cf(const T& Cf) { this->SetAtIndex(K::kCf, Cf); }
  /// cornering stiffness (rear)
  const T& Cr() const { return this->GetAtIndex(K::kCr); }
  void set_Cr(const T& Cr) { this->SetAtIndex(K::kCr, Cr); }
  //@}

  /// Returns whether the current values of this vector are well-formed.
  decltype(T() < T()) IsValid() const {
    using std::isnan;
    auto result = (T(0) == T(0));
    result = result && !isnan(mass());
    result = result && !isnan(lf());
    result = result && !isnan(lr());
    result = result && !isnan(Iz());
    result = result && !isnan(Cf());
    result = result && !isnan(Cr());
    return result;
  }
};

}  // namespace automotive
}  // namespace drake
