#pragma once

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <stdexcept>
#include <string>

#include <Eigen/Core>

#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace automotive {

/// Describes the row indices of a BicycleParameters.
struct BicycleParametersIndices {
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
class BicycleParameters : public systems::BasicVector<T> {
 public:
  // An abbreviation for our row index constants.
  typedef BicycleParametersIndices K;

  /// Default constructor.  Sets all rows to zero.
  BicycleParameters() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  BicycleParameters<T>* DoClone() const override {
    auto result = new BicycleParameters;
    result->set_value(this->get_value());
    return result;
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
};

}  // namespace automotive
}  // namespace drake
