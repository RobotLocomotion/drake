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

/// Describes the row indices of a BicycleCarState.
struct BicycleCarStateIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 6;

  // The index of each individual coordinate.
  static const int kPsi = 0;
  static const int kPsiDot = 1;
  static const int kBeta = 2;
  static const int kVel = 3;
  static const int kSx = 4;
  static const int kSy = 5;
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class BicycleCarState : public systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef BicycleCarStateIndices K;

  /// Default constructor.  Sets all rows to zero.
  BicycleCarState() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  BicycleCarState<T>* DoClone() const override { return new BicycleCarState; }

  /// @name Getters and Setters
  //@{
  /// yaw angle
  const T& Psi() const { return this->GetAtIndex(K::kPsi); }
  void set_Psi(const T& Psi) { this->SetAtIndex(K::kPsi, Psi); }
  /// yaw angular rate
  const T& Psi_dot() const { return this->GetAtIndex(K::kPsiDot); }
  void set_Psi_dot(const T& Psi_dot) { this->SetAtIndex(K::kPsiDot, Psi_dot); }
  /// slip angle at the center of mass
  const T& beta() const { return this->GetAtIndex(K::kBeta); }
  void set_beta(const T& beta) { this->SetAtIndex(K::kBeta, beta); }
  /// velocity magnitude
  const T& vel() const { return this->GetAtIndex(K::kVel); }
  void set_vel(const T& vel) { this->SetAtIndex(K::kVel, vel); }
  /// x-position at the center of mass
  const T& sx() const { return this->GetAtIndex(K::kSx); }
  void set_sx(const T& sx) { this->SetAtIndex(K::kSx, sx); }
  /// y-position at the center of mass
  const T& sy() const { return this->GetAtIndex(K::kSy); }
  void set_sy(const T& sy) { this->SetAtIndex(K::kSy, sy); }
  //@}

  /// Returns whether the current values of this vector are well-formed.
  decltype(T() < T()) IsValid() const {
    using std::isnan;
    auto result = (T(0) == T(0));
    result = result && !isnan(Psi());
    result = result && !isnan(Psi_dot());
    result = result && !isnan(beta());
    result = result && !isnan(vel());
    result = result && !isnan(sx());
    result = result && !isnan(sy());
    return result;
  }
};

}  // namespace automotive
}  // namespace drake
