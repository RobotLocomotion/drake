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

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `BicycleCarParametersIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class BicycleCarParameters : public systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef BicycleCarParametersIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c mass defaults to 2278.0 kg.
  /// @arg @c lf defaults to 1.292 m.
  /// @arg @c lr defaults to 1.515 m.
  /// @arg @c Iz defaults to 3210.0 kg m^2.
  /// @arg @c Cf defaults to 10.8e4 N / rad.
  /// @arg @c Cr defaults to 10.8e4 N / rad.
  BicycleCarParameters() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_mass(2278.0);
    this->set_lf(1.292);
    this->set_lr(1.515);
    this->set_Iz(3210.0);
    this->set_Cf(10.8e4);
    this->set_Cr(10.8e4);
  }

  BicycleCarParameters<T>* DoClone() const override {
    return new BicycleCarParameters;
  }

  /// @name Getters and Setters
  //@{
  /// mass
  /// @note @c mass is expressed in units of kg.
  /// @note @c mass has a limited domain of [0.0, +Inf].
  const T& mass() const { return this->GetAtIndex(K::kMass); }
  void set_mass(const T& mass) { this->SetAtIndex(K::kMass, mass); }
  /// distance from the center of mass to the front axle
  /// @note @c lf is expressed in units of m.
  /// @note @c lf has a limited domain of [0.0, +Inf].
  const T& lf() const { return this->GetAtIndex(K::kLf); }
  void set_lf(const T& lf) { this->SetAtIndex(K::kLf, lf); }
  /// distance from the center of mass to the rear axle
  /// @note @c lr is expressed in units of m.
  /// @note @c lr has a limited domain of [0.0, +Inf].
  const T& lr() const { return this->GetAtIndex(K::kLr); }
  void set_lr(const T& lr) { this->SetAtIndex(K::kLr, lr); }
  /// moment of inertia about the yaw-axis
  /// @note @c Iz is expressed in units of kg m^2.
  /// @note @c Iz has a limited domain of [0.0, +Inf].
  const T& Iz() const { return this->GetAtIndex(K::kIz); }
  void set_Iz(const T& Iz) { this->SetAtIndex(K::kIz, Iz); }
  /// cornering stiffness (front)
  /// @note @c Cf is expressed in units of N / rad.
  /// @note @c Cf has a limited domain of [0.0, +Inf].
  const T& Cf() const { return this->GetAtIndex(K::kCf); }
  void set_Cf(const T& Cf) { this->SetAtIndex(K::kCf, Cf); }
  /// cornering stiffness (rear)
  /// @note @c Cr is expressed in units of N / rad.
  /// @note @c Cr has a limited domain of [0.0, +Inf].
  const T& Cr() const { return this->GetAtIndex(K::kCr); }
  void set_Cr(const T& Cr) { this->SetAtIndex(K::kCr, Cr); }
  //@}

  /// See BicycleCarParametersIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return BicycleCarParametersIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  decltype(T() < T()) IsValid() const {
    using std::isnan;
    auto result = (T(0) == T(0));
    result = result && !isnan(mass());
    result = result && (mass() >= T(0.0));
    result = result && !isnan(lf());
    result = result && (lf() >= T(0.0));
    result = result && !isnan(lr());
    result = result && (lr() >= T(0.0));
    result = result && !isnan(Iz());
    result = result && (Iz() >= T(0.0));
    result = result && !isnan(Cf());
    result = result && (Cf() >= T(0.0));
    result = result && !isnan(Cr());
    result = result && (Cr() >= T(0.0));
    return result;
  }
};

}  // namespace automotive
}  // namespace drake
