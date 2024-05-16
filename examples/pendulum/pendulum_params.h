#pragma once

// This file was previously auto-generated, but now is just a normal source
// file in git. However, it still retains some historical oddities from its
// heritage. In general, we do recommend against subclassing BasicVector in
// new code.

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_bool.h"
#include "drake/common/dummy_value.h"
#include "drake/common/name_value.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/symbolic/expression.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace examples {
namespace pendulum {

/// Describes the row indices of a PendulumParams.
struct PendulumParamsIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 4;

  // The index of each individual coordinate.
  static const int kMass = 0;
  static const int kLength = 1;
  static const int kDamping = 2;
  static const int kGravity = 3;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `PendulumParamsIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class PendulumParams final : public drake::systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef PendulumParamsIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c mass defaults to 1.0 kg.
  /// @arg @c length defaults to 0.5 m.
  /// @arg @c damping defaults to 0.1 kg m^2/s.
  /// @arg @c gravity defaults to 9.81 m/s^2.
  PendulumParams() : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_mass(1.0);
    this->set_length(0.5);
    this->set_damping(0.1);
    this->set_gravity(9.81);
  }

  // Note: It's safe to implement copy and move because this class is final.

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable
  //@{
  PendulumParams(const PendulumParams& other)
      : drake::systems::BasicVector<T>(other.values()) {}
  PendulumParams(PendulumParams&& other) noexcept
      : drake::systems::BasicVector<T>(std::move(other.values())) {}
  PendulumParams& operator=(const PendulumParams& other) {
    this->values() = other.values();
    return *this;
  }
  PendulumParams& operator=(PendulumParams&& other) noexcept {
    this->values() = std::move(other.values());
    other.values().resize(0);
    return *this;
  }
  //@}

  /// Create a symbolic::Variable for each element with the known variable
  /// name.  This is only available for T == symbolic::Expression.
  template <typename U = T>
  typename std::enable_if_t<std::is_same_v<U, symbolic::Expression>>
  SetToNamedVariables() {
    this->set_mass(symbolic::Variable("mass"));
    this->set_length(symbolic::Variable("length"));
    this->set_damping(symbolic::Variable("damping"));
    this->set_gravity(symbolic::Variable("gravity"));
  }

  [[nodiscard]] PendulumParams<T>* DoClone() const final {
    return new PendulumParams;
  }

  /// @name Getters and Setters
  //@{
  /// The simple pendulum has a single point mass at the end of the arm.
  /// @note @c mass is expressed in units of kg.
  /// @note @c mass has a limited domain of [0.0, +Inf].
  const T& mass() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kMass);
  }
  /// Setter that matches mass().
  void set_mass(const T& mass) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kMass, mass);
  }
  /// Fluent setter that matches mass().
  /// Returns a copy of `this` with mass set to a new value.
  [[nodiscard]] PendulumParams<T> with_mass(const T& mass) const {
    PendulumParams<T> result(*this);
    result.set_mass(mass);
    return result;
  }
  /// The length of the pendulum arm.
  /// @note @c length is expressed in units of m.
  /// @note @c length has a limited domain of [0.0, +Inf].
  const T& length() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kLength);
  }
  /// Setter that matches length().
  void set_length(const T& length) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kLength, length);
  }
  /// Fluent setter that matches length().
  /// Returns a copy of `this` with length set to a new value.
  [[nodiscard]] PendulumParams<T> with_length(const T& length) const {
    PendulumParams<T> result(*this);
    result.set_length(length);
    return result;
  }
  /// The damping friction coefficient relating angular velocity to torque.
  /// @note @c damping is expressed in units of kg m^2/s.
  /// @note @c damping has a limited domain of [0.0, +Inf].
  const T& damping() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kDamping);
  }
  /// Setter that matches damping().
  void set_damping(const T& damping) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kDamping, damping);
  }
  /// Fluent setter that matches damping().
  /// Returns a copy of `this` with damping set to a new value.
  [[nodiscard]] PendulumParams<T> with_damping(const T& damping) const {
    PendulumParams<T> result(*this);
    result.set_damping(damping);
    return result;
  }
  /// An approximate value for gravitational acceleration.
  /// @note @c gravity is expressed in units of m/s^2.
  /// @note @c gravity has a limited domain of [0.0, +Inf].
  const T& gravity() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kGravity);
  }
  /// Setter that matches gravity().
  void set_gravity(const T& gravity) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kGravity, gravity);
  }
  /// Fluent setter that matches gravity().
  /// Returns a copy of `this` with gravity set to a new value.
  [[nodiscard]] PendulumParams<T> with_gravity(const T& gravity) const {
    PendulumParams<T> result(*this);
    result.set_gravity(gravity);
    return result;
  }
  //@}

  /// Visit each field of this named vector, passing them (in order) to the
  /// given Archive.  The archive can read and/or write to the vector values.
  /// One common use of Serialize is the //common/yaml tools.
  template <typename Archive>
  void Serialize(Archive* a) {
    T& mass_ref = this->GetAtIndex(K::kMass);
    a->Visit(drake::MakeNameValue("mass", &mass_ref));
    T& length_ref = this->GetAtIndex(K::kLength);
    a->Visit(drake::MakeNameValue("length", &length_ref));
    T& damping_ref = this->GetAtIndex(K::kDamping);
    a->Visit(drake::MakeNameValue("damping", &damping_ref));
    T& gravity_ref = this->GetAtIndex(K::kGravity);
    a->Visit(drake::MakeNameValue("gravity", &gravity_ref));
  }

  /// See PendulumParamsIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return PendulumParamsIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::boolean<T> IsValid() const {
    using std::isnan;
    drake::boolean<T> result{true};
    result = result && !isnan(mass());
    result = result && (mass() >= T(0.0));
    result = result && !isnan(length());
    result = result && (length() >= T(0.0));
    result = result && !isnan(damping());
    result = result && (damping() >= T(0.0));
    result = result && !isnan(gravity());
    result = result && (gravity() >= T(0.0));
    return result;
  }

  void GetElementBounds(Eigen::VectorXd* lower,
                        Eigen::VectorXd* upper) const final {
    const double kInf = std::numeric_limits<double>::infinity();
    *lower = Eigen::Matrix<double, 4, 1>::Constant(-kInf);
    *upper = Eigen::Matrix<double, 4, 1>::Constant(kInf);
    (*lower)(K::kMass) = 0.0;
    (*lower)(K::kLength) = 0.0;
    (*lower)(K::kDamping) = 0.0;
    (*lower)(K::kGravity) = 0.0;
  }

 private:
  void ThrowIfEmpty() const {
    if (this->values().size() == 0) {
      throw std::out_of_range(
          "The PendulumParams vector has been moved-from; "
          "accessor methods may no longer be used");
    }
  }
};

}  // namespace pendulum
}  // namespace examples
}  // namespace drake
