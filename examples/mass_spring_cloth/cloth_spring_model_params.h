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
namespace mass_spring_cloth {

/// Describes the row indices of a ClothSpringModelParams.
struct ClothSpringModelParamsIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 4;

  // The index of each individual coordinate.
  static const int kMass = 0;
  static const int kK = 1;
  static const int kD = 2;
  static const int kGravity = 3;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `ClothSpringModelParamsIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class ClothSpringModelParams final : public drake::systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef ClothSpringModelParamsIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c mass defaults to 1.0 kg.
  /// @arg @c k defaults to 100.0 N/m.
  /// @arg @c d defaults to 10 Ns/m.
  /// @arg @c gravity defaults to -9.81 m/s^2.
  ClothSpringModelParams()
      : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_mass(1.0);
    this->set_k(100.0);
    this->set_d(10);
    this->set_gravity(-9.81);
  }

  // Note: It's safe to implement copy and move because this class is final.

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable
  //@{
  ClothSpringModelParams(const ClothSpringModelParams& other)
      : drake::systems::BasicVector<T>(other.values()) {}
  ClothSpringModelParams(ClothSpringModelParams&& other) noexcept
      : drake::systems::BasicVector<T>(std::move(other.values())) {}
  ClothSpringModelParams& operator=(const ClothSpringModelParams& other) {
    this->values() = other.values();
    return *this;
  }
  ClothSpringModelParams& operator=(ClothSpringModelParams&& other) noexcept {
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
    this->set_k(symbolic::Variable("k"));
    this->set_d(symbolic::Variable("d"));
    this->set_gravity(symbolic::Variable("gravity"));
  }

  [[nodiscard]] ClothSpringModelParams<T>* DoClone() const final {
    return new ClothSpringModelParams;
  }

  /// @name Getters and Setters
  //@{
  /// Mass of the entire system.
  /// @note @c mass is expressed in units of kg.
  /// @note @c mass has a limited domain of [0.001, +Inf].
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
  [[nodiscard]] ClothSpringModelParams<T> with_mass(const T& mass) const {
    ClothSpringModelParams<T> result(*this);
    result.set_mass(mass);
    return result;
  }
  /// Elastic stiffness of the springs.
  /// @note @c k is expressed in units of N/m.
  /// @note @c k has a limited domain of [0.0, +Inf].
  const T& k() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kK);
  }
  /// Setter that matches k().
  void set_k(const T& k) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kK, k);
  }
  /// Fluent setter that matches k().
  /// Returns a copy of `this` with k set to a new value.
  [[nodiscard]] ClothSpringModelParams<T> with_k(const T& k) const {
    ClothSpringModelParams<T> result(*this);
    result.set_k(k);
    return result;
  }
  /// Damping coefficient of the springs.
  /// @note @c d is expressed in units of Ns/m.
  /// @note @c d has a limited domain of [0.0, +Inf].
  const T& d() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kD);
  }
  /// Setter that matches d().
  void set_d(const T& d) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kD, d);
  }
  /// Fluent setter that matches d().
  /// Returns a copy of `this` with d set to a new value.
  [[nodiscard]] ClothSpringModelParams<T> with_d(const T& d) const {
    ClothSpringModelParams<T> result(*this);
    result.set_d(d);
    return result;
  }
  /// Gravitational constant.
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
  [[nodiscard]] ClothSpringModelParams<T> with_gravity(const T& gravity) const {
    ClothSpringModelParams<T> result(*this);
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
    T& k_ref = this->GetAtIndex(K::kK);
    a->Visit(drake::MakeNameValue("k", &k_ref));
    T& d_ref = this->GetAtIndex(K::kD);
    a->Visit(drake::MakeNameValue("d", &d_ref));
    T& gravity_ref = this->GetAtIndex(K::kGravity);
    a->Visit(drake::MakeNameValue("gravity", &gravity_ref));
  }

  /// See ClothSpringModelParamsIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return ClothSpringModelParamsIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::boolean<T> IsValid() const {
    using std::isnan;
    drake::boolean<T> result{true};
    result = result && !isnan(mass());
    result = result && (mass() >= T(0.001));
    result = result && !isnan(k());
    result = result && (k() >= T(0.0));
    result = result && !isnan(d());
    result = result && (d() >= T(0.0));
    result = result && !isnan(gravity());
    result = result && (gravity() >= T(0.0));
    return result;
  }

  void GetElementBounds(Eigen::VectorXd* lower,
                        Eigen::VectorXd* upper) const final {
    const double kInf = std::numeric_limits<double>::infinity();
    *lower = Eigen::Matrix<double, 4, 1>::Constant(-kInf);
    *upper = Eigen::Matrix<double, 4, 1>::Constant(kInf);
    (*lower)(K::kMass) = 0.001;
    (*lower)(K::kK) = 0.0;
    (*lower)(K::kD) = 0.0;
    (*lower)(K::kGravity) = 0.0;
  }

 private:
  void ThrowIfEmpty() const {
    if (this->values().size() == 0) {
      throw std::out_of_range(
          "The ClothSpringModelParams vector has been moved-from; "
          "accessor methods may no longer be used");
    }
  }
};

}  // namespace mass_spring_cloth
}  // namespace examples
}  // namespace drake
