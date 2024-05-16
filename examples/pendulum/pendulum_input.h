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

/// Describes the row indices of a PendulumInput.
struct PendulumInputIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 1;

  // The index of each individual coordinate.
  static const int kTau = 0;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `PendulumInputIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class PendulumInput final : public drake::systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef PendulumInputIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c tau defaults to 0.0 Newton-meters.
  PendulumInput() : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_tau(0.0);
  }

  // Note: It's safe to implement copy and move because this class is final.

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable
  //@{
  PendulumInput(const PendulumInput& other)
      : drake::systems::BasicVector<T>(other.values()) {}
  PendulumInput(PendulumInput&& other) noexcept
      : drake::systems::BasicVector<T>(std::move(other.values())) {}
  PendulumInput& operator=(const PendulumInput& other) {
    this->values() = other.values();
    return *this;
  }
  PendulumInput& operator=(PendulumInput&& other) noexcept {
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
    this->set_tau(symbolic::Variable("tau"));
  }

  [[nodiscard]] PendulumInput<T>* DoClone() const final {
    return new PendulumInput;
  }

  /// @name Getters and Setters
  //@{
  /// Torque at the joint.
  /// @note @c tau is expressed in units of Newton-meters.
  const T& tau() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kTau);
  }
  /// Setter that matches tau().
  void set_tau(const T& tau) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kTau, tau);
  }
  /// Fluent setter that matches tau().
  /// Returns a copy of `this` with tau set to a new value.
  [[nodiscard]] PendulumInput<T> with_tau(const T& tau) const {
    PendulumInput<T> result(*this);
    result.set_tau(tau);
    return result;
  }
  //@}

  /// Visit each field of this named vector, passing them (in order) to the
  /// given Archive.  The archive can read and/or write to the vector values.
  /// One common use of Serialize is the //common/yaml tools.
  template <typename Archive>
  void Serialize(Archive* a) {
    T& tau_ref = this->GetAtIndex(K::kTau);
    a->Visit(drake::MakeNameValue("tau", &tau_ref));
  }

  /// See PendulumInputIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return PendulumInputIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::boolean<T> IsValid() const {
    using std::isnan;
    drake::boolean<T> result{true};
    result = result && !isnan(tau());
    return result;
  }

 private:
  void ThrowIfEmpty() const {
    if (this->values().size() == 0) {
      throw std::out_of_range(
          "The PendulumInput vector has been moved-from; "
          "accessor methods may no longer be used");
    }
  }
};

}  // namespace pendulum
}  // namespace examples
}  // namespace drake
