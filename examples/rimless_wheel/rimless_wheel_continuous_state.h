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
namespace rimless_wheel {

/// Describes the row indices of a RimlessWheelContinuousState.
struct RimlessWheelContinuousStateIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 2;

  // The index of each individual coordinate.
  static const int kTheta = 0;
  static const int kThetadot = 1;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words,
  /// `RimlessWheelContinuousStateIndices::GetCoordinateNames()[i]` is the name
  /// for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class RimlessWheelContinuousState final
    : public drake::systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef RimlessWheelContinuousStateIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c theta defaults to 0.0 radians.
  /// @arg @c thetadot defaults to 0.0 rad/sec.
  RimlessWheelContinuousState()
      : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_theta(0.0);
    this->set_thetadot(0.0);
  }

  // Note: It's safe to implement copy and move because this class is final.

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable
  //@{
  RimlessWheelContinuousState(const RimlessWheelContinuousState& other)
      : drake::systems::BasicVector<T>(other.values()) {}
  RimlessWheelContinuousState(RimlessWheelContinuousState&& other) noexcept
      : drake::systems::BasicVector<T>(std::move(other.values())) {}
  RimlessWheelContinuousState& operator=(
      const RimlessWheelContinuousState& other) {
    this->values() = other.values();
    return *this;
  }
  RimlessWheelContinuousState& operator=(
      RimlessWheelContinuousState&& other) noexcept {
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
    this->set_theta(symbolic::Variable("theta"));
    this->set_thetadot(symbolic::Variable("thetadot"));
  }

  [[nodiscard]] RimlessWheelContinuousState<T>* DoClone() const final {
    return new RimlessWheelContinuousState;
  }

  /// @name Getters and Setters
  //@{
  /// The orientation of the stance leg, measured clockwise from the vertical
  /// axis.
  /// @note @c theta is expressed in units of radians.
  const T& theta() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kTheta);
  }
  /// Setter that matches theta().
  void set_theta(const T& theta) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kTheta, theta);
  }
  /// Fluent setter that matches theta().
  /// Returns a copy of `this` with theta set to a new value.
  [[nodiscard]] RimlessWheelContinuousState<T> with_theta(
      const T& theta) const {
    RimlessWheelContinuousState<T> result(*this);
    result.set_theta(theta);
    return result;
  }
  /// The angular velocity of the stance leg.
  /// @note @c thetadot is expressed in units of rad/sec.
  const T& thetadot() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kThetadot);
  }
  /// Setter that matches thetadot().
  void set_thetadot(const T& thetadot) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kThetadot, thetadot);
  }
  /// Fluent setter that matches thetadot().
  /// Returns a copy of `this` with thetadot set to a new value.
  [[nodiscard]] RimlessWheelContinuousState<T> with_thetadot(
      const T& thetadot) const {
    RimlessWheelContinuousState<T> result(*this);
    result.set_thetadot(thetadot);
    return result;
  }
  //@}

  /// Visit each field of this named vector, passing them (in order) to the
  /// given Archive.  The archive can read and/or write to the vector values.
  /// One common use of Serialize is the //common/yaml tools.
  template <typename Archive>
  void Serialize(Archive* a) {
    T& theta_ref = this->GetAtIndex(K::kTheta);
    a->Visit(drake::MakeNameValue("theta", &theta_ref));
    T& thetadot_ref = this->GetAtIndex(K::kThetadot);
    a->Visit(drake::MakeNameValue("thetadot", &thetadot_ref));
  }

  /// See RimlessWheelContinuousStateIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return RimlessWheelContinuousStateIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::boolean<T> IsValid() const {
    using std::isnan;
    drake::boolean<T> result{true};
    result = result && !isnan(theta());
    result = result && !isnan(thetadot());
    return result;
  }

 private:
  void ThrowIfEmpty() const {
    if (this->values().size() == 0) {
      throw std::out_of_range(
          "The RimlessWheelContinuousState vector has been moved-from; "
          "accessor methods may no longer be used");
    }
  }
};

}  // namespace rimless_wheel
}  // namespace examples
}  // namespace drake
