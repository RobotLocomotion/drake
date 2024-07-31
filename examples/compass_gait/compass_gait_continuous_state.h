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
namespace compass_gait {

/// Describes the row indices of a CompassGaitContinuousState.
struct CompassGaitContinuousStateIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 4;

  // The index of each individual coordinate.
  static const int kStance = 0;
  static const int kSwing = 1;
  static const int kStancedot = 2;
  static const int kSwingdot = 3;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words,
  /// `CompassGaitContinuousStateIndices::GetCoordinateNames()[i]` is the name
  /// for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class CompassGaitContinuousState final : public drake::systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef CompassGaitContinuousStateIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c stance defaults to 0.0 radians.
  /// @arg @c swing defaults to 0.0 radians.
  /// @arg @c stancedot defaults to 0.0 rad/sec.
  /// @arg @c swingdot defaults to 0.0 rad/sec.
  CompassGaitContinuousState()
      : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_stance(0.0);
    this->set_swing(0.0);
    this->set_stancedot(0.0);
    this->set_swingdot(0.0);
  }

  // Note: It's safe to implement copy and move because this class is final.

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable
  //@{
  CompassGaitContinuousState(const CompassGaitContinuousState& other)
      : drake::systems::BasicVector<T>(other.values()) {}
  CompassGaitContinuousState(CompassGaitContinuousState&& other) noexcept
      : drake::systems::BasicVector<T>(std::move(other.values())) {}
  CompassGaitContinuousState& operator=(
      const CompassGaitContinuousState& other) {
    this->values() = other.values();
    return *this;
  }
  CompassGaitContinuousState& operator=(
      CompassGaitContinuousState&& other) noexcept {
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
    this->set_stance(symbolic::Variable("stance"));
    this->set_swing(symbolic::Variable("swing"));
    this->set_stancedot(symbolic::Variable("stancedot"));
    this->set_swingdot(symbolic::Variable("swingdot"));
  }

  [[nodiscard]] CompassGaitContinuousState<T>* DoClone() const final {
    return new CompassGaitContinuousState;
  }

  /// @name Getters and Setters
  //@{
  /// The orientation of the stance leg, measured clockwise from the vertical
  /// axis.
  /// @note @c stance is expressed in units of radians.
  const T& stance() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kStance);
  }
  /// Setter that matches stance().
  void set_stance(const T& stance) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kStance, stance);
  }
  /// Fluent setter that matches stance().
  /// Returns a copy of `this` with stance set to a new value.
  [[nodiscard]] CompassGaitContinuousState<T> with_stance(
      const T& stance) const {
    CompassGaitContinuousState<T> result(*this);
    result.set_stance(stance);
    return result;
  }
  /// The orientation of the swing leg, measured clockwise from the vertical
  /// axis.
  /// @note @c swing is expressed in units of radians.
  const T& swing() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kSwing);
  }
  /// Setter that matches swing().
  void set_swing(const T& swing) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kSwing, swing);
  }
  /// Fluent setter that matches swing().
  /// Returns a copy of `this` with swing set to a new value.
  [[nodiscard]] CompassGaitContinuousState<T> with_swing(const T& swing) const {
    CompassGaitContinuousState<T> result(*this);
    result.set_swing(swing);
    return result;
  }
  /// The angular velocity of the stance leg.
  /// @note @c stancedot is expressed in units of rad/sec.
  const T& stancedot() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kStancedot);
  }
  /// Setter that matches stancedot().
  void set_stancedot(const T& stancedot) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kStancedot, stancedot);
  }
  /// Fluent setter that matches stancedot().
  /// Returns a copy of `this` with stancedot set to a new value.
  [[nodiscard]] CompassGaitContinuousState<T> with_stancedot(
      const T& stancedot) const {
    CompassGaitContinuousState<T> result(*this);
    result.set_stancedot(stancedot);
    return result;
  }
  /// The angular velocity of the swing leg.
  /// @note @c swingdot is expressed in units of rad/sec.
  const T& swingdot() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kSwingdot);
  }
  /// Setter that matches swingdot().
  void set_swingdot(const T& swingdot) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kSwingdot, swingdot);
  }
  /// Fluent setter that matches swingdot().
  /// Returns a copy of `this` with swingdot set to a new value.
  [[nodiscard]] CompassGaitContinuousState<T> with_swingdot(
      const T& swingdot) const {
    CompassGaitContinuousState<T> result(*this);
    result.set_swingdot(swingdot);
    return result;
  }
  //@}

  /// Visit each field of this named vector, passing them (in order) to the
  /// given Archive.  The archive can read and/or write to the vector values.
  /// One common use of Serialize is the //common/yaml tools.
  template <typename Archive>
  void Serialize(Archive* a) {
    T& stance_ref = this->GetAtIndex(K::kStance);
    a->Visit(drake::MakeNameValue("stance", &stance_ref));
    T& swing_ref = this->GetAtIndex(K::kSwing);
    a->Visit(drake::MakeNameValue("swing", &swing_ref));
    T& stancedot_ref = this->GetAtIndex(K::kStancedot);
    a->Visit(drake::MakeNameValue("stancedot", &stancedot_ref));
    T& swingdot_ref = this->GetAtIndex(K::kSwingdot);
    a->Visit(drake::MakeNameValue("swingdot", &swingdot_ref));
  }

  /// See CompassGaitContinuousStateIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return CompassGaitContinuousStateIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::boolean<T> IsValid() const {
    using std::isnan;
    drake::boolean<T> result{true};
    result = result && !isnan(stance());
    result = result && !isnan(swing());
    result = result && !isnan(stancedot());
    result = result && !isnan(swingdot());
    return result;
  }

 private:
  void ThrowIfEmpty() const {
    if (this->values().size() == 0) {
      throw std::out_of_range(
          "The CompassGaitContinuousState vector has been moved-from; "
          "accessor methods may no longer be used");
    }
  }
};

}  // namespace compass_gait
}  // namespace examples
}  // namespace drake
