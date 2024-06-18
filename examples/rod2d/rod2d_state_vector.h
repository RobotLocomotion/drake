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
namespace rod2d {

/// Describes the row indices of a Rod2dStateVector.
struct Rod2dStateVectorIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 6;

  // The index of each individual coordinate.
  static const int kX = 0;
  static const int kY = 1;
  static const int kTheta = 2;
  static const int kXdot = 3;
  static const int kYdot = 4;
  static const int kThetadot = 5;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `Rod2dStateVectorIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class Rod2dStateVector final : public drake::systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef Rod2dStateVectorIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c x defaults to 0.0 m.
  /// @arg @c y defaults to 0.0 m.
  /// @arg @c theta defaults to 0.0 rad.
  /// @arg @c xdot defaults to 0.0 m/s.
  /// @arg @c ydot defaults to 0.0 m/s.
  /// @arg @c thetadot defaults to 0.0 rad/s.
  Rod2dStateVector() : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_x(0.0);
    this->set_y(0.0);
    this->set_theta(0.0);
    this->set_xdot(0.0);
    this->set_ydot(0.0);
    this->set_thetadot(0.0);
  }

  // Note: It's safe to implement copy and move because this class is final.

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable
  //@{
  Rod2dStateVector(const Rod2dStateVector& other)
      : drake::systems::BasicVector<T>(other.values()) {}
  Rod2dStateVector(Rod2dStateVector&& other) noexcept
      : drake::systems::BasicVector<T>(std::move(other.values())) {}
  Rod2dStateVector& operator=(const Rod2dStateVector& other) {
    this->values() = other.values();
    return *this;
  }
  Rod2dStateVector& operator=(Rod2dStateVector&& other) noexcept {
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
    this->set_x(symbolic::Variable("x"));
    this->set_y(symbolic::Variable("y"));
    this->set_theta(symbolic::Variable("theta"));
    this->set_xdot(symbolic::Variable("xdot"));
    this->set_ydot(symbolic::Variable("ydot"));
    this->set_thetadot(symbolic::Variable("thetadot"));
  }

  [[nodiscard]] Rod2dStateVector<T>* DoClone() const final {
    return new Rod2dStateVector;
  }

  /// @name Getters and Setters
  //@{
  /// Horizontal location of the center-of-mass.
  /// @note @c x is expressed in units of m.
  const T& x() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kX);
  }
  /// Setter that matches x().
  void set_x(const T& x) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kX, x);
  }
  /// Fluent setter that matches x().
  /// Returns a copy of `this` with x set to a new value.
  [[nodiscard]] Rod2dStateVector<T> with_x(const T& x) const {
    Rod2dStateVector<T> result(*this);
    result.set_x(x);
    return result;
  }
  /// Vertical location of the center-of-mass.
  /// @note @c y is expressed in units of m.
  const T& y() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kY);
  }
  /// Setter that matches y().
  void set_y(const T& y) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kY, y);
  }
  /// Fluent setter that matches y().
  /// Returns a copy of `this` with y set to a new value.
  [[nodiscard]] Rod2dStateVector<T> with_y(const T& y) const {
    Rod2dStateVector<T> result(*this);
    result.set_y(y);
    return result;
  }
  /// Angle of the rod (measured counter-clockwise).
  /// @note @c theta is expressed in units of rad.
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
  [[nodiscard]] Rod2dStateVector<T> with_theta(const T& theta) const {
    Rod2dStateVector<T> result(*this);
    result.set_theta(theta);
    return result;
  }
  /// Velocity of the horizontal location of the center-of-mass.
  /// @note @c xdot is expressed in units of m/s.
  const T& xdot() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kXdot);
  }
  /// Setter that matches xdot().
  void set_xdot(const T& xdot) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kXdot, xdot);
  }
  /// Fluent setter that matches xdot().
  /// Returns a copy of `this` with xdot set to a new value.
  [[nodiscard]] Rod2dStateVector<T> with_xdot(const T& xdot) const {
    Rod2dStateVector<T> result(*this);
    result.set_xdot(xdot);
    return result;
  }
  /// Velocity of the vertical location of the center-of-mass.
  /// @note @c ydot is expressed in units of m/s.
  const T& ydot() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kYdot);
  }
  /// Setter that matches ydot().
  void set_ydot(const T& ydot) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kYdot, ydot);
  }
  /// Fluent setter that matches ydot().
  /// Returns a copy of `this` with ydot set to a new value.
  [[nodiscard]] Rod2dStateVector<T> with_ydot(const T& ydot) const {
    Rod2dStateVector<T> result(*this);
    result.set_ydot(ydot);
    return result;
  }
  /// Angular velocity of the rod.
  /// @note @c thetadot is expressed in units of rad/s.
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
  [[nodiscard]] Rod2dStateVector<T> with_thetadot(const T& thetadot) const {
    Rod2dStateVector<T> result(*this);
    result.set_thetadot(thetadot);
    return result;
  }
  //@}

  /// Visit each field of this named vector, passing them (in order) to the
  /// given Archive.  The archive can read and/or write to the vector values.
  /// One common use of Serialize is the //common/yaml tools.
  template <typename Archive>
  void Serialize(Archive* a) {
    T& x_ref = this->GetAtIndex(K::kX);
    a->Visit(drake::MakeNameValue("x", &x_ref));
    T& y_ref = this->GetAtIndex(K::kY);
    a->Visit(drake::MakeNameValue("y", &y_ref));
    T& theta_ref = this->GetAtIndex(K::kTheta);
    a->Visit(drake::MakeNameValue("theta", &theta_ref));
    T& xdot_ref = this->GetAtIndex(K::kXdot);
    a->Visit(drake::MakeNameValue("xdot", &xdot_ref));
    T& ydot_ref = this->GetAtIndex(K::kYdot);
    a->Visit(drake::MakeNameValue("ydot", &ydot_ref));
    T& thetadot_ref = this->GetAtIndex(K::kThetadot);
    a->Visit(drake::MakeNameValue("thetadot", &thetadot_ref));
  }

  /// See Rod2dStateVectorIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return Rod2dStateVectorIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::boolean<T> IsValid() const {
    using std::isnan;
    drake::boolean<T> result{true};
    result = result && !isnan(x());
    result = result && !isnan(y());
    result = result && !isnan(theta());
    result = result && !isnan(xdot());
    result = result && !isnan(ydot());
    result = result && !isnan(thetadot());
    return result;
  }

 private:
  void ThrowIfEmpty() const {
    if (this->values().size() == 0) {
      throw std::out_of_range(
          "The Rod2dStateVector vector has been moved-from; "
          "accessor methods may no longer be used");
    }
  }
};

}  // namespace rod2d
}  // namespace examples
}  // namespace drake
