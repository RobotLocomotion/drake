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
namespace multibody {
namespace cart_pole {

/// Describes the row indices of a CartPoleParams.
struct CartPoleParamsIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 4;

  // The index of each individual coordinate.
  static const int kMc = 0;
  static const int kMp = 1;
  static const int kL = 2;
  static const int kGravity = 3;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `CartPoleParamsIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class CartPoleParams final : public drake::systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef CartPoleParamsIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c mc defaults to 10.0 kg.
  /// @arg @c mp defaults to 1.0 kg.
  /// @arg @c l defaults to 0.5 m.
  /// @arg @c gravity defaults to 9.81 m/s^2.
  CartPoleParams() : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_mc(10.0);
    this->set_mp(1.0);
    this->set_l(0.5);
    this->set_gravity(9.81);
  }

  // Note: It's safe to implement copy and move because this class is final.

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable
  //@{
  CartPoleParams(const CartPoleParams& other)
      : drake::systems::BasicVector<T>(other.values()) {}
  CartPoleParams(CartPoleParams&& other) noexcept
      : drake::systems::BasicVector<T>(std::move(other.values())) {}
  CartPoleParams& operator=(const CartPoleParams& other) {
    this->values() = other.values();
    return *this;
  }
  CartPoleParams& operator=(CartPoleParams&& other) noexcept {
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
    this->set_mc(symbolic::Variable("mc"));
    this->set_mp(symbolic::Variable("mp"));
    this->set_l(symbolic::Variable("l"));
    this->set_gravity(symbolic::Variable("gravity"));
  }

  [[nodiscard]] CartPoleParams<T>* DoClone() const final {
    return new CartPoleParams;
  }

  /// @name Getters and Setters
  //@{
  /// Mass of the cart.
  /// @note @c mc is expressed in units of kg.
  /// @note @c mc has a limited domain of [0.0, +Inf].
  const T& mc() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kMc);
  }
  /// Setter that matches mc().
  void set_mc(const T& mc) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kMc, mc);
  }
  /// Fluent setter that matches mc().
  /// Returns a copy of `this` with mc set to a new value.
  [[nodiscard]] CartPoleParams<T> with_mc(const T& mc) const {
    CartPoleParams<T> result(*this);
    result.set_mc(mc);
    return result;
  }
  /// Value of the point mass at the end of the pole.
  /// @note @c mp is expressed in units of kg.
  /// @note @c mp has a limited domain of [0.0, +Inf].
  const T& mp() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kMp);
  }
  /// Setter that matches mp().
  void set_mp(const T& mp) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kMp, mp);
  }
  /// Fluent setter that matches mp().
  /// Returns a copy of `this` with mp set to a new value.
  [[nodiscard]] CartPoleParams<T> with_mp(const T& mp) const {
    CartPoleParams<T> result(*this);
    result.set_mp(mp);
    return result;
  }
  /// Length of the pole.
  /// @note @c l is expressed in units of m.
  /// @note @c l has a limited domain of [0.0, +Inf].
  const T& l() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kL);
  }
  /// Setter that matches l().
  void set_l(const T& l) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kL, l);
  }
  /// Fluent setter that matches l().
  /// Returns a copy of `this` with l set to a new value.
  [[nodiscard]] CartPoleParams<T> with_l(const T& l) const {
    CartPoleParams<T> result(*this);
    result.set_l(l);
    return result;
  }
  /// Standard acceleration due to gravity near Earth's surface.
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
  [[nodiscard]] CartPoleParams<T> with_gravity(const T& gravity) const {
    CartPoleParams<T> result(*this);
    result.set_gravity(gravity);
    return result;
  }
  //@}

  /// Visit each field of this named vector, passing them (in order) to the
  /// given Archive.  The archive can read and/or write to the vector values.
  /// One common use of Serialize is the //common/yaml tools.
  template <typename Archive>
  void Serialize(Archive* a) {
    T& mc_ref = this->GetAtIndex(K::kMc);
    a->Visit(drake::MakeNameValue("mc", &mc_ref));
    T& mp_ref = this->GetAtIndex(K::kMp);
    a->Visit(drake::MakeNameValue("mp", &mp_ref));
    T& l_ref = this->GetAtIndex(K::kL);
    a->Visit(drake::MakeNameValue("l", &l_ref));
    T& gravity_ref = this->GetAtIndex(K::kGravity);
    a->Visit(drake::MakeNameValue("gravity", &gravity_ref));
  }

  /// See CartPoleParamsIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return CartPoleParamsIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::boolean<T> IsValid() const {
    using std::isnan;
    drake::boolean<T> result{true};
    result = result && !isnan(mc());
    result = result && (mc() >= T(0.0));
    result = result && !isnan(mp());
    result = result && (mp() >= T(0.0));
    result = result && !isnan(l());
    result = result && (l() >= T(0.0));
    result = result && !isnan(gravity());
    result = result && (gravity() >= T(0.0));
    return result;
  }

  void GetElementBounds(Eigen::VectorXd* lower,
                        Eigen::VectorXd* upper) const final {
    const double kInf = std::numeric_limits<double>::infinity();
    *lower = Eigen::Matrix<double, 4, 1>::Constant(-kInf);
    *upper = Eigen::Matrix<double, 4, 1>::Constant(kInf);
    (*lower)(K::kMc) = 0.0;
    (*lower)(K::kMp) = 0.0;
    (*lower)(K::kL) = 0.0;
    (*lower)(K::kGravity) = 0.0;
  }

 private:
  void ThrowIfEmpty() const {
    if (this->values().size() == 0) {
      throw std::out_of_range(
          "The CartPoleParams vector has been moved-from; "
          "accessor methods may no longer be used");
    }
  }
};

}  // namespace cart_pole
}  // namespace multibody
}  // namespace examples
}  // namespace drake
