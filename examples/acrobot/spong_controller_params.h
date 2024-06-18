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
namespace acrobot {

/// Describes the row indices of a SpongControllerParams.
struct SpongControllerParamsIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 4;

  // The index of each individual coordinate.
  static const int kKE = 0;
  static const int kKP = 1;
  static const int kKD = 2;
  static const int kBalancingThreshold = 3;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `SpongControllerParamsIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class SpongControllerParams final : public drake::systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef SpongControllerParamsIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c k_e defaults to 5.0 s.
  /// @arg @c k_p defaults to 50.0 s^-2.
  /// @arg @c k_d defaults to 5.0 s^-1.
  /// @arg @c balancing_threshold defaults to 1e3 None.
  SpongControllerParams() : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_k_e(5.0);
    this->set_k_p(50.0);
    this->set_k_d(5.0);
    this->set_balancing_threshold(1e3);
  }

  // Note: It's safe to implement copy and move because this class is final.

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable
  //@{
  SpongControllerParams(const SpongControllerParams& other)
      : drake::systems::BasicVector<T>(other.values()) {}
  SpongControllerParams(SpongControllerParams&& other) noexcept
      : drake::systems::BasicVector<T>(std::move(other.values())) {}
  SpongControllerParams& operator=(const SpongControllerParams& other) {
    this->values() = other.values();
    return *this;
  }
  SpongControllerParams& operator=(SpongControllerParams&& other) noexcept {
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
    this->set_k_e(symbolic::Variable("k_e"));
    this->set_k_p(symbolic::Variable("k_p"));
    this->set_k_d(symbolic::Variable("k_d"));
    this->set_balancing_threshold(symbolic::Variable("balancing_threshold"));
  }

  [[nodiscard]] SpongControllerParams<T>* DoClone() const final {
    return new SpongControllerParams;
  }

  /// @name Getters and Setters
  //@{
  /// Energy shaping gain.
  /// @note @c k_e is expressed in units of s.
  /// @note @c k_e has a limited domain of [0.0, +Inf].
  const T& k_e() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kKE);
  }
  /// Setter that matches k_e().
  void set_k_e(const T& k_e) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kKE, k_e);
  }
  /// Fluent setter that matches k_e().
  /// Returns a copy of `this` with k_e set to a new value.
  [[nodiscard]] SpongControllerParams<T> with_k_e(const T& k_e) const {
    SpongControllerParams<T> result(*this);
    result.set_k_e(k_e);
    return result;
  }
  /// Partial feedback linearization proportional gain.
  /// @note @c k_p is expressed in units of s^-2.
  /// @note @c k_p has a limited domain of [0.0, +Inf].
  const T& k_p() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kKP);
  }
  /// Setter that matches k_p().
  void set_k_p(const T& k_p) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kKP, k_p);
  }
  /// Fluent setter that matches k_p().
  /// Returns a copy of `this` with k_p set to a new value.
  [[nodiscard]] SpongControllerParams<T> with_k_p(const T& k_p) const {
    SpongControllerParams<T> result(*this);
    result.set_k_p(k_p);
    return result;
  }
  /// Partial feedback linearization derivative gain.
  /// @note @c k_d is expressed in units of s^-1.
  /// @note @c k_d has a limited domain of [0.0, +Inf].
  const T& k_d() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kKD);
  }
  /// Setter that matches k_d().
  void set_k_d(const T& k_d) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kKD, k_d);
  }
  /// Fluent setter that matches k_d().
  /// Returns a copy of `this` with k_d set to a new value.
  [[nodiscard]] SpongControllerParams<T> with_k_d(const T& k_d) const {
    SpongControllerParams<T> result(*this);
    result.set_k_d(k_d);
    return result;
  }
  /// Cost value at which to switch from swing up to balancing.
  /// @note @c balancing_threshold is expressed in units of None.
  /// @note @c balancing_threshold has a limited domain of [0.0, +Inf].
  const T& balancing_threshold() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kBalancingThreshold);
  }
  /// Setter that matches balancing_threshold().
  void set_balancing_threshold(const T& balancing_threshold) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kBalancingThreshold, balancing_threshold);
  }
  /// Fluent setter that matches balancing_threshold().
  /// Returns a copy of `this` with balancing_threshold set to a new value.
  [[nodiscard]] SpongControllerParams<T> with_balancing_threshold(
      const T& balancing_threshold) const {
    SpongControllerParams<T> result(*this);
    result.set_balancing_threshold(balancing_threshold);
    return result;
  }
  //@}

  /// Visit each field of this named vector, passing them (in order) to the
  /// given Archive.  The archive can read and/or write to the vector values.
  /// One common use of Serialize is the //common/yaml tools.
  template <typename Archive>
  void Serialize(Archive* a) {
    T& k_e_ref = this->GetAtIndex(K::kKE);
    a->Visit(drake::MakeNameValue("k_e", &k_e_ref));
    T& k_p_ref = this->GetAtIndex(K::kKP);
    a->Visit(drake::MakeNameValue("k_p", &k_p_ref));
    T& k_d_ref = this->GetAtIndex(K::kKD);
    a->Visit(drake::MakeNameValue("k_d", &k_d_ref));
    T& balancing_threshold_ref = this->GetAtIndex(K::kBalancingThreshold);
    a->Visit(
        drake::MakeNameValue("balancing_threshold", &balancing_threshold_ref));
  }

  /// See SpongControllerParamsIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return SpongControllerParamsIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::boolean<T> IsValid() const {
    using std::isnan;
    drake::boolean<T> result{true};
    result = result && !isnan(k_e());
    result = result && (k_e() >= T(0.0));
    result = result && !isnan(k_p());
    result = result && (k_p() >= T(0.0));
    result = result && !isnan(k_d());
    result = result && (k_d() >= T(0.0));
    result = result && !isnan(balancing_threshold());
    result = result && (balancing_threshold() >= T(0.0));
    return result;
  }

  void GetElementBounds(Eigen::VectorXd* lower,
                        Eigen::VectorXd* upper) const final {
    const double kInf = std::numeric_limits<double>::infinity();
    *lower = Eigen::Matrix<double, 4, 1>::Constant(-kInf);
    *upper = Eigen::Matrix<double, 4, 1>::Constant(kInf);
    (*lower)(K::kKE) = 0.0;
    (*lower)(K::kKP) = 0.0;
    (*lower)(K::kKD) = 0.0;
    (*lower)(K::kBalancingThreshold) = 0.0;
  }

 private:
  void ThrowIfEmpty() const {
    if (this->values().size() == 0) {
      throw std::out_of_range(
          "The SpongControllerParams vector has been moved-from; "
          "accessor methods may no longer be used");
    }
  }
};

}  // namespace acrobot
}  // namespace examples
}  // namespace drake
