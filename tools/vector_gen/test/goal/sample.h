#pragma once

// GENERATED GOAL DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

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
#include "drake/common/symbolic.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace tools {
namespace test {

/// Describes the row indices of a Sample.
struct SampleIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 4;

  // The index of each individual coordinate.
  static const int kX = 0;
  static const int kTwoWord = 1;
  static const int kAbsone = 2;
  static const int kUnset = 3;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `SampleIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class Sample final : public drake::systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef SampleIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c x defaults to 42.0 m/s.
  /// @arg @c two_word defaults to 0.0 with unknown units.
  /// @arg @c absone defaults to 0.0 with unknown units.
  /// @arg @c unset defaults to a dummy value with unknown units.
  Sample() : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_x(42.0);
    this->set_two_word(0.0);
    this->set_absone(0.0);
    this->set_unset(drake::dummy_value<T>::get());
  }

  // Note: It's safe to implement copy and move because this class is final.

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable
  //@{
  Sample(const Sample& other)
      : drake::systems::BasicVector<T>(other.values()) {}
  Sample(Sample&& other) noexcept
      : drake::systems::BasicVector<T>(std::move(other.values())) {}
  Sample& operator=(const Sample& other) {
    this->values() = other.values();
    return *this;
  }
  Sample& operator=(Sample&& other) noexcept {
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
    this->set_two_word(symbolic::Variable("two_word"));
    this->set_absone(symbolic::Variable("absone"));
    this->set_unset(symbolic::Variable("unset"));
  }

  [[nodiscard]] Sample<T>* DoClone() const final { return new Sample; }

  /// @name Getters and Setters
  //@{
  /// Some coordinate
  /// @note @c x is expressed in units of m/s.
  /// @note @c x has a limited domain of [0.0, +Inf].
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
  [[nodiscard]] Sample<T> with_x(const T& x) const {
    Sample<T> result(*this);
    result.set_x(x);
    return result;
  }
  /// A very long documentation string that will certainly flow across multiple
  /// lines of C++
  /// @note @c two_word has a limited domain of [-Inf, 2.0].
  const T& two_word() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kTwoWord);
  }
  /// Setter that matches two_word().
  void set_two_word(const T& two_word) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kTwoWord, two_word);
  }
  /// Fluent setter that matches two_word().
  /// Returns a copy of `this` with two_word set to a new value.
  [[nodiscard]] Sample<T> with_two_word(const T& two_word) const {
    Sample<T> result(*this);
    result.set_two_word(two_word);
    return result;
  }
  /// A signed, normalized value
  /// @note @c absone has a limited domain of [-1.0, 1.0].
  const T& absone() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kAbsone);
  }
  /// Setter that matches absone().
  void set_absone(const T& absone) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kAbsone, absone);
  }
  /// Fluent setter that matches absone().
  /// Returns a copy of `this` with absone set to a new value.
  [[nodiscard]] Sample<T> with_absone(const T& absone) const {
    Sample<T> result(*this);
    result.set_absone(absone);
    return result;
  }
  /// A value that is unset by default
  const T& unset() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kUnset);
  }
  /// Setter that matches unset().
  void set_unset(const T& unset) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kUnset, unset);
  }
  /// Fluent setter that matches unset().
  /// Returns a copy of `this` with unset set to a new value.
  [[nodiscard]] Sample<T> with_unset(const T& unset) const {
    Sample<T> result(*this);
    result.set_unset(unset);
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
    T& two_word_ref = this->GetAtIndex(K::kTwoWord);
    a->Visit(drake::MakeNameValue("two_word", &two_word_ref));
    T& absone_ref = this->GetAtIndex(K::kAbsone);
    a->Visit(drake::MakeNameValue("absone", &absone_ref));
    T& unset_ref = this->GetAtIndex(K::kUnset);
    a->Visit(drake::MakeNameValue("unset", &unset_ref));
  }

  /// See SampleIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return SampleIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::boolean<T> IsValid() const {
    using std::isnan;
    drake::boolean<T> result{true};
    result = result && !isnan(x());
    result = result && (x() >= T(0.0));
    result = result && !isnan(two_word());
    result = result && (two_word() <= T(2.0));
    result = result && !isnan(absone());
    result = result && (absone() >= T(-1.0));
    result = result && (absone() <= T(1.0));
    result = result && !isnan(unset());
    return result;
  }

  void GetElementBounds(Eigen::VectorXd* lower,
                        Eigen::VectorXd* upper) const final {
    const double kInf = std::numeric_limits<double>::infinity();
    *lower = Eigen::Matrix<double, 4, 1>::Constant(-kInf);
    *upper = Eigen::Matrix<double, 4, 1>::Constant(kInf);
    (*lower)(K::kX) = 0.0;
    (*upper)(K::kTwoWord) = 2.0;
    (*lower)(K::kAbsone) = -1.0;
    (*upper)(K::kAbsone) = 1.0;
  }

 private:
  void ThrowIfEmpty() const {
    if (this->values().size() == 0) {
      throw std::out_of_range(
          "The Sample vector has been moved-from; "
          "accessor methods may no longer be used");
    }
  }
};

}  // namespace test
}  // namespace tools
}  // namespace drake
