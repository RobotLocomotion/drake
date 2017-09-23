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
namespace tools {
namespace test {

/// Describes the row indices of a Sample.
struct SampleIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 3;

  // The index of each individual coordinate.
  static const int kX = 0;
  static const int kTwoWord = 1;
  static const int kAbsone = 2;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `SampleIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class Sample : public systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef SampleIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c x defaults to 42.0 m/s.
  /// @arg @c two_word defaults to 0.0 with unknown units.
  /// @arg @c absone defaults to 0.0 with unknown units.
  Sample() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_x(42.0);
    this->set_two_word(0.0);
    this->set_absone(0.0);
  }

  Sample<T>* DoClone() const override { return new Sample; }

  /// @name Getters and Setters
  //@{
  /// Some coordinate
  /// @note @c x is expressed in units of m/s.
  /// @note @c x has a limited domain of [0.0, +Inf].
  const T& x() const { return this->GetAtIndex(K::kX); }
  void set_x(const T& x) { this->SetAtIndex(K::kX, x); }
  /// A very long documentation string that will certainly flow across multiple
  /// lines of C++
  const T& two_word() const { return this->GetAtIndex(K::kTwoWord); }
  void set_two_word(const T& two_word) {
    this->SetAtIndex(K::kTwoWord, two_word);
  }
  /// A signed, normalized value
  /// @note @c absone has a limited domain of [-1.0, 1.0].
  const T& absone() const { return this->GetAtIndex(K::kAbsone); }
  void set_absone(const T& absone) { this->SetAtIndex(K::kAbsone, absone); }
  //@}

  /// See SampleIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return SampleIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  decltype(T() < T()) IsValid() const {
    using std::isnan;
    auto result = (T(0) == T(0));
    result = result && !isnan(x());
    result = result && (x() >= T(0.0));
    result = result && !isnan(two_word());
    result = result && !isnan(absone());
    result = result && (absone() >= T(-1.0));
    result = result && (absone() <= T(1.0));
    return result;
  }

  // VectorBase override.
  void CalcInequalityConstraint(VectorX<T>* value) const override {
    value->resize(3);
    (*value)[0] = x() - T(0.0);
    (*value)[1] = absone() - T(-1.0);
    (*value)[2] = T(1.0) - absone();
  }
};

}  // namespace test
}  // namespace tools
}  // namespace drake
