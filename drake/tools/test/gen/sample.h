#pragma once

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <cmath>
#include <stdexcept>
#include <string>

#include <Eigen/Core>

#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace tools {
namespace test {

/// Describes the row indices of a Sample.
struct SampleIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 2;

  // The index of each individual coordinate.
  static const int kX = 0;
  static const int kTwoWord = 1;
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class Sample : public systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef SampleIndices K;

  /// Default constructor.  Sets all rows to their default value.
  /// @arg @c x defaults to 42.0 in units of m/s.
  /// @arg @c two_word defaults to 0.0 in units of unknown.
  Sample() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_x(42.0);
    this->set_two_word(0.0);
  }

  Sample<T>* DoClone() const override { return new Sample; }

  /// @name Getters and Setters
  //@{
  /// Some coordinate
  /// @note @c x is expressed in units of m/s.
  const T& x() const { return this->GetAtIndex(K::kX); }
  void set_x(const T& x) { this->SetAtIndex(K::kX, x); }
  /// A very long documentation string that will certainly flow across multiple
  /// lines of C++
  /// @note @c two_word is expressed in units of unknown.
  const T& two_word() const { return this->GetAtIndex(K::kTwoWord); }
  void set_two_word(const T& two_word) {
    this->SetAtIndex(K::kTwoWord, two_word);
  }
  //@}

  /// Returns whether the current values of this vector are well-formed.
  decltype(T() < T()) IsValid() const {
    using std::isnan;
    auto result = (T(0) == T(0));
    result = result && !isnan(x());
    result = result && !isnan(two_word());
    return result;
  }
};

}  // namespace test
}  // namespace tools
}  // namespace drake
