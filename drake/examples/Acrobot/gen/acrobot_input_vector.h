#pragma once

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "drake/common/never_destroyed.h"
#include "drake/common/symbolic_formula.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace examples {
namespace acrobot {

/// Describes the row indices of a AcrobotInputVector.
struct AcrobotInputVectorIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 1;

  // The index of each individual coordinate.
  static const int kTau = 0;

  /// Returns a reference to a std::vector containing the names of each value
  /// within this class, sorted by this class's index. In other words, the name
  /// of the value returned by GetAtIndex() is the string at the same index in
  /// the returned std::vector.
  static const std::vector<std::string>& GetCoordinateNames() {
    return coordinates.access();
  }

 private:
  static const never_destroyed<std::vector<std::string>> coordinates;
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class AcrobotInputVector : public systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef AcrobotInputVectorIndices K;

  /// Default constructor.  Sets all rows to zero.
  AcrobotInputVector() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  AcrobotInputVector<T>* DoClone() const override {
    return new AcrobotInputVector;
  }

  /// @name Getters and Setters
  //@{
  /// tau
  const T& tau() const { return this->GetAtIndex(K::kTau); }
  void set_tau(const T& tau) { this->SetAtIndex(K::kTau, tau); }
  //@}

  /// See AcrobotInputVectorIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return AcrobotInputVectorIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  decltype(T() < T()) IsValid() const {
    using std::isnan;
    auto result = (T(0) == T(0));
    result = result && !isnan(tau());
    return result;
  }
};

}  // namespace acrobot
}  // namespace examples
}  // namespace drake
