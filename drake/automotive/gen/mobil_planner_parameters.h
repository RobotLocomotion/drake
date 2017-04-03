#pragma once

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <cmath>
#include <stdexcept>
#include <string>

#include <Eigen/Core>

#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace automotive {

/// Describes the row indices of a MobilPlannerParameters.
struct MobilPlannerParametersIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 3;

  // The index of each individual coordinate.
  static const int kP = 0;
  static const int kThreshold = 1;
  static const int kMaxDeceleration = 2;
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class MobilPlannerParameters : public systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef MobilPlannerParametersIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c p defaults to 0.5 in units of dimensionless.
  /// @arg @c threshold defaults to 0.1 in units of m/s^2.
  /// @arg @c max_deceleration defaults to 4.0 in units of m/s^2.
  MobilPlannerParameters() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_p(0.5);
    this->set_threshold(0.1);
    this->set_max_deceleration(4.0);
  }

  MobilPlannerParameters<T>* DoClone() const override {
    return new MobilPlannerParameters;
  }

  /// @name Getters and Setters
  //@{
  /// politeness factor (0.0 is purely egoistic, higher values increase
  /// politeness)
  /// @note @c p is expressed in units of dimensionless.
  /// @note @c p has a limited domain of [0.0, 1.0].
  const T& p() const { return this->GetAtIndex(K::kP); }
  void set_p(const T& p) { this->SetAtIndex(K::kP, p); }
  /// acceleration threshold for changing lanes (Delta_a_th)
  /// @note @c threshold is expressed in units of m/s^2.
  /// @note @c threshold has a limited domain of [0.0, +Inf].
  const T& threshold() const { return this->GetAtIndex(K::kThreshold); }
  void set_threshold(const T& threshold) {
    this->SetAtIndex(K::kThreshold, threshold);
  }
  /// maximum safe deceleration (b_safe)
  /// @note @c max_deceleration is expressed in units of m/s^2.
  /// @note @c max_deceleration has a limited domain of [0.0, +Inf].
  const T& max_deceleration() const {
    return this->GetAtIndex(K::kMaxDeceleration);
  }
  void set_max_deceleration(const T& max_deceleration) {
    this->SetAtIndex(K::kMaxDeceleration, max_deceleration);
  }
  //@}

  /// Returns whether the current values of this vector are well-formed.
  decltype(T() < T()) IsValid() const {
    using std::isnan;
    auto result = (T(0) == T(0));
    result = result && !isnan(p());
    result = result && (p() >= T(0.0));
    result = result && (p() <= T(1.0));
    result = result && !isnan(threshold());
    result = result && (threshold() >= T(0.0));
    result = result && !isnan(max_deceleration());
    result = result && (max_deceleration() >= T(0.0));
    return result;
  }
};

}  // namespace automotive
}  // namespace drake
