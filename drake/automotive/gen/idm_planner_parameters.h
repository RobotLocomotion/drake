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

/// Describes the row indices of a IdmPlannerParameters.
struct IdmPlannerParametersIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 6;

  // The index of each individual coordinate.
  static const int kVRef = 0;
  static const int kA = 1;
  static const int kB = 2;
  static const int kS0 = 3;
  static const int kTimeHeadway = 4;
  static const int kDelta = 5;
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class IdmPlannerParameters : public systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef IdmPlannerParametersIndices K;

  /// Default constructor.  Sets all rows to zero.
  IdmPlannerParameters() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  IdmPlannerParameters<T>* DoClone() const override {
    return new IdmPlannerParameters;
  }

  /// @name Getters and Setters
  //@{
  /// desired velocity in free traffic
  const T& v_ref() const { return this->GetAtIndex(K::kVRef); }
  void set_v_ref(const T& v_ref) { this->SetAtIndex(K::kVRef, v_ref); }
  /// max acceleration
  const T& a() const { return this->GetAtIndex(K::kA); }
  void set_a(const T& a) { this->SetAtIndex(K::kA, a); }
  /// comfortable braking deceleration
  const T& b() const { return this->GetAtIndex(K::kB); }
  void set_b(const T& b) { this->SetAtIndex(K::kB, b); }
  /// minimum desired net distance
  const T& s_0() const { return this->GetAtIndex(K::kS0); }
  void set_s_0(const T& s_0) { this->SetAtIndex(K::kS0, s_0); }
  /// desired time headway to vehicle in front
  const T& time_headway() const { return this->GetAtIndex(K::kTimeHeadway); }
  void set_time_headway(const T& time_headway) {
    this->SetAtIndex(K::kTimeHeadway, time_headway);
  }
  /// free-road exponent
  const T& delta() const { return this->GetAtIndex(K::kDelta); }
  void set_delta(const T& delta) { this->SetAtIndex(K::kDelta, delta); }
  //@}

  /// Returns whether the current values of this vector are well-formed.
  decltype(T() < T()) IsValid() const {
    using std::isnan;
    auto result = (T(0) == T(0));
    result = result && !isnan(v_ref());
    result = result && !isnan(a());
    result = result && !isnan(b());
    result = result && !isnan(s_0());
    result = result && !isnan(time_headway());
    result = result && !isnan(delta());
    return result;
  }
};

}  // namespace automotive
}  // namespace drake
