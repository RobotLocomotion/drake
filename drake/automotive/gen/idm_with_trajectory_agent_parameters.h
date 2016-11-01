#pragma once

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <stdexcept>
#include <string>

#include <Eigen/Core>

#include "drake/common/drake_export.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace automotive {

/// Describes the row indices of a IdmWithTrajectoryAgentParameters.
struct DRAKE_EXPORT IdmWithTrajectoryAgentParametersIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 7;

  // The index of each individual coordinate.
  static const int kA = 0;
  static const int kB = 1;
  static const int kV0 = 2;
  static const int kS0 = 3;
  static const int kTimeHeadway = 4;
  static const int kDelta = 5;
  static const int kLA = 6;
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class IdmWithTrajectoryAgentParameters : public systems::BasicVector<T> {
 public:
  // An abbreviation for our row index constants.
  typedef IdmWithTrajectoryAgentParametersIndices K;

  /// Default constructor.  Sets all rows to zero.
  IdmWithTrajectoryAgentParameters()
      : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  /// @name Getters and Setters
  //@{
  // max acceleration
  const T a() const { return this->GetAtIndex(K::kA); }
  void set_a(const T& a) { this->SetAtIndex(K::kA, a); }
  // comfortable braking deceleration
  const T b() const { return this->GetAtIndex(K::kB); }
  void set_b(const T& b) { this->SetAtIndex(K::kB, b); }
  // desired velocity in free traffic
  const T v_0() const { return this->GetAtIndex(K::kV0); }
  void set_v_0(const T& v_0) { this->SetAtIndex(K::kV0, v_0); }
  // minimum desired net distance
  const T s_0() const { return this->GetAtIndex(K::kS0); }
  void set_s_0(const T& s_0) { this->SetAtIndex(K::kS0, s_0); }
  // desired time headway to vehicle in front
  const T time_headway() const { return this->GetAtIndex(K::kTimeHeadway); }
  void set_time_headway(const T& time_headway) {
    this->SetAtIndex(K::kTimeHeadway, time_headway);
  }
  // free-road exponent
  const T delta() const { return this->GetAtIndex(K::kDelta); }
  void set_delta(const T& delta) { this->SetAtIndex(K::kDelta, delta); }
  // length of leading car
  const T l_a() const { return this->GetAtIndex(K::kLA); }
  void set_l_a(const T& l_a) { this->SetAtIndex(K::kLA, l_a); }
  //@}
};

}  // namespace automotive
}  // namespace drake
