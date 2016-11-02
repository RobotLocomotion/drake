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

/// Describes the row indices of a IdmWithTrajectoryAgentState.
struct DRAKE_EXPORT IdmWithTrajectoryAgentStateIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 5;

  // The index of each individual coordinate.
  static const int kXE = 0;
  static const int kVE = 1;
  static const int kXA = 2;
  static const int kVA = 3;
  static const int kAA = 4;
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class IdmWithTrajectoryAgentState : public systems::BasicVector<T> {
 public:
  // An abbreviation for our row index constants.
  typedef IdmWithTrajectoryAgentStateIndices K;

  /// Default constructor.  Sets all rows to zero.
  IdmWithTrajectoryAgentState() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  /// @name Getters and Setters
  //@{
  // x_e
  const T x_e() const { return this->GetAtIndex(K::kXE); }
  void set_x_e(const T& x_e) { this->SetAtIndex(K::kXE, x_e); }
  // v_e
  const T v_e() const { return this->GetAtIndex(K::kVE); }
  void set_v_e(const T& v_e) { this->SetAtIndex(K::kVE, v_e); }
  // x_a
  const T x_a() const { return this->GetAtIndex(K::kXA); }
  void set_x_a(const T& x_a) { this->SetAtIndex(K::kXA, x_a); }
  // v_a
  const T v_a() const { return this->GetAtIndex(K::kVA); }
  void set_v_a(const T& v_a) { this->SetAtIndex(K::kVA, v_a); }
  // a_a
  const T a_a() const { return this->GetAtIndex(K::kAA); }
  void set_a_a(const T& a_a) { this->SetAtIndex(K::kAA, a_a); }
  //@}
};

}  // namespace automotive
}  // namespace drake
