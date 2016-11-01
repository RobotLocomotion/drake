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

/// Describes the row indices of a EulerFloatingJointState.
struct DRAKE_EXPORT EulerFloatingJointStateIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 6;

  // The index of each individual coordinate.
  static const int kX = 0;
  static const int kY = 1;
  static const int kZ = 2;
  static const int kRoll = 3;
  static const int kPitch = 4;
  static const int kYaw = 5;
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class EulerFloatingJointState : public systems::BasicVector<T> {
 public:
  // An abbreviation for our row index constants.
  typedef EulerFloatingJointStateIndices K;

  /// Default constructor.  Sets all rows to zero.
  EulerFloatingJointState() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  /// @name Getters and Setters
  //@{
  // x
  const T x() const { return this->GetAtIndex(K::kX); }
  void set_x(const T& x) { this->SetAtIndex(K::kX, x); }
  // y
  const T y() const { return this->GetAtIndex(K::kY); }
  void set_y(const T& y) { this->SetAtIndex(K::kY, y); }
  // z
  const T z() const { return this->GetAtIndex(K::kZ); }
  void set_z(const T& z) { this->SetAtIndex(K::kZ, z); }
  // roll
  const T roll() const { return this->GetAtIndex(K::kRoll); }
  void set_roll(const T& roll) { this->SetAtIndex(K::kRoll, roll); }
  // pitch
  const T pitch() const { return this->GetAtIndex(K::kPitch); }
  void set_pitch(const T& pitch) { this->SetAtIndex(K::kPitch, pitch); }
  // yaw
  const T yaw() const { return this->GetAtIndex(K::kYaw); }
  void set_yaw(const T& yaw) { this->SetAtIndex(K::kYaw, yaw); }
  //@}
};

}  // namespace automotive
}  // namespace drake
