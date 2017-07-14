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
namespace manipulation {
namespace schunk_wsg {

/// Describes the row indices of a SchunkWsgTrajectoryGeneratorStateVector.
struct SchunkWsgTrajectoryGeneratorStateVectorIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 4;

  // The index of each individual coordinate.
  static const int kLastTargetPosition = 0;
  static const int kTrajectoryStartTime = 1;
  static const int kLastPosition = 2;
  static const int kMaxForce = 3;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words,
  /// `SchunkWsgTrajectoryGeneratorStateVectorIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class SchunkWsgTrajectoryGeneratorStateVector : public systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef SchunkWsgTrajectoryGeneratorStateVectorIndices K;

  /// Default constructor.  Sets all rows to zero.
  SchunkWsgTrajectoryGeneratorStateVector()
      : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  SchunkWsgTrajectoryGeneratorStateVector<T>* DoClone() const override {
    return new SchunkWsgTrajectoryGeneratorStateVector;
  }

  /// @name Getters and Setters
  //@{
  /// last_target_position
  const T& last_target_position() const {
    return this->GetAtIndex(K::kLastTargetPosition);
  }
  void set_last_target_position(const T& last_target_position) {
    this->SetAtIndex(K::kLastTargetPosition, last_target_position);
  }
  /// trajectory_start_time
  const T& trajectory_start_time() const {
    return this->GetAtIndex(K::kTrajectoryStartTime);
  }
  void set_trajectory_start_time(const T& trajectory_start_time) {
    this->SetAtIndex(K::kTrajectoryStartTime, trajectory_start_time);
  }
  /// last_position
  const T& last_position() const { return this->GetAtIndex(K::kLastPosition); }
  void set_last_position(const T& last_position) {
    this->SetAtIndex(K::kLastPosition, last_position);
  }
  /// max_force
  const T& max_force() const { return this->GetAtIndex(K::kMaxForce); }
  void set_max_force(const T& max_force) {
    this->SetAtIndex(K::kMaxForce, max_force);
  }
  //@}

  /// See SchunkWsgTrajectoryGeneratorStateVectorIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return SchunkWsgTrajectoryGeneratorStateVectorIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  decltype(T() < T()) IsValid() const {
    using std::isnan;
    auto result = (T(0) == T(0));
    result = result && !isnan(last_target_position());
    result = result && !isnan(trajectory_start_time());
    result = result && !isnan(last_position());
    result = result && !isnan(max_force());
    return result;
  }
};

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
