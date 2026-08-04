#pragma once

// This file was previously auto-generated, but now is just a normal source
// file in git. However, it still retains some historical oddities from its
// heritage. In general, we do recommend against subclassing BasicVector in
// new code.

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
#include "drake/common/symbolic/expression.h"
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
class SchunkWsgTrajectoryGeneratorStateVector final
    : public drake::systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef SchunkWsgTrajectoryGeneratorStateVectorIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c last_target_position defaults to 0.0 with unknown units.
  /// @arg @c trajectory_start_time defaults to 0.0 with unknown units.
  /// @arg @c last_position defaults to 0.0 with unknown units.
  /// @arg @c max_force defaults to 0.0 with unknown units.
  SchunkWsgTrajectoryGeneratorStateVector()
      : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    const double kInf = std::numeric_limits<double>::infinity();
    this->set_last_target_position(0.0);
    this->set_trajectory_start_time(kInf);
    this->set_last_position(0.0);
    this->set_max_force(0.0);
  }

  // Note: It's safe to implement copy and move because this class is final.

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable
  //@{
  SchunkWsgTrajectoryGeneratorStateVector(
      const SchunkWsgTrajectoryGeneratorStateVector& other)
      : drake::systems::BasicVector<T>(other.values()) {}
  SchunkWsgTrajectoryGeneratorStateVector(
      SchunkWsgTrajectoryGeneratorStateVector&& other) noexcept
      : drake::systems::BasicVector<T>(std::move(other.values())) {}
  SchunkWsgTrajectoryGeneratorStateVector& operator=(
      const SchunkWsgTrajectoryGeneratorStateVector& other) {
    this->values() = other.values();
    return *this;
  }
  SchunkWsgTrajectoryGeneratorStateVector& operator=(
      SchunkWsgTrajectoryGeneratorStateVector&& other) noexcept {
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
    this->set_last_target_position(symbolic::Variable("last_target_position"));
    this->set_trajectory_start_time(
        symbolic::Variable("trajectory_start_time"));
    this->set_last_position(symbolic::Variable("last_position"));
    this->set_max_force(symbolic::Variable("max_force"));
  }

  [[nodiscard]] SchunkWsgTrajectoryGeneratorStateVector<T>* DoClone()
      const final {
    return new SchunkWsgTrajectoryGeneratorStateVector;
  }

  /// @name Getters and Setters
  //@{
  /// last_target_position
  const T& last_target_position() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kLastTargetPosition);
  }
  /// Setter that matches last_target_position().
  void set_last_target_position(const T& last_target_position) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kLastTargetPosition, last_target_position);
  }
  /// Fluent setter that matches last_target_position().
  /// Returns a copy of `this` with last_target_position set to a new value.
  [[nodiscard]] SchunkWsgTrajectoryGeneratorStateVector<T>
  with_last_target_position(const T& last_target_position) const {
    SchunkWsgTrajectoryGeneratorStateVector<T> result(*this);
    result.set_last_target_position(last_target_position);
    return result;
  }
  /// trajectory_start_time
  const T& trajectory_start_time() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kTrajectoryStartTime);
  }
  /// Setter that matches trajectory_start_time().
  void set_trajectory_start_time(const T& trajectory_start_time) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kTrajectoryStartTime, trajectory_start_time);
  }
  /// Fluent setter that matches trajectory_start_time().
  /// Returns a copy of `this` with trajectory_start_time set to a new value.
  [[nodiscard]] SchunkWsgTrajectoryGeneratorStateVector<T>
  with_trajectory_start_time(const T& trajectory_start_time) const {
    SchunkWsgTrajectoryGeneratorStateVector<T> result(*this);
    result.set_trajectory_start_time(trajectory_start_time);
    return result;
  }
  /// last_position
  const T& last_position() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kLastPosition);
  }
  /// Setter that matches last_position().
  void set_last_position(const T& last_position) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kLastPosition, last_position);
  }
  /// Fluent setter that matches last_position().
  /// Returns a copy of `this` with last_position set to a new value.
  [[nodiscard]] SchunkWsgTrajectoryGeneratorStateVector<T> with_last_position(
      const T& last_position) const {
    SchunkWsgTrajectoryGeneratorStateVector<T> result(*this);
    result.set_last_position(last_position);
    return result;
  }
  /// max_force
  const T& max_force() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kMaxForce);
  }
  /// Setter that matches max_force().
  void set_max_force(const T& max_force) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kMaxForce, max_force);
  }
  /// Fluent setter that matches max_force().
  /// Returns a copy of `this` with max_force set to a new value.
  [[nodiscard]] SchunkWsgTrajectoryGeneratorStateVector<T> with_max_force(
      const T& max_force) const {
    SchunkWsgTrajectoryGeneratorStateVector<T> result(*this);
    result.set_max_force(max_force);
    return result;
  }
  //@}

  /// Visit each field of this named vector, passing them (in order) to the
  /// given Archive.  The archive can read and/or write to the vector values.
  /// One common use of Serialize is the //common/yaml tools.
  template <typename Archive>
  void Serialize(Archive* a) {
    T& last_target_position_ref = this->GetAtIndex(K::kLastTargetPosition);
    a->Visit(drake::MakeNameValue("last_target_position",
                                  &last_target_position_ref));
    T& trajectory_start_time_ref = this->GetAtIndex(K::kTrajectoryStartTime);
    a->Visit(drake::MakeNameValue("trajectory_start_time",
                                  &trajectory_start_time_ref));
    T& last_position_ref = this->GetAtIndex(K::kLastPosition);
    a->Visit(drake::MakeNameValue("last_position", &last_position_ref));
    T& max_force_ref = this->GetAtIndex(K::kMaxForce);
    a->Visit(drake::MakeNameValue("max_force", &max_force_ref));
  }

  /// See SchunkWsgTrajectoryGeneratorStateVectorIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return SchunkWsgTrajectoryGeneratorStateVectorIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::boolean<T> IsValid() const {
    using std::isnan;
    drake::boolean<T> result{true};
    result = result && !isnan(last_target_position());
    result = result && !isnan(trajectory_start_time());
    result = result && !isnan(last_position());
    result = result && !isnan(max_force());
    return result;
  }

 private:
  void ThrowIfEmpty() const {
    if (this->values().size() == 0) {
      throw std::out_of_range(
          "The SchunkWsgTrajectoryGeneratorStateVector vector has been "
          "moved-from; "
          "accessor methods may no longer be used");
    }
  }
};

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
