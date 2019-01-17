#pragma once

// This file was originally created by drake/tools/lcm_vector_gen.py;
// we have committed it to git in order to add deprecation warnings.

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_bool.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/dummy_value.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/symbolic.h"
#include "drake/systems/framework/basic_vector.h"

// TODO(jwnimmer-tri) Elevate this to drake/common.
#if __has_cpp_attribute(nodiscard)
#define DRAKE_VECTOR_GEN_NODISCARD [[nodiscard]]  // NOLINT(whitespace/braces)
#else
#define DRAKE_VECTOR_GEN_NODISCARD
#endif

namespace drake {
namespace manipulation {
namespace schunk_wsg {

/// Describes the row indices of a SchunkWsgCommand.
struct SchunkWsgCommandIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 3;

  // The index of each individual coordinate.
  static const int kUtime = 0;
  static const int kTargetPositionMm = 1;
  static const int kForce = 2;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `SchunkWsgCommandIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class SchunkWsgCommand final : public drake::systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef SchunkWsgCommandIndices K;

  // TODO(jwnimmer-tri) Remove this class after 2019-03-01.
  // There is no replacement -- these objects should never be needed anymore.
  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c utime defaults to 0.0 microsecond.
  /// @arg @c target_position_mm defaults to 0.0 mm.
  /// @arg @c force defaults to 0.0 N.
  DRAKE_DEPRECATED("This class will be removed after 2019-03-01")
  SchunkWsgCommand() : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_utime(0.0);
    this->set_target_position_mm(0.0);
    this->set_force(0.0);
  }

  // Note: It's safe to implement copy and move because this class is final.

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable
  //@{
  SchunkWsgCommand(const SchunkWsgCommand& other)
      : drake::systems::BasicVector<T>(other.values()) {}
  SchunkWsgCommand(SchunkWsgCommand&& other) noexcept
      : drake::systems::BasicVector<T>(std::move(other.values())) {}
  SchunkWsgCommand& operator=(const SchunkWsgCommand& other) {
    this->values() = other.values();
    return *this;
  }
  SchunkWsgCommand& operator=(SchunkWsgCommand&& other) noexcept {
    this->values() = std::move(other.values());
    other.values().resize(0);
    return *this;
  }
  //@}

  /// Create a symbolic::Variable for each element with the known variable
  /// name.  This is only available for T == symbolic::Expression.
  template <typename U = T>
  typename std::enable_if<std::is_same<U, symbolic::Expression>::value>::type
  SetToNamedVariables() {
    this->set_utime(symbolic::Variable("utime"));
    this->set_target_position_mm(symbolic::Variable("target_position_mm"));
    this->set_force(symbolic::Variable("force"));
  }

  SchunkWsgCommand<T>* DoClone() const final {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    return new SchunkWsgCommand;
#pragma GCC diagnostic pop
  }

  /// @name Getters and Setters
  //@{
  /// Timestamp
  /// @note @c utime is expressed in units of microsecond.
  const T& utime() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kUtime);
  }
  /// Setter that matches utime().
  void set_utime(const T& utime) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kUtime, utime);
  }
  /// Fluent setter that matches utime().
  /// Returns a copy of `this` with utime set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  SchunkWsgCommand<T> with_utime(const T& utime) const {
    SchunkWsgCommand<T> result(*this);
    result.set_utime(utime);
    return result;
  }
  /// Distance between fingers
  /// @note @c target_position_mm is expressed in units of mm.
  const T& target_position_mm() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kTargetPositionMm);
  }
  /// Setter that matches target_position_mm().
  void set_target_position_mm(const T& target_position_mm) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kTargetPositionMm, target_position_mm);
  }
  /// Fluent setter that matches target_position_mm().
  /// Returns a copy of `this` with target_position_mm set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  SchunkWsgCommand<T> with_target_position_mm(
      const T& target_position_mm) const {
    SchunkWsgCommand<T> result(*this);
    result.set_target_position_mm(target_position_mm);
    return result;
  }
  /// Max total grasping force applied by both fingers.
  /// @note @c force is expressed in units of N.
  const T& force() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kForce);
  }
  /// Setter that matches force().
  void set_force(const T& force) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kForce, force);
  }
  /// Fluent setter that matches force().
  /// Returns a copy of `this` with force set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  SchunkWsgCommand<T> with_force(const T& force) const {
    SchunkWsgCommand<T> result(*this);
    result.set_force(force);
    return result;
  }
  //@}

  /// See SchunkWsgCommandIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return SchunkWsgCommandIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::boolean<T> IsValid() const {
    using std::isnan;
    drake::boolean<T> result{true};
    result = result && !isnan(utime());
    result = result && !isnan(target_position_mm());
    result = result && !isnan(force());
    return result;
  }

 private:
  void ThrowIfEmpty() const {
    if (this->values().size() == 0) {
      throw std::out_of_range(
          "The SchunkWsgCommand vector has been moved-from; "
          "accessor methods may no longer be used");
    }
  }
};

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake

#undef DRAKE_VECTOR_GEN_NODISCARD
