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
namespace examples {
namespace scene_graph {
namespace bouncing_ball {

/// Describes the row indices of a BouncingBallVector.
struct BouncingBallVectorIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 2;

  // The index of each individual coordinate.
  static const int kZ = 0;
  static const int kZdot = 1;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `BouncingBallVectorIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class BouncingBallVector final : public drake::systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef BouncingBallVectorIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c z defaults to 0.0 with unknown units.
  /// @arg @c zdot defaults to 0.0 with unknown units.
  BouncingBallVector() : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_z(0.0);
    this->set_zdot(0.0);
  }

  // Note: It's safe to implement copy and move because this class is final.

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable
  //@{
  BouncingBallVector(const BouncingBallVector& other)
      : drake::systems::BasicVector<T>(other.values()) {}
  BouncingBallVector(BouncingBallVector&& other) noexcept
      : drake::systems::BasicVector<T>(std::move(other.values())) {}
  BouncingBallVector& operator=(const BouncingBallVector& other) {
    this->values() = other.values();
    return *this;
  }
  BouncingBallVector& operator=(BouncingBallVector&& other) noexcept {
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
    this->set_z(symbolic::Variable("z"));
    this->set_zdot(symbolic::Variable("zdot"));
  }

  [[nodiscard]] BouncingBallVector<T>* DoClone() const final {
    return new BouncingBallVector;
  }

  /// @name Getters and Setters
  //@{
  /// z
  const T& z() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kZ);
  }
  /// Setter that matches z().
  void set_z(const T& z) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kZ, z);
  }
  /// Fluent setter that matches z().
  /// Returns a copy of `this` with z set to a new value.
  [[nodiscard]] BouncingBallVector<T> with_z(const T& z) const {
    BouncingBallVector<T> result(*this);
    result.set_z(z);
    return result;
  }
  /// zdot
  const T& zdot() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kZdot);
  }
  /// Setter that matches zdot().
  void set_zdot(const T& zdot) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kZdot, zdot);
  }
  /// Fluent setter that matches zdot().
  /// Returns a copy of `this` with zdot set to a new value.
  [[nodiscard]] BouncingBallVector<T> with_zdot(const T& zdot) const {
    BouncingBallVector<T> result(*this);
    result.set_zdot(zdot);
    return result;
  }
  //@}

  /// Visit each field of this named vector, passing them (in order) to the
  /// given Archive.  The archive can read and/or write to the vector values.
  /// One common use of Serialize is the //common/yaml tools.
  template <typename Archive>
  void Serialize(Archive* a) {
    T& z_ref = this->GetAtIndex(K::kZ);
    a->Visit(drake::MakeNameValue("z", &z_ref));
    T& zdot_ref = this->GetAtIndex(K::kZdot);
    a->Visit(drake::MakeNameValue("zdot", &zdot_ref));
  }

  /// See BouncingBallVectorIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return BouncingBallVectorIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::boolean<T> IsValid() const {
    using std::isnan;
    drake::boolean<T> result{true};
    result = result && !isnan(z());
    result = result && !isnan(zdot());
    return result;
  }

 private:
  void ThrowIfEmpty() const {
    if (this->values().size() == 0) {
      throw std::out_of_range(
          "The BouncingBallVector vector has been moved-from; "
          "accessor methods may no longer be used");
    }
  }
};

}  // namespace bouncing_ball
}  // namespace scene_graph
}  // namespace examples
}  // namespace drake
