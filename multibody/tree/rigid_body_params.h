#pragma once

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "drake/common/never_destroyed.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/multibody/tree/unit_inertia.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace multibody {
namespace internal {

/// Describes the row indices of a RigidBodyParams.
struct RigidBodyParamsIndex {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 10;

  // The index of each individual coordinate.
  static const int k_mass = 0;   // mass
  static const int k_com_x = 1;  // center of mass x
  static const int k_com_y = 2;  // center of mass y
  static const int k_com_z = 3;  // center of mass z
  static const int k_ixx = 4;    // moment of inertia Ixx
  static const int k_iyy = 5;    // moment of inertia Iyy
  static const int k_izz = 6;    // moment of inertia Izz
  static const int k_ixy = 7;    // product of inertia Ixy
  static const int k_ixz = 8;    // product of inertia Ixz
  static const int k_iyz = 9;    // product of inertia Iyz

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `RigidBodyParams::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class RigidBodyParams final : public drake::systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef RigidBodyParamsIndex K;

  /// Default constructor
  RigidBodyParams() : drake::systems::BasicVector<T>(K::kNumCoordinates) {}

  /// Constructor off of SpatialInertia<T>.
  explicit RigidBodyParams(const SpatialInertia<T>& spatial_inertia)
      : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    set_spatial_inertia(spatial_inertia);
  }

  // Note: It's safe to implement copy and move because this class is final.

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable
  //@{
  RigidBodyParams(const RigidBodyParams& other)
      : drake::systems::BasicVector<T>(other.values()) {}
  RigidBodyParams(RigidBodyParams&& other) noexcept
      : drake::systems::BasicVector<T>(std::move(other.values())) {}
  RigidBodyParams& operator=(const RigidBodyParams& other) {
    this->values() = other.values();
    return *this;
  }
  RigidBodyParams& operator=(RigidBodyParams&& other) noexcept {
    this->values() = std::move(other.values());
    other.values().resize(0);
    return *this;
  }
  //@}

  [[nodiscard]] RigidBodyParams<T>* DoClone() const final {
    return new RigidBodyParams;
  }

  /// @name Getters and Setters
  //@{
  /// Mass of the rigid body.
  const T& mass() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::k_mass);
  }
  /// Setter that matches mass().
  void set_mass(const T& mass) {
    ThrowIfEmpty();
    this->SetAtIndex(K::k_mass, mass);
  }
  /// Center of mass of the rigid body.
  const Vector3<T> com() const {
    ThrowIfEmpty();
    return Vector3<T>(this->GetAtIndex(K::k_com_x),
                      this->GetAtIndex(K::k_com_y),
                      this->GetAtIndex(K::k_com_z));
  }
  /// Setter that matches com().
  void set_com(const Vector3<T>& com) {
    ThrowIfEmpty();
    this->SetAtIndex(K::k_com_x, com(0));
    this->SetAtIndex(K::k_com_y, com(1));
    this->SetAtIndex(K::k_com_z, com(2));
  }
  /// Unit inertia of the body.
  UnitInertia<T> unit_inertia() const {
    return UnitInertia<T>(
        this->GetAtIndex(K::k_ixx), this->GetAtIndex(K::k_iyy),
        this->GetAtIndex(K::k_izz), this->GetAtIndex(K::k_ixy),
        this->GetAtIndex(K::k_ixz), this->GetAtIndex(K::k_iyz));
  }
  /// Setter that matches unit_inertia().
  void set_unit_inertia(const UnitInertia<T>& unit_inertia) {
    ThrowIfEmpty();
    this->SetAtIndex(K::k_ixx, unit_inertia(0, 0));
    this->SetAtIndex(K::k_iyy, unit_inertia(1, 1));
    this->SetAtIndex(K::k_izz, unit_inertia(2, 2));
    this->SetAtIndex(K::k_ixy, unit_inertia(1, 0));
    this->SetAtIndex(K::k_ixz, unit_inertia(2, 0));
    this->SetAtIndex(K::k_iyz, unit_inertia(2, 1));
  }
  /// Spatial inertia of the body
  SpatialInertia<T> CopySpatialInertia() const {
    ThrowIfEmpty();
    return SpatialInertia<T>(mass(), com(), unit_inertia());
  }
  /// Setter that matches spatial_inertia().
  void set_spatial_inertia(const SpatialInertia<T>& spatial_inertia) {
    ThrowIfEmpty();
    set_mass(spatial_inertia.get_mass());
    set_com(spatial_inertia.get_com());
    set_unit_inertia(spatial_inertia.get_unit_inertia());
  }
  //@}

  /// See RigidBodyParamsIndex::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return RigidBodyParamsIndex::GetCoordinateNames();
  }

 private:
  void ThrowIfEmpty() const {
    if (this->values().size() == 0) {
      throw std::out_of_range(
          "The RigidBodyParams vector has been moved-from; "
          "accessor methods may no longer be used");
    }
  }
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
