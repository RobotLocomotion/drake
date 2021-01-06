#pragma once

#include "drake/common/default_scalars.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace multibody {
namespace internal {
namespace parameter_conversion {

// Conversions for SpatialInertia<T>
struct SpatialInertiaIndex {
  // The total number of rows (coordinates).
  inline static const int k_num_coordinates = 10;

  // The index of each individual coordinate.
  inline static const int k_mass = 0;   // [0]: mass
  inline static const int k_com_x = 1;  // [1]: center_of_mass_x
  inline static const int k_com_y = 2;  // [2]: center_of_mass_y
  inline static const int k_com_z = 3;  // [3]: center_of_masS_z
  inline static const int k_Gxx = 4;    // [4]: unit_inertia_Gxx
  inline static const int k_Gyy = 5;    // [5]: unit_inertia_Gyy
  inline static const int k_Gzz = 6;    // [6]: unit_inertia_Gzz
  inline static const int k_Gxy = 7;    // [7]: unit_inertia_Gxy
  inline static const int k_Gxz = 8;    // [8]: unit_inertia_Gxz
  inline static const int k_Gyz = 9;    // [9]: unit_inertia_Gyz
};

// Converts a SpatialInertia<T> to a BasicVector<T>
template <typename T>
systems::BasicVector<T> ToBasicVector(
    const SpatialInertia<T>& spatial_inertia) {
  const Vector3<T>& com = spatial_inertia.get_com();
  const UnitInertia<T>& unit_inertia = spatial_inertia.get_unit_inertia();
  return systems::BasicVector<T>(
      {
      // mass
      spatial_inertia.get_mass(),
      // center of mass
      com(0), com(1), com(2),
      // unit inertia
      unit_inertia(0, 0), unit_inertia(1, 1), unit_inertia(2, 2),
      unit_inertia(0, 1), unit_inertia(0, 2), unit_inertia(1, 2)});
}

// Extracts the mass from the BasicVector<T> representing a SpatialInertia<T> as
// a const reference.
template <typename T>
const T& GetMass(const systems::BasicVector<T>& spatial_inertia_vector) {
  DRAKE_DEMAND(spatial_inertia_vector.size() ==
               SpatialInertiaIndex::k_num_coordinates);
  return spatial_inertia_vector[SpatialInertiaIndex::k_mass];
}

// Extracts the center of mass from the BasicVector<T> representing a
// SpatialInertia<T>.
template <typename T>
Vector3<T> GetCenterOfMass(
    const systems::BasicVector<T>& spatial_inertia_vector) {
  DRAKE_DEMAND(spatial_inertia_vector.size() ==
               SpatialInertiaIndex::k_num_coordinates);
  return Vector3<T>(
      spatial_inertia_vector[SpatialInertiaIndex::k_com_x],
      spatial_inertia_vector[SpatialInertiaIndex::k_com_y],
      spatial_inertia_vector[SpatialInertiaIndex::k_com_z]);
}

// Converts a BasicVector<T> to a SpatialInertia<T>
template <typename T>
SpatialInertia<T> ToSpatialInertia(
    const systems::BasicVector<T>& spatial_inertia_basic_vector) {
  DRAKE_DEMAND(spatial_inertia_basic_vector.size() ==
               SpatialInertiaIndex::k_num_coordinates);
  const auto& spatial_inertia_vector = spatial_inertia_basic_vector.get_value();
  return SpatialInertia<T>(
      spatial_inertia_vector[SpatialInertiaIndex::k_mass],
      Vector3<T>(spatial_inertia_vector[SpatialInertiaIndex::k_com_x],
                 spatial_inertia_vector[SpatialInertiaIndex::k_com_y],
                 spatial_inertia_vector[SpatialInertiaIndex::k_com_z]),
      UnitInertia<T>(spatial_inertia_vector[SpatialInertiaIndex::k_Gxx],
                     spatial_inertia_vector[SpatialInertiaIndex::k_Gyy],
                     spatial_inertia_vector[SpatialInertiaIndex::k_Gzz],
                     spatial_inertia_vector[SpatialInertiaIndex::k_Gxy],
                     spatial_inertia_vector[SpatialInertiaIndex::k_Gxz],
                     spatial_inertia_vector[SpatialInertiaIndex::k_Gyz]),
      true);
}
}  // namespace parameter_conversion
}  // namespace internal
}  // namespace multibody
}  // namespace drake
