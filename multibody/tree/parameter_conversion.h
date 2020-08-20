#pragma once

#include "drake/common/default_scalars.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace multibody {
namespace internal {
namespace parameter_conversion {

// Conversions for SpatialInertia<T>
// A flattened SpatialInertia<T> looks like:
// [0]: mass
// [1]: center_of_mass_x
// [2]: center_of_mass_y
// [3]: center_of_masS_z
// [4]: unit_inertia_Gxx
// [5]: unit_inertia_Gyy
// [6]: unit_inertia_Gzz
// [7]: unit_inertia_Gxy
// [8]: unit_inertia_Gxz
// [9]: unit_inertia_Gyz


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

// Converts a BasicVector<T> to a SpatialInertia<T>
template <typename T>
SpatialInertia<T> ToSpatialInertia(
    const systems::BasicVector<T>& spatial_inertia_vector) {
  DRAKE_DEMAND(spatial_inertia_vector.size() == 10);
  return SpatialInertia<T>(
      spatial_inertia_vector[0],
      Vector3<T>(spatial_inertia_vector[1], spatial_inertia_vector[2],
                 spatial_inertia_vector[3]),
      UnitInertia<T>(spatial_inertia_vector[4], spatial_inertia_vector[5],
                     spatial_inertia_vector[6], spatial_inertia_vector[7],
                     spatial_inertia_vector[8], spatial_inertia_vector[9]),
      true);
}
}  // namespace parameter_conversion
}  // namespace internal
}  // namespace multibody
}  // namespace drake
