#pragma once

#include <algorithm>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/volume_mesh_field.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

/* @note This pressure field generation is highly dependent on the
   implementation of the capsule mesh in MakeCapsuleVolumeMesh(). In
   particular it depends on the first two vertices in its vertex list
   being the sole internal vertices (lying on the medial axis) and the
   rest of the vertices being boundary. If the implementation in
   MakeCapsuleVolumeMesh() were to change, this pressure field generation
   would also need to change. */
template <typename T>
VolumeMeshFieldLinear<T, T> MakeCapsulePressureField(
    const Capsule&, const VolumeMesh<T>* mesh_C, const T elastic_modulus) {
  DRAKE_DEMAND(elastic_modulus > T(0));
  std::vector<T> pressure_values;
  pressure_values.resize(mesh_C->num_vertices());

  std::fill(pressure_values.begin(), pressure_values.end(), T(0.0));

  // Only the inner vertices lying on the medial axis (vertex 0 and 1) have
  // non-zero pressure values.
  pressure_values[0] = pressure_values[1] = elastic_modulus;

  return VolumeMeshFieldLinear<T, T>("pressure", std::move(pressure_values),
                                     mesh_C);
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
