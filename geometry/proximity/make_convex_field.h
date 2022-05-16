#pragma once

#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/volume_mesh_field.h"

namespace drake {
namespace geometry {
namespace internal {

/*
 @pre This pressure field generation is highly dependent on the implementation
 of the convex mesh in MakeConvexVolumeMesh(). In particular it assumes the
 last vertex in its vertex list is the sole internal vertex and the rest of
 the vertices are boundary. If the implementation in MakeConvexVolumeMesh()
 were to change, this pressure field generation would also need to change.

 @param[in] mesh_C       A pointer to a tetrahedral mesh of a convex shape.
                         It is aliased in the returned pressure field and
                         must remain alive as long as the field.
 @param[in] hydroelastic_modulus  Scale extent to pressure.
 @return                 The pressure field defined on the tetrahedral mesh.
 @pre                    `hydroelastic_modulus` is strictly positive.
                         `mesh_C` is non-null.
 @tparam T               The scalar type for representing the mesh vertex
                         positions and the pressure value.
 */
template <typename T>
VolumeMeshFieldLinear<T, T> MakeConvexPressureField(
    const VolumeMesh<T>* mesh_C, const T& hydroelastic_modulus) {
  DRAKE_DEMAND(hydroelastic_modulus > T(0));
  DRAKE_DEMAND(mesh_C != nullptr);

  std::vector<T> pressure_values(mesh_C->num_vertices(), 0.0);

  // Only the inner vertex (last vertex) has non-zero pressure value.
  pressure_values.back() = hydroelastic_modulus;

  return VolumeMeshFieldLinear<T, T>(std::move(pressure_values), mesh_C);
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
