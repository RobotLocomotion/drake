#pragma once

#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/volume_mesh_field.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

/*
 @pre This pressure field generation is highly dependent on the
 implementation of the capsule mesh in MakeCapsuleVolumeMesh(). In
 particular it depends on the first two vertices in its vertex list
 being the sole internal vertices (lying on the medial axis) and the
 rest of the vertices being boundary. If the implementation in
 MakeCapsuleVolumeMesh() were to change, this pressure field generation
 would also need to change.

 The following picture files show examples of pressure fields generated from
 MakeCapsulePressureField(). The files are distributed with the source code.

 | geometry/proximity/images/capsule_mesh_medial_axis_slice.png |
 | x = 0 slice of the capsule mesh, showing the interpolated pressure field. |

 | geometry/proximity/images/capsule_mesh_medial_axis_isosurfaces.png |
 | Isosurfaces of the capsule pressure field. |

 @param[in] capsule      The capsule with its canonical frame C.
 @param[in,out] mesh_C   A pointer to a tetrahedral mesh of the capsule. It
                         is aliased in the returned pressure field and must
                         remain alive as long as the field. The position
                         vectors of mesh vertices are expressed in the
                         capsule's frame C.
 @param[in] hydroelastic_modulus  Scale extent to pressure.
 @return                 The pressure field defined on the tetrahedral mesh.
 @pre                    `hydroelastic_modulus` is strictly positive.
                         `mesh_C` is non-null.
 @tparam T               The scalar type for representing the mesh vertex
                         positions and the pressure value.
 */
template <typename T>
VolumeMeshFieldLinear<T, T> MakeCapsulePressureField(
    const Capsule& capsule, const VolumeMesh<T>* mesh_C,
    const T hydroelastic_modulus) {
  DRAKE_DEMAND(hydroelastic_modulus > T(0));
  DRAKE_DEMAND(mesh_C != nullptr);
  // We only partially check the precondition of the mesh (see @pre). The first
  // two vertices should always be the endpoints of the capsule's medial axis.
  // The first with positive z and the second with negative z.
  DRAKE_DEMAND(mesh_C->vertex(0) ==
               Eigen::Vector3d(0, 0, capsule.length() / 2));
  DRAKE_DEMAND(mesh_C->vertex(1) ==
               Eigen::Vector3d(0, 0, -capsule.length() / 2));

  std::vector<T> pressure_values(mesh_C->num_vertices(), 0.0);

  // Only the inner vertices lying on the medial axis (vertex 0 and 1) have
  // non-zero pressure values.
  pressure_values[0] = pressure_values[1] = hydroelastic_modulus;

  return VolumeMeshFieldLinear<T, T>(std::move(pressure_values), mesh_C);
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
