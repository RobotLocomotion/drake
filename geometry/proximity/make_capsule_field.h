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

 With r the distance to the capsule's medial axis, the extent is defined as e(r)
 = 1 - r/(R-δ), where R is the radius of the capsule and δ the margin. The
 pressure field is then defined as p(r) = E * e(r), with E the hydroelastic
 modulus. The zero level set of this pressure field lies on a capsule of the
 same length (Capsule::length(), the length of the cylindrical region) and
 radius equal to R-δ. The maximum pressure field is E at the medial axis, i.e.
 the line segment connecting the first two vertices of the input mesh.

 @param[in] capsule      The capsule with its canonical frame C.
 @param[in,out] mesh_C   A pointer to a tetrahedral mesh of the capsule. It
                         is aliased in the returned pressure field and must
                         remain alive as long as the field. The position
                         vectors of mesh vertices are expressed in the
                         capsule's frame C.
 @param[in] hydroelastic_modulus  Scale extent to pressure.
 @param[in] margin       The margin δ.
 @return                 The pressure field defined on the tetrahedral mesh.
 @pre                    `hydroelastic_modulus` is strictly positive.
                         `mesh_C` is non-null.
 @tparam T               The scalar type for representing the mesh vertex
                         positions and the pressure value.
 */
template <typename T>
VolumeMeshFieldLinear<T, T> MakeCapsulePressureField(
    const Capsule& capsule, const VolumeMesh<T>* mesh_C,
    const T hydroelastic_modulus, const double margin = 0) {
  DRAKE_DEMAND(hydroelastic_modulus > T(0));
  DRAKE_DEMAND(capsule.radius() > margin);
  DRAKE_DEMAND(mesh_C != nullptr);
  // We only partially check the precondition of the mesh (see @pre). The first
  // two vertices should always be the endpoints of the capsule's medial axis.
  // The first with positive z and the second with negative z.
  DRAKE_DEMAND(mesh_C->vertex(0) ==
               Eigen::Vector3d(0, 0, capsule.length() / 2));
  DRAKE_DEMAND(mesh_C->vertex(1) ==
               Eigen::Vector3d(0, 0, -capsule.length() / 2));

  // Pressure gradient along the radial direction (in either the sphere or the
  // cylinder).
  const T pressure_gradient =
      hydroelastic_modulus / (capsule.radius() - margin);

  // Compute pressure at the surface by linearly extrapolating to a distance
  // equal to the margin. The extrapolation is exact since the field is linear
  // along the radial coordinate (in either the spherical or cylindrical
  // regions).
  const T surface_pressure = -margin * pressure_gradient;

  // All vertices are on the surface except the first two (overwritten below).
  std::vector<T> pressure_values(mesh_C->num_vertices(), surface_pressure);

  // Only the inner vertices lying on the medial axis (vertex 0 and 1) have
  // non-zero pressure values.
  pressure_values[0] = pressure_values[1] = hydroelastic_modulus;

  return VolumeMeshFieldLinear<T, T>(std::move(pressure_values), mesh_C,
                                     MeshGradientMode::kOkOrMarkDegenerate);
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
