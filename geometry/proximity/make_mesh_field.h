#pragma once

#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/volume_mesh_field.h"

namespace drake {
namespace geometry {
namespace internal {

/* Creates a pressure field on a tetrahedral volume mesh of a (possibly
 non-convex) shape. This function complements MakeConvexPressureField().

 @param[in] mesh_M   A pointer to a tetrahedral mesh.
                     It is aliased in the returned pressure field and must
                     remain alive as long as the field.
 @param[in] hydroelastic_modulus    Scale extent to pressure.
 @return             The pressure field defined on the tetrahedral mesh.

 @pre                `hydroelastic_modulus` is strictly positive.
                     `mesh_M` is non-null.

 @tparam T          The scalar type for representing the mesh vertex
                    positions and the pressure value. It must be double
                    or AutoDiffXd.
 */
template <typename T>
VolumeMeshFieldLinear<T, T> MakeVolumeMeshPressureField(
    const VolumeMesh<T>* mesh_M, const T& hydroelastic_modulus);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
