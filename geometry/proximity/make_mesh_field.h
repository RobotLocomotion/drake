#pragma once

#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/volume_mesh_field.h"

namespace drake {
namespace geometry {
namespace internal {

/* Creates a pressure field on a tetrahedral volume mesh of a (possibly
 non-convex) shape. This function complements MakeConvexPressureField().

 We use distance to surface to approximate strain, so we calculate the
 fraction of a vertex's depth to the maximum depth among all vertices in
 the volume.  Using distance this way can produce bad gradients since a
 distance field is non-smooth, and this is not the same as the extent-field
 method in [Elandt2019] which uses Laplace's equation to calculate a smooth
 "temperature gradient" for the strain field.

 We will use a smoother field in the future. For now, mathematically the value
 of the pressure field P(v) at vertex v is:

   P(v) = hydroelastic_modulus * dist(v, ∂M) / dist(v*, ∂M)

 where:
 ∂M is the boundary of the volume mesh M,
 dist(v, ∂M) = inf |v − w| : w ∈ ∂M (distance to the boundary),
 v* = arg max dist(v, ∂M) : v ∈ M

 @param[in] mesh_M   A pointer to a tetrahedral mesh.
                     It is aliased in the returned pressure field and must
                     remain alive as long as the field.
 @param[in] hydroelastic_modulus   Scale penetration extent to pressure.
                     Its unit is Pascals. See [Elandt2019].
 @return             The pressure field defined on the tetrahedral mesh.

 @pre                `hydroelastic_modulus` is strictly positive.
                     `mesh_M` is non-null.

 @throw  std::exception if the mesh has no interior vertices.

 @tparam T          The scalar type for representing the mesh vertex
                    positions and the pressure value. It must be double
                    or AutoDiffXd.

  [Elandt2019]    R. Elandt, E. Drumwright, M. Sherman, and A. Ruina. A
                  pressure field model for fast, robust approximation of net
                  contact force and moment between nominally rigid objects.
                  IROS 2019: 8238-8245.
 */
template <typename T>
VolumeMeshFieldLinear<T, T> MakeVolumeMeshPressureField(
    const VolumeMesh<T>* mesh_M, const T& hydroelastic_modulus);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
