#pragma once

#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/volume_mesh_field.h"

namespace drake {
namespace geometry {
namespace internal {

/* Creates a pressure field on a tetrahedral volume mesh of a (possibly
 non-convex) shape. This function complements MakeConvexPressureField().

 @note Unlike all other functions that compute pressure fields, this one in
 particular works on generally non-convex geometries. For all the other (convex)
 cases, `mesh_M` is meant to represent the "inflated" geometry by amount defined
 by `margin`. For this method however, `mesh_M` must correspond to the
 non-inflated geometry (the original mesh). This is a requirement since the
 distance function required in the computation of the pressure field might not
 be well defined for "inflated" meshes. Inflated meshes might contain cracks
 that, while negligible in the computation of forces, can affect the topology in
 ways that change the distance computation significantly.

 @note For interior vertices, this function estimates the distance field ϕ*(x)
 on the inflated mesh from the distance field ϕ(x) on the non-inflated mesh as
 ϕ*(x) = ϕ(x) + δ, with δ the margin value. This estimation is extending an
 amount δ the line connecting point x and its closest point on the boundary of
 the non-inflated mesh. For boundary vertices, the distance is ϕ*(x) = 0.

 Given the distance field ϕ*(x) (defined positive inside the object), we define
 the extent function as in the Elastic Foundation Model, i.e. e(x) =
 (-ϕ*(x)-δ)/(H-δ), where the elastic foundation depth H is defined as the
 maximum distance ϕ*(x) over the volume of the mesh (actually the maximum ϕ*(xᵢ)
 over all mesh vertices xᵢ). The pressure field is then defined as p(x) =
 E⋅e(x). Therefore the zero pressure level set is located at a distance ϕ* = δ
 from the surface, and the maximum pressure is p = E.

 @param[in] mesh_M   A pointer to a tetrahedral mesh.
                     It is aliased in the returned pressure field and must
                     remain alive as long as the field.
 @param[in] hydroelastic_modulus   Scale penetration extent to pressure.
                     Its unit is Pascals. See [Elandt2019].
 @param[in] margin   Margin δ.
 @return             The pressure field defined on the tetrahedral mesh.

 @pre                `hydroelastic_modulus` is strictly positive.
                     `mesh_M` is non-null.
 @pre               H > δ.

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
    const VolumeMesh<T>* mesh_M, const T& hydroelastic_modulus,
    double margin = 0);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
