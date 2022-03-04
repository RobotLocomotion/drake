#pragma once

#include <vector>

#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

/* Generates a tetrahedral volume mesh that partitions the interior of a convex
triangle mesh. The resulting mesh is computed by computing a single vertex on
the interior of the input mesh, namely the centroid of the volume enclosed
assuming the volume has uniform density. Then for each triangle of the surface
mesh (pqr) a tetrahedron (pqrc) is created, where 'c' is the centroid of the
enclosed volume. Consideration is taken to orient the vertices such that the
resulting tetrahedron has positive signed volume. The set of tetrahedra
generated partition the volume enclosed by the surface mesh and the surface of
the resulting volume mesh is identical to the input surface mesh. More formally
speaking the resulting volume mesh is a conformal simplicial complex with a
single internal vertex c. The link of c,
link(c) = closure(star(c)) \ star(closure(c)) is the input: `surface_mesh`.

For help visualizing this 3D operation we provide the following 2D analog:

v0─────────────────────v3    v0─────────────────────v3
 │                     │      │ \_               _/ │
 │                     │      │   \_           _/   │
 │                     │      │     \_       _/     │
 │                     │      │       \_   _/       │
 │                     │      │         \ /         │
 │                     │ ───► │         _c_         │
 │                     │      │       _/   \_       │
 │                     │      │     _/       \_     │
 │                     │      │   _/           \_   │
 │                     │      │ _/               \_ │
 │                     │      │/                   \│
v1─────────────────────v2    v1─────────────────────v2

The centroid, c, of the polygon (v0,v1,v2,v3) is computed. Then for each 2D
facet {(v0,v1), (v1,v2), (v2,v3), (v3,v0)} the following 3D facets are created:
{(v0,v1,c), (v1,v2,c), (v2,v3,c), (v3,v1,c)}.

 @param[in] surface_mesh
     The triangle surface mesh representing the surface of the resulting
     polyhedral volume mesh.
 @pre We make the assumption that `surface_mesh` is sufficiently "nice".
      That is:
        - The closure of the interior of the mesh is a convex set.
        - The mesh is closed and watertight.
        - The surface normals of each face consistently either inwardly or
          outwardly oriented.
 @retval volume_mesh
 @tparam_nonsymbolic_scalar
 */
template <typename T>
VolumeMesh<T> MakeConvexVolumeMesh(
    const TriangleSurfaceMesh<double>& surface_mesh);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
