#pragma once

#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

/* Generates a tetrahedral volume mesh of the convex region enclosed by a
closed triangle surface mesh. The resulting mesh contains a vertex at the
centroid of the enclosed volume. Each tetrahedron is formed by connecting the
centroid to a surface triangle. Consideration is taken to orient the vertices
such that each tetrahedron has a positive signed volume.

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

 @param[in] convex
     The Convex shape specification describing the triangle surface mesh.
 @pre We make the assumption that `surface_mesh` is sufficiently "nice".
      That is:
        - The closure of the interior of the mesh is a convex set.
        - The mesh is closed and watertight.
        - The surface normals of all faces are consistently outwardly oriented.
 @retval volume_mesh
 @tparam_nonsymbolic_scalar
 */
template <typename T>
VolumeMesh<T> MakeConvexVolumeMesh(const Convex& convex);

/* Also creates a tetrahedral volume mesh, but, this time, from a triangle mesh.
 @pre `surface_mesh` is sufficiently "nice" (see above). */
template <typename T>
VolumeMesh<T> MakeConvexVolumeMesh(
    const TriangleSurfaceMesh<double>& surface_mesh);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
