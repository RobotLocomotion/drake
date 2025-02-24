#pragma once

#include "drake/geometry/mesh_source.h"
#include "drake/geometry/proximity/polygon_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

/* Creates a convex hull (represented by a PolygonSurfaceMesh) for the provided
 mesh data. The mesh data can come from on-disk files or in-memory files -- but
 the content must be that of a supported Drake mesh type.

 The convex hull is built upon *all* of the vertex values in the mesh data
 (regardless of how the mesh is organized or even if it includes vertices that
 are not otherwise incorporated in faces).

 The vertex positions can be scaled uniformly around the origin of the mesh
 data's canonical frame.

 The mesh can also be "inflated" by a given margin amount δ. If margin is zero,
 no inflation is applied and the convex hull of the original set of vertices is
 computed.

 With "inflation", we mean the process of moving each of the faces in the convex
 hull an amount δ along its (outwards) normal. This effectively grows or
 "inflates" the convex hull, producing an additional layer of thickness δ all
 around the convex hull of the original set of vertices.

 @note Geometry inflation by a margin δ is meant to support speculative
 contact constraints (contact constraints before contact actually happens) for
 "compliant" hydroelastics. Since the hydroelastic model is not meant for thin
 objects, inflation of planar (zero thickness) meshes is not implemented. Margin
 is ignored for planar meshes.

 @param mesh_source  The source of the mesh data.
 @param scale        The scale to apply to the vertex data -- the vertex
                     position vectors are scaled relative to the mesh data's
                     canonical frame's origin.
 @param margin       The margin amount δ, see @ref hydro_margin.

 @throws std::exception if the mesh data is in an unsupported format.
 @throws std::exception if the mesh data is ill formed.
 @throws std::exception if the mesh data is degenerate (insufficient numer of
                           vertices, co-linear or coincident vertices, etc.) All
                           of the vertices lying on a plane is *not* degenerate.
 @throws std::exception if there is an unforeseen error in computing the convex
                           hull.
 @throws std::exception if `margin` is negative. */
PolygonSurfaceMesh<double> MakeConvexHull(const MeshSource& mesh_source,
                                          const Vector3<double>& scale,
                                          double margin = 0);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
