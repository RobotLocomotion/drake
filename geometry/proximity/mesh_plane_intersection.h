#pragma once

#include <unordered_map>
#include <vector>

#include "drake/common/sorted_pair.h"
#include "drake/geometry/proximity/plane.h"
#include "drake/geometry/proximity/volume_mesh_field.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

/* Intersects a tetrahedron with a plane, storing the result in the provided
 collections.

 This method constructs a mesh by a sequence of invocations. It guarantees
 the output surface mesh has the same topological coherency as the input mesh.
 For example, if the plane cuts through a tetrahedron edge e at vertex v, then
 each of the tetrahedra incident to that edge will be cut into a polygon which
 includes the same vertex v; no duplicate vertices will be introduced.

 This is accomplished by storing the edges that have already been evaluated.
 Call after call, the cached set of intersected edges grows and subsequent
 calls look up edges in the cache to see if the plane-edge intersection has
 already been accounted for.

 The unique vertices are written to `vertices_W`, the unique triangles are
 written to `faces`, and the pressure values at each intersection vertex are
 written to `surface_e`.

 The face vertices are ordered such that the normal implied by their winding
 points in the direction of the plane's normal.

 After each call the following invariants are maintained:

   - `vertices_W.size() == surface_e.size()`
   - `i ∈ [0, N) ∀ i` referenced in `faces` (and `N = vertices_W.size()`).
   - The iᵗʰ entry in `surface_e` is the pressure at the position for the iᵗʰ
     vertex in `vertices_W`.
   - Every vertex position in `vertices_W` that represents an intersection of
     edge and plane can be found in `cut_edges`. All remaining vertex
     positions are the centroids of the intersected polygon.
   - No intersection guarantees no changes to _any_ of the out parameters.

 @param[in] tet_index       The index of the tetrahedron to attempt to
                            intersect.
 @param[in] field_M         The volume mesh field (and mesh) containing the
                            tetrahedra to intersect. The vertex positions are
                            all measured and expressed in Frame M.
 @param[in] plane_M         The definition of a plane measured and expressed
                            in Frame M.
 @param[in] X_WM            The relative pose between the mesh frame M and the
                            world frame W.
 @param[in,out] faces       The triangles (defined by triples of vertex
                            indices) forming the intersection mesh so far.
 @param[in,out] vertices_W  The vertex positions for the intersecting mesh,
                            measured and expressed in Frame W.
 @param[in,out] surface_e   The per-vertex field values. Upon returning,
                            surface_e.size() == vertices_W.size() is true.
 @param[in,out] cut_edges   The cache of volume mesh edges that have already
                            been cut and the surface mesh vertex associated
                            with it.
 @pre `tet_index` lies in the range `[0, field_M.mesh().num_elements())`.
 */
template <typename T>
void SliceTetWithPlane(
    VolumeElementIndex tet_index,
    const VolumeMeshField<double, double>& field_M, const Plane<T>& plane_M,
    const math::RigidTransform<T>& X_WM, std::vector<SurfaceFace>* faces,
    std::vector<SurfaceVertex<T>>* vertices_W, std::vector<T>* surface_e,
    std::unordered_map<SortedPair<VolumeVertexIndex>, SurfaceVertexIndex>*
        cut_edges);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
