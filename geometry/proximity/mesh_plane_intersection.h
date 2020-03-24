#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "drake/common/sorted_pair.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/bounding_volume_hierarchy.h"
#include "drake/geometry/proximity/plane.h"
#include "drake/geometry/proximity/volume_mesh_field.h"
#include "drake/geometry/query_results/contact_surface.h"
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
 @param[in] field_M         The _linear_ volume mesh field (and mesh) containing
                            the tetrahedra to intersect. The vertex positions
                            are all measured and expressed in Frame M. The
                            linearity of the field is a strict requirement of
                            the algorithm.
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
void SliceTetWithPlane(VolumeElementIndex tet_index,
                       const VolumeMeshFieldLinear<double, double>& field_M,
                       const Plane<T>& plane_M,
                       const math::RigidTransform<T>& X_WM,
                       std::vector<SurfaceFace>* faces,
                       std::vector<SurfaceVertex<T>>* vertices_W,
                       std::vector<T>* surface_e,
                       std::unordered_map<SortedPair<VolumeVertexIndex>,
                                          SurfaceVertexIndex>* cut_edges);

/* Computes a ContactSurface by intersecting a plane with a set of tetrahedra
 drawn from the given volume mesh (and its pressure field). The indicated
 tetrahedra would typically be the result of broadphase culling.

 @param[in] mesh_id         The id associated with the volume mesh.
 @param[in] mesh_field_M    The _linear_ mesh field (and corresponding mesh,
                            mesh_M) from which we compute the ContactSurface.
                            The field (and the mesh vertices) are measured and
                            expressed in Frame M. The linearity of the field is
                            a strict requirement of the underlying algorithm.
 @param[in] plane_id        The id associated with the plane.
 @param[in] plane_M         The plane to intersect against the tetrahedra;
                            measured and expressed in the same frame M.
 @param[in] tet_indces      Indices for the tetrahedra in mesh_M to test against
                            the plane.
 @param[in] X_WM            The relative pose between the mesh frame M and the
                            world frame W. Used to guarantee that the contact
                            surface is measured and expressed in the world
                            frame.
 @returns `nullptr` if there is no intersection, otherwise the appropriate
           ContactSurface. The normals of the contact surface mesh will all
           be parallel with the plane normal.
 @pre  `i ∈ [0, N) ∀ i ∈ tet_indices`, where N is the number of tetrahedra in
       mesh_M.
*/
template <typename T>
std::unique_ptr<ContactSurface<T>> ComputeContactSurface(
    GeometryId mesh_id,
    const VolumeMeshFieldLinear<double, double>& mesh_field_M,
    GeometryId plane_id, const Plane<T>& plane_M,
    const std::vector<VolumeElementIndex> tet_indices,
    const math::RigidTransform<T>& X_WM);

// TODO(SeanCurtis-TRI): This is, in some sense, the "public" api. It refers to
//  half spaces. Does it belong in this file? Does it belong elsewhere? At the
//  end of the day, where does surface mesh-soft half space go?
/* Computes the ContactSurface formed by a rigid half space and a given
 soft mesh.

 The definition of the half space is implicit in the call -- it is the type
 defined by the HalfSpace class, thus, only its id and its pose in a common
 frame (the world frame) is necessary.

 @param[in] id_S        The id of the soft mesh.
 @param[in] field_S     The _linear_ mesh field (and its corresponding mesh
                        `mesh_S`) for the soft mesh. The field and mesh vertices
                        are measured and expressed in Frame S. The linearity of
                        the field is a strict requirement of the underlying
                        algorithm.
 @param[in] bvh_S       The bounding-volume hierarchy of the soft volume mesh
                        measured and expressed in Frame S.
 @param[in] X_WS        The relative pose of Frame S and the world frame W.
 @param[in] id_R        The id of the rigid half space.
 @param[in] X_WR        The relative pose between Frame R -- the frame the half
                        space is defined in -- and the world frame W.
 @returns `nullptr` if there is no collision, otherwise the ContactSurface
          between geometries S and R. The normals of the contact surface mesh
          will all be parallel with the plane normal.
 @tparam T The underlying scalar type. Must be a valid Eigen scalar.
 */
template <typename T>
std::unique_ptr<ContactSurface<T>>
ComputeContactSurfaceFromSoftVolumeRigidHalfSpace(
    const GeometryId id_S, const VolumeMeshFieldLinear<double, double>& field_S,
    const BoundingVolumeHierarchy<VolumeMesh<double>>& bvh_S,
    const math::RigidTransform<T>& X_WS, const GeometryId id_R,
    const math::RigidTransform<T>& X_WR);


}  // namespace internal
}  // namespace geometry
}  // namespace drake
