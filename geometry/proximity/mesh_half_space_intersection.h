#pragma once

#include <unordered_map>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/sorted_pair.h"
#include "drake/geometry/proximity/posed_half_space.h"
#include "drake/geometry/proximity/surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

/*
 Computes the intersection between a triangle and a half space.
 The intersecting geometry (e.g. vertices and triangles) are added to the
 provided collections. This method is intended to be used in contexts where
 zero area triangles and triangles coplanar with the half space boundary are
 unimportant (see note below).

 This function is a component of the larger operation of clipping a
 SurfaceMesh by a half space. When doing that, we don't want to introduce any
 duplicate vertices (beyond those already in the input mesh). This function
 uses bookkeeping to prevent the addition of such duplicate vertices, which can
 be created when either a vertex lies inside/on the half space or when an edge
 intersects the half space. The method tracks these resultant vertices using
 `vertices_to_newly_created_vertices` and `edges_to_newly_created_vertices`,
 respectively.

 @param mesh_F
     The surface mesh to intersect, measured and expressed in Frame F.
 @param tri_index
     The index of the triangle, drawn from `mesh_F` to intersect with the half
     space.
 @param half_space_F
     The half space measured and expressed in Frame F.
 @param X_WF
     The relative pose between Frame F and Frame W.
 @param[in,out] new_vertices_W
     The accumulator for all of the vertices in the intersecting mesh. It should
     be empty for the first call and will gradually accumulate all of the
     vertices with each subsequent call to this method. The vertex positions are
     measured and expressed in frame W.
 @param[in,out] new_faces
     The accumulator for all of the faces in the intersecting mesh. It should be
     empty for the first call and will gradually accumulate all of the faces
     with each subsequent call to this method.
 @param[in,out] vertices_to_newly_created_vertices
     The accumulated mapping from indices in `vertices_F` to indices in
     `new_vertices_W`. It should be empty for the first call and will gradually
     accumulate elements with each subsequent call to this method.
 @param[in,out] edges_to_newly_created_vertices
     The accumulated mapping from pairs of indices in `vertices_F` to indices in
     `new_vertices_W`. It should be empty for the first call and will gradually
     accumulate elements with each subsequent call to this method.

 @note Unlike most geometric intersection routines, this method does not
       require the user to provide (or the algorithm to compute) a reasonable
       floating point tolerance for zero. This simple interface implies both
       that the method may construct degenerate triangles and that it may
       fail to register an intersection with a triangle that is coplanar with
       the half space surface. In certain applications (e.g., hydroelastic
       contact), both degenerate triangles and triangles coplanar with the
       half space surface are innocuous (in hydroelastic contact, for example,
       both cases contribute nothing to the contact wrench).
*/
template <typename T>
void ConstructTriangleHalfspaceIntersectionPolygon(
    const SurfaceMesh<double>& mesh_F, SurfaceFaceIndex tri_index,
    const PosedHalfSpace<T>& half_space_F, const math::RigidTransform<T>& X_WF,
    std::vector<SurfaceVertex<T>>* new_vertices_W,
    std::vector<SurfaceFace>* new_faces,
    std::unordered_map<SurfaceVertexIndex, SurfaceVertexIndex>*
        vertices_to_newly_created_vertices,
    std::unordered_map<SortedPair<SurfaceVertexIndex>, SurfaceVertexIndex>*
        edges_to_newly_created_vertices);

/*
 Computes a triangular surface mesh by intersecting a half space with a set of
 triangles drawn from the given surface mesh. The indicated triangles would
 typically be the result of broadphase culling.

 Each triangle in `mesh_W` is a proper subset of one triangle in `input_mesh_F`
 (i.e., fully contained within the triangle in `input_mesh_F`). As such,
 the face normal for each triangle in `mesh_W` will point in the same direction
 as its containing triangle in `input_mesh_F`.

 @param input_mesh_F
     The mesh with vertices measured and expressed in some frame, F.
 @param half_space_F
     The half space measured and expressed in Frame F.
 @param tri_indices
     A collection of triangle indices in `input_mesh_F`.
 @param X_WF
     The relative pose of Frame F with the world frame W.
 @retval `mesh_W`, the SurfaceMesh corresponding to the intersection between
         the mesh and the half space; the vertices are measured and expressed in
         Frame W.
 */
template <typename T>
SurfaceMesh<T> ConstructSurfaceMeshFromMeshHalfspaceIntersection(
    const SurfaceMesh<double>& input_mesh_F,
    const PosedHalfSpace<T>& half_space_F,
    const std::vector<SurfaceFaceIndex>& tri_indices,
    const math::RigidTransform<T>& X_WF);


}  // namespace internal
}  // namespace geometry
}  // namespace drake
