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
 intersects the half space. The method tracks these created vertices using
 `vertices_to_newly_created_vertices` and `edges_to_newly_created_vertices`,
 respectively.

 @param vertices_F the vertices from the input mesh as position vectors measured
        and expressed in Frame F.
 @param triangle a single face from the input mesh.
 @param half_space_F the half space represented by its boundary plane such that
        the plane's normal points out of the half space, expressed in Frame F.
 @param[in,out] new_vertices_F the accumulator for all of the vertices in the
                intersecting mesh. It should be empty to start and will
                gradually accumulate all of the vertices with each subsequent
                call to this method.
 @param[in,out] new_faces the accumulator for all of the faces in the
                intersecting mesh. It should be empty to start and will
                gradually accumulate all of the faces with each subsequent
                call to this method.
 @param[in,out] vertices_to_newly_created_vertices the accumulated mapping from
                indices in `vertices_F` to indices in `new_vertices_F`. This
                mapping should be empty to start and will gradually accumulate
                elements with each subsequent call to this method.
 @param[in,out] edges_to_newly_created_vertices the accumulated mapping from
                pairs of indices in `vertices_F` to indices in `new_vertices_F`.
                This mapping should be empty to start and will gradually
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
    const std::vector<SurfaceVertex<T>>& vertices_F,
    const SurfaceFace& triangle,
    const PosedHalfSpace<T>& half_space_F,
    std::vector<SurfaceVertex<T>>* new_vertices_F,
    std::vector<SurfaceFace>* new_faces,
    std::unordered_map<SurfaceVertexIndex, SurfaceVertexIndex>*
        vertices_to_newly_created_vertices,
    std::unordered_map<SortedPair<SurfaceVertexIndex>, SurfaceVertexIndex>*
        edges_to_newly_created_vertices);

/*
 Constructs the SurfaceMesh that results from intersecting a triangle mesh with
 a half space.
 @param input_mesh_F the mesh with vertices measured and expressed in some
        frame, F.
 @param half_space_F the half space represented by its boundary plane such that
        the plane's normal points out of the half space, expressed in Frame F.
 @returns the SurfaceMesh corresponding to the intersection, the vertices of
          which will be measured and expressed in Frame F.
 */
template <typename T>
SurfaceMesh<T> ConstructSurfaceMeshFromMeshHalfspaceIntersection(
    const SurfaceMesh<T>& input_mesh_F,
    const PosedHalfSpace<T>& half_space_F);


}  // namespace internal
}  // namespace geometry
}  // namespace drake
