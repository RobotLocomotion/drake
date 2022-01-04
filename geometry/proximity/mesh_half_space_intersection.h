#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/sorted_pair.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/bvh.h"
#include "drake/geometry/proximity/contact_surface_utility.h"
#include "drake/geometry/proximity/posed_half_space.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/query_results/contact_surface.h"

namespace drake {
namespace geometry {
namespace internal {

/*
 Computes the intersection between a triangle and a half space.
 The intersecting geometry (e.g. vertices and triangles) are added to the
 provided mesh builder. This method is intended to be used in contexts where
 zero area triangles and triangles coplanar with the half space boundary are
 unimportant (see note below).

 This function is a component of the larger operation of clipping a
 TriangleSurfaceMesh by a half space.

 We do not introduce any duplicate vertices (beyond those already in the input
 mesh). This function uses bookkeeping to prevent the addition of such
 duplicate vertices, which can be created when either a vertex lies inside/on
 the half space or when an edge intersects the boundary of the half space. The
 method tracks these resultant vertices using
 `vertices_to_newly_created_vertices` and `edges_to_newly_created_vertices`,
 respectively.

 @param mesh_F
     The surface mesh to intersect, measured and expressed in Frame F.
 @param tri_index
     The index of the triangle, drawn from `mesh_F` to intersect with the half
     space.
 @param half_space_F
     The half space measured and expressed in the mesh's frame F.
 @param pressure_in_F
     A function that evaluates the pressure at a position X inside the half
     space measured and expressed in the common frame F.
 @param grad_pressure_in_W
     The gradient of the pressure field, expressed in the world frame.
 @param X_WF
     The relative pose between Frame F and Frame W.
 @param[in,out] builder_W
     The mesh builder that will add the resulting intersecting polygon to the
     mesh (along with sampled pressure values). The mesh being constructed has
     vertex positions measured and expressed in the world frame.
 @param[in,out] vertices_to_newly_created_vertices
     The accumulated mapping from indices in `mesh_F.vertices()` to indices in
     `new_vertices_W`. It should be empty for the first call and will gradually
     accumulate elements with each subsequent call to this method.
 @param[in,out] edges_to_newly_created_vertices
     The accumulated mapping from pairs of indices in `mesh_F.vertices()` to
     indices in `new_vertices_W`. It should be empty for the first call and
     will gradually accumulate elements with each subsequent call to this
     method.

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
template <typename MeshBuilder>
void ConstructTriangleHalfspaceIntersectionPolygon(
    const TriangleSurfaceMesh<double>& mesh_F, int tri_index,
    const PosedHalfSpace<typename MeshBuilder::ScalarType>& half_space_F,
    const std::function<typename MeshBuilder::ScalarType(
        const Vector3<typename MeshBuilder::ScalarType>&)>&
        pressure_in_F,
    const Vector3<typename MeshBuilder::ScalarType>& grad_pressure_in_W,
    const math::RigidTransform<typename MeshBuilder::ScalarType>& X_WF,
    MeshBuilder* builder_W,
    std::unordered_map<int, int>* vertices_to_newly_created_vertices,
    std::unordered_map<SortedPair<int>, int>* edges_to_newly_created_vertices);

/*
 Computes a ContactSurface between a half space and a collection of triangles
 drawn from a triangle surface mesh. The indicated triangles would typically be
 the result of broadphase culling.

 Each face in the contact surface's mesh corresponds to one triangle in
 `input_mesh_F` (although multiple contact surface faces may map to the same
 input triangle based on the ContactSurface's mesh representation). The
 correspondence is that the contact surface mesh face is always fully contained
 by a single triangle in the input mesh. As such, the face normal for each face
 in the contact surface mesh will point in the same direction as its
 corresponding triangle's in `input_mesh_F`.

 @param mesh_id
     The identifier for the surface mesh.
 @param input_mesh_F
     The mesh with vertices measured and expressed in some frame, F.
 @param half_space_id
     The identifier for the half space.
 @param half_space_F
     The half space measured and expressed in the common frame F.
 @param pressure_in_F
     A function that evaluates the pressure at a position X inside the half
     space measured and expressed in the common frame F.
 @param tri_indices
     A collection of triangle indices in `input_mesh_F`.
 @param X_WF
     The relative pose of Frame F with the world frame W.
 @param grad_p_W
     The gradient of the half space's pressure field, expressed in the world
     frame.
 @retval `mesh_W`, the TriangleSurfaceMesh corresponding to the intersection
         between the mesh and the half space; the vertices are measured and
         expressed in Frame W. `nullptr` if there is no intersection.
 */
template <typename MeshBuilder>
std::unique_ptr<ContactSurface<typename MeshBuilder::ScalarType>>
ComputeContactSurface(
    GeometryId mesh_id, const TriangleSurfaceMesh<double>& input_mesh_F,
    GeometryId half_space_id,
    const PosedHalfSpace<typename MeshBuilder::ScalarType>& half_space_F,
    const std::function<typename MeshBuilder::ScalarType(
        const Vector3<typename MeshBuilder::ScalarType>&)>&
        pressure_in_F,
    const Vector3<typename MeshBuilder::ScalarType>& grad_p_W,
    const std::vector<int>& tri_indices,
    const math::RigidTransform<typename MeshBuilder::ScalarType>& X_WF);

/*
 Computes the ContactSurface formed by a soft half space and the given rigid
 mesh.

 The definition of the half space is implicit in the call -- it is the type
 defined by the HalfSpace class, thus, only its id, its pose in a common frame
 (the world frame), and the `pressure_scale` is necessary.

 @param[in] id_S            The id of the soft half space.
 @param[in] X_WS            The relative pose of Frame S (the soft half space's
                            canonical frame) and the world frame W.
 @param[in] pressure_scale  A linear scale factor that transforms penetration
                            depth into pressure values. Generally,
                            `pressure_scale = hydroelastic_modulus / thickness`.
 @param[in] id_R            The id of the rigid mesh.
 @param[in] mesh_R          The rigid mesh. The mesh vertices are measured and
                            expressed in Frame R.
 @param[in] bvh_R           The bounding-volume hierarchy of the rigid surface
                            mesh measured and expressed in Frame R.
 @param[in] X_WR            The relative pose between Frame R -- the frame the
                            mesh is defined in -- and the world frame W.
 @param[in] representation  The preferred representation of each contact
                            polygon.
 @returns `nullptr` if there is no collision, otherwise the ContactSurface
          between geometries S and R. Each triangle in the contact surface is a
          piece of a triangle in the input `mesh_R`; the normals of the former
          will match the normals of the latter.
 @tparam T The underlying scalar type. Must be a valid Eigen scalar. Currently,
           only double is supported.
 */
template <typename T>
std::unique_ptr<ContactSurface<T>>
ComputeContactSurfaceFromSoftHalfSpaceRigidMesh(
    GeometryId id_S, const math::RigidTransform<T>& X_WS, double pressure_scale,
    GeometryId id_R, const TriangleSurfaceMesh<double>& mesh_R,
    const Bvh<Obb, TriangleSurfaceMesh<double>>& bvh_R,
    const math::RigidTransform<T>& X_WR,
    HydroelasticContactRepresentation representation);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
