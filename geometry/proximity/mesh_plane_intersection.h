#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "drake/common/sorted_pair.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/bvh.h"
#include "drake/geometry/proximity/contact_surface_utility.h"
#include "drake/geometry/proximity/plane.h"
#include "drake/geometry/proximity/volume_mesh_field.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

/* Intersects a tetrahedron with a plane, the resulting polygon is passed
 into the provided MeshBuilder.

 This method constructs a mesh by a sequence of invocations. It guarantees
 the output surface mesh has the same topological coherency as the input mesh.
 For example, if the plane cuts through a tetrahedron edge e at a point P, then
 each of the tetrahedra incident to that edge will be cut into a polygon on
 the plane which all share a _single_ vertex at P; no duplicate vertices will be
 introduced.

 This is accomplished by storing the edges that have already been evaluated.
 Call after call, the cached set of intersected edges grows and subsequent
 calls look up edges in the cache to see if the plane-edge intersection has
 already been accounted for.

 The unique vertices, their corresponding pressure values, and the unique
 faces are all stored within the MeshBuilder; only their indices are stored
 in the cache.

 The face vertices are ordered such that the normal implied by their winding
 points in the direction of the plane's normal.

 If there is no intersection, then neither the mesh, nor the cache data
 structures will change.

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
 @param[in,out] builder_W   The mesh builder that will add the resulting
                            intersecting polygon to the mesh (along with sampled
                            pressure values). It builds the mesh in the world
                            frame (requiring polygon quantities to be expressed
                            appropriately).
 @param[in,out] cut_edges   The cache of volume mesh edges that have already
                            been cut and the index of the surface mesh vertex
                            (indexing into vertices_W) that represents the cut
                            point.
 @returns The number of faces added.
 @pre `tet_index` lies in the range `[0, field_M.mesh().num_elements())`.
 */
template <typename MeshBuilder>
int SliceTetWithPlane(
    int tet_index, const VolumeMeshFieldLinear<double, double>& field_M,
    const Plane<typename MeshBuilder::ScalarType>& plane_M,
    const math::RigidTransform<typename MeshBuilder::ScalarType>& X_WM,
    MeshBuilder* builder_W,
    std::unordered_map<SortedPair<int>, int>* cut_edges);

/* Computes a ContactSurface by intersecting a plane with a set of tetrahedra
 drawn from the given volume mesh (and its pressure field). The indicated
 tetrahedra would typically be the result of broadphase culling.

 The mesh representation of the resulting contact surface is defined by the type
 of MeshBuilder provided: polygon or triangle.

 @param[in] mesh_id         The id associated with the volume mesh.
 @param[in] mesh_field_M    The _linear_ mesh field (and corresponding mesh,
                            mesh_M) from which we compute the ContactSurface.
                            The field (and the mesh vertices) are measured and
                            expressed in Frame M. The linearity of the field is
                            a strict requirement of the underlying algorithm.
 @param[in] plane_id        The id associated with the plane.
 @param[in] plane_M         The plane to intersect against the tetrahedra;
                            measured and expressed in the same frame M.
 @param[in] tet_indices     Indices for the tetrahedra in mesh_M to test against
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
template <typename MeshBuilder>
std::unique_ptr<ContactSurface<typename MeshBuilder::ScalarType>>
ComputeContactSurface(
    GeometryId mesh_id,
    const VolumeMeshFieldLinear<double, double>& mesh_field_M,
    GeometryId plane_id, const Plane<typename MeshBuilder::ScalarType>& plane_M,
    const std::vector<int>& tet_indices,
    const math::RigidTransform<typename MeshBuilder::ScalarType>& X_WM);

// TODO(SeanCurtis-TRI): This is, in some sense, the "public" api. It refers to
//  half spaces. Does it belong in this file? Does it belong elsewhere? At the
//  end of the day, where does surface mesh-soft half space go?
/* Computes the ContactSurface formed by a rigid half space and a given
 soft mesh.

 The definition of the half space is implicit in the call -- it is the type
 defined by the HalfSpace class, thus, only its id and its pose in a common
 frame (the world frame) is necessary.

 @param[in] id_S             The id of the soft mesh.
 @param[in] field_S          The _linear_ mesh field (and its corresponding mesh
                             `mesh_S`) for the soft mesh. The field and mesh
                             vertices are measured and expressed in Frame S. The
                             linearity of the field is a strict requirement of
                             the underlying algorithm.
 @param[in] bvh_S            The bounding-volume hierarchy of the soft volume
                             mesh measured and expressed in Frame S.
 @param[in] X_WS             The relative pose of Frame S and the world frame W.
 @param[in] id_R             The id of the rigid half space.
 @param[in] X_WR             The relative pose between Frame R -- the frame the
                             half space is defined in -- and the world frame W.
 @param[in] representation   The preferred representation of each contact
                             polygon.
 @returns `nullptr` if there is no collision, otherwise the ContactSurface
          between geometries S and R. The normals of the contact surface mesh
          will all be parallel with the plane normal.
 @tparam_nonsymbolic_scalar
 */
template <typename T>
std::unique_ptr<ContactSurface<T>>
ComputeContactSurfaceFromSoftVolumeRigidHalfSpace(
    const GeometryId id_S, const VolumeMeshFieldLinear<double, double>& field_S,
    const Bvh<Obb, VolumeMesh<double>>& bvh_S,
    const math::RigidTransform<T>& X_WS, const GeometryId id_R,
    const math::RigidTransform<T>& X_WR,
    HydroelasticContactRepresentation representation);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
