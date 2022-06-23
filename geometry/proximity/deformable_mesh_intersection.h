#pragma once

#include <memory>
#include <vector>

#include "drake/geometry/proximity/bvh.h"
#include "drake/geometry/proximity/deformable_contact_geometries.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

// TODO(DamrongGuoy): Redefine DeformableContactSurface to have
//  {ContactSurface, tetrahedron indices of polygons, and
//  barycentric coordinates of centroids of polygons}. Then, return the
//  DeformableContactSurface instead of returning ContactSurface with two
//  output parameters. Possibly do it as a part of
//  https://github.com/RobotLocomotion/drake/pull/17383.

/* Computes the contact surface between a deformable geometry and a rigid
 geometry.
 @param[in] deformable_id
     Id of the deformable geometry.
 @param[in] deformable_D
     A deformable geometry expressed in frame D. It provides the
     deformed tetrahedral mesh and the approximated signed distance field.
 @param[in] rigid_id
     Id of the rigid geometry.
 @param[in] rigid_mesh_R
     The rigid geometry is represented as a surface mesh, whose vertices are
     measusured and expressed in frame R. We assume that triangles are oriented
     outward.
 @param[in] rigid_bvh_R
     A bounding volume hierarchy built on the geometry contained in
     rigid_mesh_R.
 @param[in] X_DR
     The pose of the rigid geometry's frame R in deformable geometry's frame D.
 @param[out] tetrahedron_index_of_polygons
     Each contact polygon is completely contained within one tetrahedron of the
     deformable mesh. For the i-th contact polygon in the contact surface,
     tetrahedron_index_of_polygons[i] contains the index of the containing
     tetrahedron.
 @param[out] barycentric_centroids
     Barycentric coordinates of centroids of contact polygons with respect to
     their containing tetrahedra with the same index semantics as
     `tetrahedron_index_of_poloygons`.

 @retval contact_surface_W
     The contact surface expressed in World frame with surface normals pointing
     in the direction as described in the documentation for ContactSurface's
     constructors.

 @note contact_surface_W.num_faces() == tetrahedron_index_of_polygons.size().
       contact_surface_W.num_faces() == barycentric_centroids.size().
 @pre  tetrahedron_index_of_polygons is not nullptr.
 @pre  barycentric_centroids is not nullptr.  */
std::unique_ptr<ContactSurface<double>>
ComputeContactSurfaceFromDeformableVolumeRigidSurface(
    const GeometryId deformable_id,
    const deformable::DeformableGeometry& deformable_D,
    const GeometryId rigid_id,
    const TriangleSurfaceMesh<double>& rigid_mesh_R,
    const Bvh<Obb, TriangleSurfaceMesh<double>>& rigid_bvh_R,
    const math::RigidTransform<double>& X_DR,
    std::vector<int>* tetrahedron_index_of_polygons,
    std::vector<VolumeMesh<double>::Barycentric<double>>*
        barycentric_centroids);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
