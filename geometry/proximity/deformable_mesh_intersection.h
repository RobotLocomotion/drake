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

/* Computes the contact surface between a deformable geometry and a rigid
 geometry.
 @param[in] deformable_id
     Id of the deformable geometry
 @param[in] deformable_W
     A deformable geometry expressed in World frame. It provides the
     deformed tetrahedral mesh and the approximated signed distance field.
 @param[in] rigid_id
     Id of the rigid geometry
 @param[in] rigid_mesh_R
     The rigid geometry is represented as a surface mesh, whose vertex
     positions are in frame R. We assume that triangles are oriented
     outward.
 @param[in] rigid_bvh_R
     A bounding volume hierarchy built on the geometry contained in
     `rigid_mesh_R`.
 @param[in] X_WR
     The pose of the rigid geometry's frame R in World frame.
 @param[out] tetrahedron_index_of_polygons
     Tetrahedron index of each contact polygon. The sequence of tetrahedron
     indices is parallel to the sequence of contact polygons. If there is no
     contact, it becomes an empty sequence.

 @return
     The contact surface expressed in World frame with surface normal pointing
     in the direction described in ContactSurface according to deformable_id
     and rigid_id.

 @pre  tetrahedron_index_of_polygons is not a nullptr.   */
std::unique_ptr<ContactSurface<double>>
ComputeContactSurfaceFromDeformableVolumeRigidSurface(
    const GeometryId deformable_id,
    const deformable::DeformableGeometry& deformable_W,
    const GeometryId rigid_id,
    const TriangleSurfaceMesh<double>& rigid_mesh_R,
    const Bvh<Obb, TriangleSurfaceMesh<double>>& rigid_bvh_R,
    const math::RigidTransform<double>& X_WR,
    std::vector<int>* tetrahedron_index_of_polygons);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
