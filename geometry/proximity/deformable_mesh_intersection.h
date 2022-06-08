#pragma once

#include <memory>
#include <vector>

#include "drake/geometry/proximity/bvh.h"
#include "drake/geometry/proximity/deformable_contact_geometries.h"
#include "drake/geometry/proximity/deformable_rigid_contact_surface.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

/* Computes the contact surface between a deformable geometry and a rigid
 geometry. If the intersection is empty, returns an empty
 DeformableRigidContactSurface.
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
*/
DeformableRigidContactSurface<double>
ComputeContactSurfaceFromDeformableVolumeRigidSurface(
    const GeometryId deformable_id,
    const deformable::DeformableGeometry& deformable_D,
    const GeometryId rigid_id, const TriangleSurfaceMesh<double>& rigid_mesh_R,
    const Bvh<Obb, TriangleSurfaceMesh<double>>& rigid_bvh_R,
    const math::RigidTransform<double>& X_DR);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
