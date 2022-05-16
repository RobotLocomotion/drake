#pragma once

#include <memory>

#include "drake/geometry/proximity/bvh.h"
#include "drake/geometry/proximity/deformable_contact_geometries.h"
#include "drake/geometry/proximity/deformable_contact_surface.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

/* Computes the contact surface between a deformable geometry and a rigid
 geometry.

 @param[in] deformable_W
     A deformable geometry expressed in World frame. It provides the
     deformed tetrahedral mesh and the approximated signed distance field.
 @param[in] rigid_mesh_R
     The rigid geometry is represented as a surface mesh, whose vertex
     positions are in frame R. We assume that triangles are oriented
     outward.
 @param[in] rigid_bvh_R
     A bounding volume hierarchy built on the geometry contained in
     `rigid_mesh_R`.
 @param[in] X_WR
     The pose of the rigid geometry's frame R in World frame.
 @return
     The deformable contact surface expressed in World frame. */
std::unique_ptr<DeformableContactSurface<double>>
ComputeContactSurfaceFromDeformableVolumeRigidSurface(
    const deformable::DeformableGeometry& deformable_W,
    const TriangleSurfaceMesh<double>& rigid_mesh_R,
    const Bvh<Obb, TriangleSurfaceMesh<double>>& rigid_bvh_R,
    const math::RigidTransform<double>& X_WR);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
