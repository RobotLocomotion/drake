#pragma once

#include <memory>

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/bvh.h"
#include "drake/geometry/proximity/deformable_contact_geometries.h"
#include "drake/geometry/proximity/deformable_contact_surface.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

std::unique_ptr<DeformableContactSurface<double>>
ComputeContactSurfaceFromDeformableVolumeRigidSurface(
    const GeometryId id_D,
    const deformable::DeformableGeometry& deformable_W,
    const GeometryId id_R,
    const TriangleSurfaceMesh<double>& rigid_mesh_R,
    const Bvh<Obb, TriangleSurfaceMesh<double>>& rigid_bvh_R,
    const math::RigidTransform<double>& X_WR);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
