#include "drake/geometry/proximity/deformable_mesh_intersection.h"

#include <vector>

#include "drake/geometry/proximity/deformable_contact_surface_utility.h"
#include "drake/geometry/proximity/deformable_volume_mesh.h"
#include "drake/geometry/proximity/mesh_intersection.h"

namespace drake {
namespace geometry {
namespace internal {

std::unique_ptr<DeformableContactSurface<double>>
ComputeContactSurfaceFromDeformableVolumeRigidSurface(
    const GeometryId,  // id_D,
    const deformable::DeformableGeometry& deformable_W,
    const GeometryId,  // id_R,
    const TriangleSurfaceMesh<double>& rigid_mesh_R,
    const Bvh<Obb, TriangleSurfaceMesh<double>>& rigid_bvh_R,
    const math::RigidTransform<double>& X_WR) {
  // TODO(DamrongGuoy) Is there a better way than creating a new mesh field
  //  linear here? We do it only to reuse SurfaceVolumeIntersector.
  VolumeMeshFieldLinear<double, double> field_W(
      std::vector<double>(deformable_W.signed_distance_field().values()),
      &deformable_W.deformable_mesh().mesh(), true /*calculate gradient*/);

  SurfaceVolumeIntersector<DeformableContactBuilder<double>, Aabb> intersect;
  intersect.SampleVolumeFieldOnSurface(
      field_W, deformable_W.deformable_mesh().bvh(),
      rigid_mesh_R, rigid_bvh_R,
      X_WR,
      DeformableContactBuilder<double>(deformable_W.deformable_mesh().mesh()),
      false /* filter_face_normal_along_field_gradient */);
  if (!intersect.has_intersection()) {
    return {};
  }

  return std::make_unique<DeformableContactSurface<double>>(
      *intersect.release_field());
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
