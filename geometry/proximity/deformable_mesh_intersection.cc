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
    const deformable::DeformableGeometry& deformable_W,
    const TriangleSurfaceMesh<double>& rigid_mesh_R,
    const Bvh<Obb, TriangleSurfaceMesh<double>>& rigid_bvh_R,
    const math::RigidTransform<double>& X_WR,
    std::vector<int>* tetrahedron_index_of_polygons) {
  DRAKE_ASSERT(tetrahedron_index_of_polygons != nullptr);
  tetrahedron_index_of_polygons->clear();

  // TODO(DamrongGuoy) Is there a better way than creating a new
  //  VolumeMeshFieldLinear here? We do it here, so we can reuse
  //  SurfaceVolumeIntersector. These are some ideas that Xuchen and Damrong
  //  consider for future refactoring:
  //  1. Change the type parameter MeshBuilder of SurfaceVolumeIntersector<>
  //     to provide the tetrahedral mesh. This includes
  //     DeformableContactBuilder, TriMeshBuilder and PolyMeshBuilder.
  //  Or 2. Allow VolumeMeshFieldLinear to switch to a different tetrahedral
  //        mesh while keeping the same field values at vertices. Assume that
  //        the new mesh has the same connectivity.
  //  Or 3. Pass an additional parameter for the tetrahedral mesh to
  //        SampleVolumeFieldOnSurface(). Right now it uses the mesh of the
  //        given VolumeMeshFieldLinear.
  VolumeMeshFieldLinear<double, double> field_W(
      std::vector<double>(deformable_W.signed_distance_field().values()),
      &deformable_W.deformable_mesh().mesh(), true /*calculate gradient*/);

  DeformableContactBuilder<double> builder(
      deformable_W.deformable_mesh().mesh());
  SurfaceVolumeIntersector<DeformableContactBuilder<double>, Aabb> intersect;
  intersect.SampleVolumeFieldOnSurface(
      field_W, deformable_W.deformable_mesh().bvh(),
      rigid_mesh_R, rigid_bvh_R,
      X_WR,
      &builder,
      false /* filter_face_normal_along_field_gradient */);
  if (!intersect.has_intersection()) {
    return {};
  }

  *tetrahedron_index_of_polygons = builder.tetrahedron_index_of_polygons();
  return std::make_unique<DeformableContactSurface<double>>(
      *intersect.release_field());
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
