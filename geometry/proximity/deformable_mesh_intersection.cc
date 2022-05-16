#include "drake/geometry/proximity/deformable_mesh_intersection.h"

#include <utility>
#include <vector>

#include "drake/geometry/proximity/deformable_contact_surface_utility.h"
#include "drake/geometry/proximity/deformable_volume_mesh.h"
#include "drake/geometry/proximity/mesh_intersection.h"

namespace drake {
namespace geometry {
namespace internal {

std::unique_ptr<ContactSurface<double>>
ComputeContactSurfaceFromDeformableVolumeRigidSurface(
    const GeometryId deformable_id,
    const deformable::DeformableGeometry& deformable_W,
    const GeometryId rigid_id,
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

  DeformableContactBuilder<double> builder;
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

  // The contact surface is documented as having the normals pointing *out*
  // of the second surface and into the first. This mesh intersection
  // creates a surface mesh with normals pointing out of the rigid surface,
  // so we make sure the ids are ordered so that the rigid is the second id.
  auto contact_surface = std::make_unique<ContactSurface<double>>(
      deformable_id, rigid_id, intersect.release_mesh(),
      intersect.release_field(),
      std::make_unique<std::vector<Vector3<double>>>(
          std::move(intersect.mutable_grad_eM_M())),
      nullptr);

  return contact_surface;
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
