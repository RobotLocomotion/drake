#include "drake/geometry/proximity/deformable_mesh_intersection.h"

#include <utility>
#include <vector>

#include "drake/geometry/proximity/contact_surface_utility.h"
#include "drake/geometry/proximity/deformable_contact_surface_utility.h"
#include "drake/geometry/proximity/deformable_volume_mesh.h"
#include "drake/geometry/proximity/mesh_intersection.h"

namespace drake {
namespace geometry {
namespace internal {

// TODO(DamrongGuoy) Declare DeformableSurfaceVolumeIntersector in the header
//  file to test it directly and for future code re-use.
class DeformableSurfaceVolumeIntersector : public
    SurfaceVolumeIntersector<PolyMeshBuilder<double>, Aabb> {
 public:
  void SampleVolumeFieldOnSurface(
      const VolumeMeshFieldLinear<double, double>& volume_field_W,
      const Bvh<Aabb, VolumeMesh<double>>& bvh_W,
      const TriangleSurfaceMesh<double>& surface_N,
      const Bvh<Obb, TriangleSurfaceMesh<double>>& bvh_N,
      const math::RigidTransform<T>& X_WN) {
    PolyMeshBuilder<double> builder;
    const bool filter_face_normal_along_field_gradient = false;
    SurfaceVolumeIntersector<PolyMeshBuilder<double>, Aabb>::
        SampleVolumeFieldOnSurface(volume_field_W, bvh_W,
                                   surface_N, bvh_N, X_WN,
                                   &builder,
                                   filter_face_normal_along_field_gradient);
  }

  /* Returns indices of tetrahedra containing the contact polygons.
   @pre Call it after the rigid-deformable mesh intersection finished.
  */
  const std::vector<int>& tetrahedron_index_of_polygons() {
    return tetrahedron_index_of_polygons_;
  }

  /* Returns barycentric coordinates of the centroids of the contact polygons.
   @pre Call it after the rigid-deformable mesh intersection finished.
   */
  const std::vector<VolumeMesh<double>::Barycentric<double>>&
  barycentric_centroids() {
    return barycentric_centroids_;
  }

 protected:
  void CalcContactPolygon(
      const VolumeMeshFieldLinear<double, double>& volume_field_M,
      const TriangleSurfaceMesh<double>& surface_N,
      const math::RigidTransform<T>& X_MN,
      const math::RigidTransform<double>& X_MN_d,
      PolyMeshBuilder<double>* builder_M,
      const bool filter_face_normal_along_field_gradient,
      const int tet_index, const int tri_index) override {
    const int num_vertices_before = builder_M->num_vertices();
    const int num_polygons_before = builder_M->num_faces();
    SurfaceVolumeIntersector<PolyMeshBuilder<double>, Aabb>::CalcContactPolygon(
        volume_field_M, surface_N, X_MN, X_MN_d, builder_M,
        filter_face_normal_along_field_gradient, tet_index, tri_index);
    const int num_vertices_after = builder_M->num_vertices();
    const int num_polygons_after = builder_M->num_faces();

    if (num_polygons_after == num_polygons_before) {
      return;
    }
    DRAKE_DEMAND(num_polygons_after == num_polygons_before + 1);

    tetrahedron_index_of_polygons_.push_back(tet_index);

    std::vector<int> polygon(num_vertices_after - num_vertices_before);
    std::iota(polygon.begin(), polygon.end(), num_vertices_before);

    barycentric_centroids_.push_back(volume_field_M.mesh().CalcBarycentric(
        CalcPolygonCentroid(
            polygon, X_MN_d.rotation() * surface_N.face_normal(tri_index),
            builder_M->vertices()),
        tet_index));
  }

 private:
  std::vector<int> tetrahedron_index_of_polygons_{};
  std::vector<VolumeMesh<double>::Barycentric<double>> barycentric_centroids_{};
};

std::unique_ptr<ContactSurface<double>>
ComputeContactSurfaceFromDeformableVolumeRigidSurface(
    const GeometryId deformable_id,
    const deformable::DeformableGeometry& deformable_W,
    const GeometryId rigid_id, const TriangleSurfaceMesh<double>& rigid_mesh_R,
    const Bvh<Obb, TriangleSurfaceMesh<double>>& rigid_bvh_R,
    const math::RigidTransform<double>& X_WR,
    std::vector<int>* tetrahedron_index_of_polygons,
    std::vector<VolumeMesh<double>::Barycentric<double>>*
        barycentric_centroids) {
  DRAKE_DEMAND(tetrahedron_index_of_polygons != nullptr);
  DRAKE_DEMAND(barycentric_centroids != nullptr);
  tetrahedron_index_of_polygons->clear();
  barycentric_centroids->clear();

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

  DeformableSurfaceVolumeIntersector intersect;
  intersect.SampleVolumeFieldOnSurface(
      field_W, deformable_W.deformable_mesh().bvh(),
      rigid_mesh_R, rigid_bvh_R, X_WR);

  if (!intersect.has_intersection()) {
    return {};
  }

  *tetrahedron_index_of_polygons = intersect.tetrahedron_index_of_polygons();
  *barycentric_centroids = intersect.barycentric_centroids();
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
