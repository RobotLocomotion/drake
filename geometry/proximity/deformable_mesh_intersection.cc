#include "drake/geometry/proximity/deformable_mesh_intersection.h"

#include <unordered_set>
#include <utility>
#include <vector>

#include "drake/geometry/proximity/contact_surface_utility.h"
#include "drake/geometry/proximity/deformable_volume_mesh.h"
#include "drake/geometry/proximity/mesh_intersection.h"

namespace drake {
namespace geometry {
namespace internal {

// TODO(DamrongGuoy) Declare DeformableSurfaceVolumeIntersector in the header
//  file to test it directly and for future code re-use.

class DeformableSurfaceVolumeIntersector
    : public SurfaceVolumeIntersector<PolyMeshBuilder<double>, Aabb> {
 public:
  /* Returns the indices of tetrahedra containing the contact polygons.
   @pre Call it after SampleVolumeFieldOnSurface() finishes.  */
  std::vector<int>& mutable_tetrahedron_index_of_polygons() {
    return tetrahedron_index_of_polygons_;
  }

  /* Returns barycentric coordinates of the centroids of the contact polygons.
   @pre Call it after SampleVolumeFieldOnSurface() finishes.  */
  std::vector<VolumeMesh<double>::Barycentric<double>>&
  mutable_barycentric_centroids() {
    return barycentric_centroids_;
  }

 protected:
  /* Override the parent class's virtual function to store additional
   data for deformables. */
  void CalcContactPolygon(
      const VolumeMeshFieldLinear<double, double>& volume_field_D,
      const TriangleSurfaceMesh<double>& surface_R,
      const math::RigidTransform<T>& X_DR,
      const math::RigidTransform<double>& X_DR_d,
      PolyMeshBuilder<double>* builder_D,
      bool filter_face_normal_along_field_gradient, int tet_index,
      int tri_index) override {
    const int num_vertices_before = builder_D->num_vertices();
    // N.B. we must invoke the base implementation before recording any new
    // data.
    SurfaceVolumeIntersector<PolyMeshBuilder<double>, Aabb>::CalcContactPolygon(
        volume_field_D, surface_R, X_DR, X_DR_d, builder_D,
        filter_face_normal_along_field_gradient, tet_index, tri_index);
    const int num_vertices_after = builder_D->num_vertices();
    const int num_new_vertices = num_vertices_after - num_vertices_before;
    if (num_new_vertices == 0) {
      return;
    }
    tetrahedron_index_of_polygons_.push_back(tet_index);

    // TODO(xuchenhan-tri): Consider accessing the newly added polygon from
    //  the builder. Here we assume internal knowledge how the function
    //  SurfaceVolumeIntersector::CalcContactPolygon works, i.e., the list of
    //  new vertices form the new polygon in that order.
    std::vector<int> polygon(num_vertices_after - num_vertices_before);
    std::iota(polygon.begin(), polygon.end(), num_vertices_before);

    barycentric_centroids_.push_back(volume_field_D.mesh().CalcBarycentric(
        CalcPolygonCentroid(
            polygon, X_DR_d.rotation() * surface_R.face_normal(tri_index),
            builder_D->vertices()),
        tet_index));
  }

 private:
  std::vector<int> tetrahedron_index_of_polygons_{};
  std::vector<VolumeMesh<double>::Barycentric<double>> barycentric_centroids_{};
};

void AppendDeformableRigidContact(
    const deformable::DeformableGeometry& deformable_D,
    const GeometryId rigid_id, const TriangleSurfaceMesh<double>& rigid_mesh_R,
    const Bvh<Obb, TriangleSurfaceMesh<double>>& rigid_bvh_R,
    const math::RigidTransform<double>& X_DR,
    DeformableRigidContact<double>* deformable_rigid_contact) {
  DRAKE_DEMAND(deformable_rigid_contact != nullptr);

  DeformableSurfaceVolumeIntersector intersect;
  intersect.SampleVolumeFieldOnSurface(
      deformable_D.signed_distance_field(),
      deformable_D.deformable_mesh().bvh(), rigid_mesh_R, rigid_bvh_R, X_DR,
      false /* don't filter face normal along field gradient */);

  if (intersect.has_intersection()) {
    std::unique_ptr<PolygonSurfaceMesh<double>> contact_mesh_W =
        intersect.release_mesh();
    const PolygonSurfaceMeshFieldLinear<double, double>& signed_distance_field =
        intersect.mutable_field();
    const int num_faces = contact_mesh_W->num_faces();
    /* Compute the penetration distance at the centroid of each contact polygon
     using the signed distance field. */
    std::vector<double> penetration_distances(num_faces);
    for (int i = 0; i < num_faces; ++i) {
      /* `signed_distance_field` has a gradient, therefore `EvaluateCartesian()`
       should be cheap. */
      penetration_distances[i] = signed_distance_field.EvaluateCartesian(
          i, contact_mesh_W->element_centroid(i));
    }

    const VolumeMesh<double>& mesh = deformable_D.deformable_mesh().mesh();
    std::vector<int>& participating_tetrahedra =
        intersect.mutable_tetrahedron_index_of_polygons();
    std::unordered_set<int> participating_vertices;
    for (int e : participating_tetrahedra) {
      for (int v = 0; v < VolumeMesh<double>::kVertexPerElement; ++v) {
        participating_vertices.insert(mesh.element(e).vertex(v));
      }
    }

    deformable_rigid_contact->Append(
        rigid_id, participating_vertices, std::move(*contact_mesh_W),
        std::move(penetration_distances), std::move(participating_tetrahedra),
        std::move(intersect.mutable_barycentric_centroids()));
  }
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
