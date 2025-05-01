#include "drake/geometry/proximity/deformable_mesh_intersection.h"

#include <unordered_set>
#include <utility>
#include <vector>

#include "drake/geometry/deformable_mesh_with_bvh.h"
#include "drake/geometry/proximity/contact_surface_utility.h"
#include "drake/geometry/proximity/mesh_intersection.h"

namespace drake {
namespace geometry {
namespace internal {

// TODO(DamrongGuoy) Declare DeformableSurfaceVolumeIntersector in the header
//  file to test it directly and for future code re-use.

class DeformableSurfaceVolumeIntersector
    : public SurfaceVolumeIntersector<PolyMeshBuilder<double>, Obb, Aabb> {
 public:
  // N.B. If this class declaration moves to the header, don't inline this dtor.
  ~DeformableSurfaceVolumeIntersector() = default;

  /* Returns the indices of triangles containing the contact polygons.
   @pre Call it after SampleVolumeFieldOnSurface() finishes.  */
  std::vector<int>& mutable_triangle_index_of_polygons() {
    return triangle_index_of_polygons_;
  }

  /* Returns barycentric coordinates of the centroids of the contact polygons.
   @pre Call it after SampleVolumeFieldOnSurface() finishes.  */
  std::vector<TriangleSurfaceMesh<double>::Barycentric<double>>&
  mutable_barycentric_centroids() {
    return barycentric_centroids_;
  }

 protected:
  /* Override the parent class's virtual function to store additional
   data for deformables. */
  void CalcContactPolygon(
      const VolumeMeshFieldLinear<double, double>& volume_field_R,
      const TriangleSurfaceMesh<double>& surface_D,
      const math::RigidTransform<T>& X_RD,
      const math::RigidTransform<double>& X_RD_d,
      PolyMeshBuilder<double>* builder_R,
      bool filter_face_normal_along_field_gradient, int tet_index,
      int tri_index) override {
    const int num_vertices_before = builder_R->num_vertices();
    // N.B. we must invoke the base implementation before recording any new
    // data.
    SurfaceVolumeIntersector<PolyMeshBuilder<double>, Obb, Aabb>::
        CalcContactPolygon(volume_field_R, surface_D, X_RD, X_RD_d, builder_R,
                           filter_face_normal_along_field_gradient, tet_index,
                           tri_index);
    const int num_vertices_after = builder_R->num_vertices();
    const int num_new_vertices = num_vertices_after - num_vertices_before;
    if (num_new_vertices == 0) {
      return;
    }
    triangle_index_of_polygons_.push_back(tri_index);

    // TODO(xuchenhan-tri): Consider accessing the newly added polygon from
    //  the builder. Here we assume internal knowledge how the function
    //  SurfaceVolumeIntersector::CalcContactPolygon works, i.e., the list of
    //  new vertices form the new polygon in that order.
    std::vector<int> polygon(num_new_vertices);
    std::iota(polygon.begin(), polygon.end(), num_vertices_before);

    const Vector3<double> n_R =
        X_RD_d.rotation() * -surface_D.face_normal(tri_index);
    const Vector3<double> p_RC =
        CalcPolygonCentroid(polygon, n_R, builder_R->vertices());
    const auto X_DR_d = X_RD_d.inverse();
    // Our convention is that the deformable body quantities are always measured
    // and expressed in the world frame.
    const Vector3<double> p_WC_W = X_DR_d * p_RC;
    const Vector3<double> barycentric_centroid =
        surface_D.CalcBarycentric(p_WC_W, tri_index);
    barycentric_centroids_.push_back(barycentric_centroid);
  }

 private:
  std::vector<int> triangle_index_of_polygons_{};
  std::vector<TriangleSurfaceMesh<double>::Barycentric<double>>
      barycentric_centroids_{};
};

void AddDeformableRigidContactSurface(
    const DeformableSurfaceMeshWithBvh<double>& deformable_mesh_D,
    const DeformableVolumeMeshWithBvh<double>& deformable_volume_mesh_D,
    const std::vector<int>& surface_index_to_volume_index,
    const std::vector<int>& surface_tri_to_volume_tet,
    const GeometryId deformable_id, const GeometryId rigid_id,
    const VolumeMeshFieldLinear<double, double>& pressure_field_R,
    const Bvh<Obb, VolumeMesh<double>>& rigid_bvh_R,
    const math::RigidTransform<double>& X_RD,
    DeformableContact<double>* deformable_contact) {
  DRAKE_DEMAND(deformable_contact != nullptr);
  DeformableSurfaceVolumeIntersector intersect;
  intersect.SampleVolumeFieldOnSurface(
      pressure_field_R, rigid_bvh_R, deformable_mesh_D.mesh(),
      deformable_mesh_D.bvh(), X_RD,
      true /* Filter face normal along field gradient */);

  if (intersect.has_intersection()) {
    std::unique_ptr<PolygonSurfaceMesh<double>> contact_mesh_R =
        intersect.release_mesh();
    const PolygonSurfaceMeshFieldLinear<double, double>&
        surface_pressure_field_R = intersect.mutable_field();
    const int num_faces = contact_mesh_R->num_faces();
    /* Compute the pressure values at the centroid of each contact polygon. */
    std::vector<double> pressures(num_faces);
    for (int i = 0; i < num_faces; ++i) {
      const Vector3<double>& contact_points_R =
          contact_mesh_R->element_centroid(i);
      /* `surface_pressure_field_R` has a gradient, therefore
       `EvaluateCartesian()` should be cheap. */
      pressures[i] =
          surface_pressure_field_R.EvaluateCartesian(i, contact_points_R);
    }

    // Transform the contact mesh from the rigid frame R to the deformable frame
    // (which is aligned with the world frame).
    const math::RigidTransform<double> X_WR = X_RD.inverse();
    contact_mesh_R->TransformVertices(X_WR);
    std::unique_ptr<PolygonSurfaceMesh<double>> contact_mesh_W =
        std::move(contact_mesh_R);

    const TriangleSurfaceMesh<double>& mesh = deformable_mesh_D.mesh();
    const VolumeMesh<double>& volume_mesh = deformable_volume_mesh_D.mesh();
    // Each contact polygon generates one "participating triangle". Hence
    // `participating_triangles` contains duplicated entries when a triangle
    // covers multiple contact polygons.
    const std::vector<int>& participating_triangles =
        intersect.mutable_triangle_index_of_polygons();

    std::vector<bool> is_element_inverted(num_faces, false);
    for (int i = 0; i < num_faces; ++i) {
      const int e = participating_triangles[i];
      const int tet_index = surface_tri_to_volume_tet[e];
      if (volume_mesh.CalcTetrahedronVolume(tet_index) < 0) {
        is_element_inverted[i] = true;
      }
    }
    DRAKE_DEMAND(static_cast<int>(participating_triangles.size()) == num_faces);

    // The set of all indices of the tet vertices of the deformable body that
    // are participating in contact.
    std::unordered_set<int> participating_tri_vertices;
    // contact_tet_vertex_indices[i] stores the _tet_ indices of the vertices
    // incident to the i-th triangle in contact.
    std::vector<Vector3<int>> contact_tet_vertex_indices;
    // Each contact point generates 3 participating vertices. We overestimate by
    // ignoring potential duplications.
    participating_tri_vertices.reserve(3 * num_faces);
    contact_tet_vertex_indices.reserve(num_faces);
    const std::vector<Vector3<double>>& pressure_gradient_R =
        intersect.mutable_grad_eM_M();
    DRAKE_DEMAND(ssize(pressure_gradient_R) == num_faces);
    std::vector<Vector3<double>> pressure_gradient_W(num_faces);
    for (int e : participating_triangles) {
      Vector3<int> triangle_vertex_tet_indices;
      for (int v = 0; v < TriangleSurfaceMesh<double>::kVertexPerElement; ++v) {
        // Map from the surface mesh's vertex index to the volume mesh's vertex
        const int index =
            surface_index_to_volume_index[mesh.element(e).vertex(v)];
        triangle_vertex_tet_indices(v) = index;
        participating_tri_vertices.insert(index);
      }
      contact_tet_vertex_indices.push_back(triangle_vertex_tet_indices);
    }
    for (int e = 0; e < num_faces; ++e) {
      pressure_gradient_W[e] = X_WR * pressure_gradient_R[e];
    }

    deformable_contact->AddDeformableRigidContactSurface(
        deformable_id, rigid_id, participating_tri_vertices,
        std::move(*contact_mesh_W), std::move(pressures),
        std::move(pressure_gradient_W), std::move(is_element_inverted),
        std::move(contact_tet_vertex_indices),
        std::move(intersect.mutable_barycentric_centroids()));
  }
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
