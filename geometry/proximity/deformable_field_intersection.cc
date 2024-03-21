#include "drake/geometry/proximity/deformable_field_intersection.h"

#include <unordered_set>
#include <utility>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/aabb.h"
#include "drake/geometry/proximity/bvh.h"
#include "drake/geometry/proximity/contact_surface_utility.h"
#include "drake/geometry/proximity/field_intersection.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::Vector4i;

namespace {

// Given a contact point (centroid of a contact polygon) in a tetrahedron of
// a mesh, adds contact data from that tetrahedron. The data includes the
// barycentric coordinates of the contact point in that tetrahedron and
// the indices of vertices of that tetrahedron.
void AddDeformableContactDataFromOneTetrahedron(
    const int tetrahedron_index, const Vector3d& contact_point_W,
    const VolumeMesh<double>& tetrahedral_mesh_W,
    std::vector<Vector4d>* barycentric_coordinates,
    std::vector<Vector4i>* contact_vertex_indices,
    std::unordered_set<int>* participating_vertices) {
  barycentric_coordinates->emplace_back(
      tetrahedral_mesh_W.CalcBarycentric(contact_point_W, tetrahedron_index));
  Vector4i vertex_indices;
  for (int i = 0; i < VolumeMesh<double>::kVertexPerElement; ++i) {
    vertex_indices(i) = tetrahedral_mesh_W.element(tetrahedron_index).vertex(i);
    participating_vertices->insert(vertex_indices(i));
  }
  contact_vertex_indices->emplace_back(vertex_indices);
}

}  // namespace

void AddDeformableDeformableContactSurface(
    const VolumeMeshFieldLinear<double, double>& deformable0_sdf_W,
    const DeformableVolumeMeshWithBvh<double>& deformable0_mesh_W,
    GeometryId deformable0_id,
    const VolumeMeshFieldLinear<double, double>& deformable1_sdf_W,
    const DeformableVolumeMeshWithBvh<double>& deformable1_mesh_W,
    GeometryId deformable1_id, DeformableContact<double>* deformable_contact) {
  DRAKE_DEMAND(deformable_contact != nullptr);
  DRAKE_DEMAND(deformable1_id < deformable0_id);

  std::unique_ptr<PolygonSurfaceMesh<double>> contact_mesh_W;
  std::unique_ptr<PolygonSurfaceMeshFieldLinear<double, double>>
      contact_field_W;
  VolumeIntersector<PolyMeshBuilder<double>, Aabb> intersector;
  intersector.IntersectFields(deformable0_sdf_W, deformable0_mesh_W.bvh(),
                              deformable1_sdf_W, deformable1_mesh_W.bvh(),
                              math::RigidTransform<double>::Identity(),
                              &contact_mesh_W, &contact_field_W);

  if (contact_mesh_W == nullptr) return;

  const int num_faces = contact_mesh_W->num_faces();

  std::vector<double> signed_distances_at_contact_points;
  // Contact points are at centroids of contact polygons (not vertices of
  // the polygons).
  signed_distances_at_contact_points.reserve(num_faces);
  for (int f = 0; f < num_faces; ++f) {
    signed_distances_at_contact_points.push_back(
        // Fast evaluation because the gradient of the field has been computed.
        contact_field_W->EvaluateCartesian(
            f, contact_mesh_W->element_centroid(f)));
  }
  std::vector<Vector4d> barycentric_coordinates[2];
  std::vector<Vector4i> contact_vertex_indices[2];
  std::unordered_set<int> participating_vertices[2];
  for (const int geometry : {0, 1}) {
    barycentric_coordinates[geometry].reserve(num_faces);
    contact_vertex_indices[geometry].reserve(num_faces);
    participating_vertices[geometry].reserve(4 * num_faces);
  }
  for (int f = 0; f < num_faces; ++f) {
    const Vector3d contact_point_W = contact_mesh_W->element_centroid(f);
    AddDeformableContactDataFromOneTetrahedron(
        intersector.tet0_of_polygon(f), contact_point_W,
        deformable0_sdf_W.mesh(), &barycentric_coordinates[0],
        &contact_vertex_indices[0], &participating_vertices[0]);
    AddDeformableContactDataFromOneTetrahedron(
        intersector.tet1_of_polygon(f), contact_point_W,
        deformable1_sdf_W.mesh(), &barycentric_coordinates[1],
        &contact_vertex_indices[1], &participating_vertices[1]);
  }

  // DeformableContact::AddDeformableDeformableContactSurface(id_A, id_B)
  // requires the contact surface with normals pointing:
  //     - *out of* geometry B and
  //     - *into* geometry A.
  // However, VolumeIntersector::IntersectFields(deformable0_sdf_W,
  // deformable1_sdf_W) creates the surface mesh with normals in the direction
  // of:
  //     - increasing signed distance deformable0_sdf_W = going *out of*
  //       deformable0_mesh_W,
  //     - decreasing signed distance deformable1_sdf_W = going *into*
  //       deformable1_mesh_W.
  // As a result, we have to call AddDeformableDeformableContactSurface() in
  // the order of (deformable1_id, deformable0_id).
  //     This is OPPOSITE of hydroelastics because the signed distance for
  // deformables and the pressure field for hydroelastics have their gradients
  // in the opposite direction. The signed distance increases by going out of
  // the geometry, while the pressure field increases by going deeper into
  // the geometry.
  deformable_contact->AddDeformableDeformableContactSurface(
      deformable1_id, deformable0_id, participating_vertices[1],
      participating_vertices[0], std::move(*contact_mesh_W),
      std::move(signed_distances_at_contact_points),
      std::move(contact_vertex_indices[1]),
      std::move(contact_vertex_indices[0]),
      std::move(barycentric_coordinates[1]),
      std::move(barycentric_coordinates[0]));
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
