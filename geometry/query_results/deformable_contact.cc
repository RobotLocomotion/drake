#include "drake/geometry/query_results/deformable_contact.h"

#include <utility>

namespace drake {
namespace geometry {
namespace internal {

using multibody::contact_solvers::internal::VertexPartialPermutation;

ContactParticipation::ContactParticipation(int num_vertices)
    : participation_(num_vertices, false) {}

ContactParticipation::~ContactParticipation() = default;

void ContactParticipation::Participate(
    const std::unordered_set<int>& vertices) {
  for (int v : vertices) {
    DRAKE_DEMAND(0 <= v && v < static_cast<int>(participation_.size()));
    if (!participation_[v]) {
      ++num_vertices_in_contact_;
      participation_[v] = true;
    }
  }
}

VertexPartialPermutation ContactParticipation::CalcPartialPermutation() const {
  int permuted_vertex_index = 0;
  std::vector<int> permuted_vertex_indexes(participation_.size(), -1);
  for (int v = 0; v < static_cast<int>(participation_.size()); ++v) {
    if (participation_[v]) {
      permuted_vertex_indexes[v] = permuted_vertex_index++;
    }
  }
  return VertexPartialPermutation(std::move(permuted_vertex_indexes));
}

template <typename T>
DeformableContactSurface<T>::DeformableContactSurface(
    GeometryId id_A, GeometryId id_B, PolygonSurfaceMesh<T> contact_mesh_W,
    std::vector<T> pressures, std::vector<Vector3<T>> pressure_gradients_W,
    std::vector<bool> is_element_inverted,
    std::vector<Vector3<int>> contact_vertex_indexes_A,
    std::vector<Vector3<T>> barycentric_coordinates_A)
    : id_A_(id_A),
      id_B_(id_B),
      contact_mesh_W_(std::move(contact_mesh_W)),
      pressures_(std::move(pressures)),
      pressure_gradients_W_(std::move(pressure_gradients_W)),
      is_element_inverted_(std::move(is_element_inverted)),
      contact_vertex_indexes_A_(std::move(contact_vertex_indexes_A)),
      barycentric_coordinates_A_(std::move(barycentric_coordinates_A)) {
  const int num_contact_points = contact_mesh_W_.num_faces();
  DRAKE_DEMAND(num_contact_points == ssize(*pressures_));
  DRAKE_DEMAND(num_contact_points == ssize(std::get<std::vector<Vector3<T>>>(
                                         barycentric_coordinates_A_)));
  DRAKE_DEMAND(num_contact_points == ssize(std::get<std::vector<Vector3<int>>>(
                                         contact_vertex_indexes_A_)));
  nhats_W_.reserve(num_contact_points);
  contact_points_W_.reserve(num_contact_points);
  R_WCs_.reserve(num_contact_points);
  const int kZAxis = 2;
  for (int i = 0; i < num_contact_points; ++i) {
    /* Ensure that in rigid deformable contact, the normal is pointing from
     rigid to deformable (i.e. from B into A). */
    nhats_W_.emplace_back(-contact_mesh_W_.face_normal(i));
    contact_points_W_.emplace_back(contact_mesh_W_.element_centroid(i));
    R_WCs_.emplace_back(
        math::RotationMatrix<T>::MakeFromOneUnitVector(-nhats_W_[i], kZAxis));
  }
}

template <typename T>
DeformableContactSurface<T>::DeformableContactSurface(
    GeometryId id_A, GeometryId id_B, PolygonSurfaceMesh<T> contact_mesh_W,
    std::vector<T> signed_distances,
    std::vector<Vector4<int>> contact_vertex_indexes_A,
    std::vector<Vector4<T>> barycentric_coordinates_A,
    std::vector<Vector4<int>> contact_vertex_indexes_B,
    std::vector<Vector4<T>> barycentric_coordinates_B)
    : id_A_(id_A),
      id_B_(id_B),
      contact_mesh_W_(std::move(contact_mesh_W)),
      signed_distances_(std::move(signed_distances)),
      contact_vertex_indexes_A_(std::move(contact_vertex_indexes_A)),
      barycentric_coordinates_A_(std::move(barycentric_coordinates_A)),
      contact_vertex_indexes_B_(std::move(contact_vertex_indexes_B)),
      barycentric_coordinates_B_(std::move(barycentric_coordinates_B)) {
  const int num_contact_points = contact_mesh_W_.num_faces();
  DRAKE_DEMAND(num_contact_points == ssize(*signed_distances_));
  DRAKE_DEMAND(num_contact_points == ssize(std::get<std::vector<Vector4<int>>>(
                                         contact_vertex_indexes_A_)));
  DRAKE_DEMAND(num_contact_points == ssize(std::get<std::vector<Vector4<T>>>(
                                         barycentric_coordinates_A_)));
  DRAKE_DEMAND(num_contact_points == ssize(*barycentric_coordinates_B_));
  DRAKE_DEMAND(num_contact_points == ssize(*contact_vertex_indexes_B_));
  DRAKE_DEMAND(id_A < id_B);
  nhats_W_.reserve(num_contact_points);
  contact_points_W_.reserve(num_contact_points);
  R_WCs_.reserve(num_contact_points);
  const int kZAxis = 2;
  for (int i = 0; i < num_contact_points; ++i) {
    nhats_W_.emplace_back(contact_mesh_W_.face_normal(i));
    contact_points_W_.emplace_back(contact_mesh_W_.element_centroid(i));
    R_WCs_.emplace_back(
        math::RotationMatrix<T>::MakeFromOneUnitVector(-nhats_W_[i], kZAxis));
  }
}

template <typename T>
DeformableContactSurface<T>::~DeformableContactSurface() = default;

template <typename T>
DeformableContact<T>::~DeformableContact() = default;

template <typename T>
void DeformableContact<T>::AddDeformableRigidContactSurface(
    GeometryId deformable_id, GeometryId rigid_id,
    const std::unordered_set<int>& participating_vertices,
    PolygonSurfaceMesh<T> contact_mesh_W, std::vector<T> pressures,
    std::vector<Vector3<T>> pressure_gradients_W,
    std::vector<bool> is_element_inverted,
    std::vector<Vector3<int>> contact_vertex_indexes,
    std::vector<Vector3<T>> barycentric_coordinates) {
  const auto iter = contact_participations_.find(deformable_id);
  DRAKE_THROW_UNLESS(iter != contact_participations_.end());
  DRAKE_DEMAND(static_cast<int>(pressures.size()) ==
               contact_mesh_W.num_faces());
  DRAKE_DEMAND(static_cast<int>(pressure_gradients_W.size()) ==
               contact_mesh_W.num_faces());
  DRAKE_DEMAND(static_cast<int>(contact_vertex_indexes.size()) ==
               contact_mesh_W.num_faces());
  DRAKE_DEMAND(static_cast<int>(barycentric_coordinates.size()) ==
               contact_mesh_W.num_faces());
  iter->second.Participate(participating_vertices);
  contact_surfaces_.emplace_back(
      deformable_id, rigid_id, std::move(contact_mesh_W), std::move(pressures),
      std::move(pressure_gradients_W), std::move(is_element_inverted),
      std::move(contact_vertex_indexes), std::move(barycentric_coordinates));
}

template <typename T>
void DeformableContact<T>::Participate(
    GeometryId id, const std::unordered_set<int>& vertices) {
  DRAKE_THROW_UNLESS(IsRegistered(id));
  auto it = contact_participations_.find(id);
  it->second.Participate(vertices);
}

template <typename T>
void DeformableContact<T>::AddDeformableDeformableContactSurface(
    GeometryId id0, GeometryId id1,
    const std::unordered_set<int>& participating_vertices0,
    const std::unordered_set<int>& participating_vertices1,
    PolygonSurfaceMesh<T> contact_mesh_W, std::vector<T> signed_distances,
    std::vector<Vector4<int>> contact_vertex_indices0,
    std::vector<Vector4<int>> contact_vertex_indices1,
    std::vector<Vector4<T>> barycentric_coordinates0,
    std::vector<Vector4<T>> barycentric_coordinates1) {
  DRAKE_THROW_UNLESS(IsRegistered(id0));
  DRAKE_THROW_UNLESS(IsRegistered(id1));
  DRAKE_DEMAND(ssize(signed_distances) == contact_mesh_W.num_faces());
  DRAKE_DEMAND(ssize(contact_vertex_indices0) == contact_mesh_W.num_faces());
  DRAKE_DEMAND(ssize(contact_vertex_indices1) == contact_mesh_W.num_faces());
  DRAKE_DEMAND(ssize(barycentric_coordinates0) == contact_mesh_W.num_faces());
  DRAKE_DEMAND(ssize(barycentric_coordinates1) == contact_mesh_W.num_faces());
  contact_participations_.at(id0).Participate(participating_vertices0);
  contact_participations_.at(id1).Participate(participating_vertices1);
  contact_surfaces_.emplace_back(
      id0, id1, std::move(contact_mesh_W), std::move(signed_distances),
      std::move(contact_vertex_indices0), std::move(barycentric_coordinates0),
      std::move(contact_vertex_indices1), std::move(barycentric_coordinates1));
}

template class DeformableContactSurface<double>;
template class DeformableContact<double>;

}  // namespace internal
}  // namespace geometry
}  // namespace drake
