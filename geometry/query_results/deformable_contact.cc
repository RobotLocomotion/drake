#include "drake/geometry/query_results/deformable_contact.h"

#include <utility>

#include "drake/common/ssize.h"

namespace drake {
namespace geometry {
namespace internal {

using multibody::contact_solvers::internal::PartialPermutation;

namespace {

// TODO(xuchenhan-tri): Consider moving this function to a header file for full
// unit testing.
/* Extends a partial permutation to a full permutation.
 @pre permutation != nullptr. */
void ExtendToFullPermutation(PartialPermutation* permutation) {
  for (int i = 0; i < static_cast<int>(permutation->domain_size()); ++i) {
    /* The call to permutation.push() only conditionally adds i. */
    permutation->push(i);
  }
}

}  // namespace

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

PartialPermutation ContactParticipation::CalcVertexPermutation() const {
  /* Build the partial permutation. */
  PartialPermutation permutation = CalcVertexPartialPermutation();
  ExtendToFullPermutation(&permutation);
  return permutation;
}

PartialPermutation ContactParticipation::CalcVertexPartialPermutation() const {
  int permuted_vertex_index = 0;
  std::vector<int> permuted_vertex_indexes(participation_.size(), -1);
  for (int v = 0; v < static_cast<int>(participation_.size()); ++v) {
    if (participation_[v]) {
      permuted_vertex_indexes[v] = permuted_vertex_index++;
    }
  }
  return PartialPermutation(std::move(permuted_vertex_indexes));
}

PartialPermutation ContactParticipation::CalcDofPermutation() const {
  PartialPermutation permutation = CalcDofPartialPermutation();
  ExtendToFullPermutation(&permutation);
  return permutation;
}

PartialPermutation ContactParticipation::CalcDofPartialPermutation() const {
  /* Build the partial permutation. */
  int permuted_vertex_index = 0;
  std::vector<int> permuted_dof_indexes(3 * participation_.size(), -1);
  for (int v = 0; v < static_cast<int>(participation_.size()); ++v) {
    if (participation_[v]) {
      permuted_dof_indexes[3 * v] = 3 * permuted_vertex_index;
      permuted_dof_indexes[3 * v + 1] = 3 * permuted_vertex_index + 1;
      permuted_dof_indexes[3 * v + 2] = 3 * permuted_vertex_index + 2;
      ++permuted_vertex_index;
    }
  }
  return PartialPermutation(std::move(permuted_dof_indexes));
}

template <typename T>
DeformableContactSurface<T>::DeformableContactSurface(
    GeometryId id_A, GeometryId id_B, PolygonSurfaceMesh<T> contact_mesh_W,
    std::vector<T> signed_distances,
    std::vector<Vector4<int>> contact_vertex_indexes_A,
    std::vector<Vector4<T>> barycentric_coordinates_A,
    std::optional<std::vector<Vector4<int>>> contact_vertex_indexes_B,
    std::optional<std::vector<Vector4<T>>> barycentric_coordinates_B)
    : id_A_(id_A),
      id_B_(id_B),
      contact_mesh_W_(std::move(contact_mesh_W)),
      signed_distances_(std::move(signed_distances)),
      contact_vertex_indexes_A_(std::move(contact_vertex_indexes_A)),
      barycentric_coordinates_A_(std::move(barycentric_coordinates_A)),
      contact_vertex_indexes_B_(std::move(contact_vertex_indexes_B)),
      barycentric_coordinates_B_(std::move(barycentric_coordinates_B)) {
  const int num_contact_points = contact_mesh_W_.num_faces();
  DRAKE_DEMAND(num_contact_points ==
               static_cast<int>(signed_distances_.size()));
  DRAKE_DEMAND(num_contact_points ==
               static_cast<int>(barycentric_coordinates_A_.size()));
  DRAKE_DEMAND(num_contact_points ==
               static_cast<int>(contact_vertex_indexes_A_.size()));
  DRAKE_DEMAND(contact_vertex_indexes_B_.has_value() ==
               barycentric_coordinates_B_.has_value());
  if (contact_vertex_indexes_B_.has_value()) {
    DRAKE_DEMAND(num_contact_points ==
                 static_cast<int>(barycentric_coordinates_B_->size()));
    DRAKE_DEMAND(num_contact_points ==
                 static_cast<int>(contact_vertex_indexes_B_->size()));
    DRAKE_DEMAND(id_A < id_B);
  }
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
    PolygonSurfaceMesh<T> contact_mesh_W, std::vector<T> signed_distances,
    std::vector<Vector4<int>> contact_vertex_indexes,
    std::vector<Vector4<T>> barycentric_coordinates) {
  const auto iter = contact_participations_.find(deformable_id);
  DRAKE_THROW_UNLESS(iter != contact_participations_.end());
  DRAKE_DEMAND(static_cast<int>(signed_distances.size()) ==
               contact_mesh_W.num_faces());
  DRAKE_DEMAND(static_cast<int>(contact_vertex_indexes.size()) ==
               contact_mesh_W.num_faces());
  DRAKE_DEMAND(static_cast<int>(barycentric_coordinates.size()) ==
               contact_mesh_W.num_faces());
  iter->second.Participate(participating_vertices);
  contact_surfaces_.emplace_back(
      deformable_id, rigid_id, std::move(contact_mesh_W),
      std::move(signed_distances), std::move(contact_vertex_indexes),
      std::move(barycentric_coordinates), std::nullopt, std::nullopt);
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
