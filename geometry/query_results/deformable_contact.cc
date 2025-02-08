#include "drake/geometry/query_results/deformable_contact.h"

#include <utility>

#include "drake/common/ssize.h"

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
void PadDataToVector4(const std::vector<Vector3<T>>& data,
                      std::vector<Vector4<T>>* padded_data,
                      const T& dead_data) {
  padded_data->resize(data.size());
  for (int i = 0; i < ssize(data); ++i) {
    for (int j = 0; j < 3; ++j) {
      (*padded_data)[i][j] = data[i][j];
    }
    (*padded_data)[i][3] = dead_data;
  }
}

template <typename T>
DeformableContactSurface<T>::DeformableContactSurface(
    GeometryId id_A, GeometryId id_B, PolygonSurfaceMesh<T> contact_mesh_W,
    std::vector<T> signed_distances,
    std::variant<std::vector<Vector4<int>>, std::vector<Vector3<int>>>
        contact_vertex_indexes_A,
    std::variant<std::vector<Vector4<T>>, std::vector<Vector3<T>>>
        barycentric_coordinates_A,
    std::optional<std::vector<Vector4<int>>> contact_vertex_indexes_B,
    std::optional<std::vector<Vector4<T>>> barycentric_coordinates_B)
    : id_A_(id_A),
      id_B_(id_B),
      contact_mesh_W_(std::move(contact_mesh_W)),
      signed_distances_(std::move(signed_distances)),
      contact_vertex_indexes_B_(std::move(contact_vertex_indexes_B)),
      barycentric_coordinates_B_(std::move(barycentric_coordinates_B)) {
  const int num_contact_points = contact_mesh_W_.num_faces();
  DRAKE_DEMAND(num_contact_points ==
               static_cast<int>(signed_distances_.size()));
  bool is_rigid_vs_deformable = false;
  if (contact_vertex_indexes_A.index() == 0) {
    // Move the data from the variant to the member variable.
    contact_vertex_indexes_A_ =
        std::get<std::vector<Vector4<int>>>(contact_vertex_indexes_A);
  } else {
    is_rigid_vs_deformable = true;
    // Move the data from the variant and pad the data to Vector4<int>.
    const std::vector<Vector3<int>>& contact_vertex_indexes_A_3 =
        std::get<std::vector<Vector3<int>>>(contact_vertex_indexes_A);
    PadDataToVector4<int>(contact_vertex_indexes_A_3,
                          &contact_vertex_indexes_A_, -1);
  }
  if (barycentric_coordinates_A.index() == 0) {
    // Move the data from the variant to the member variable.
    DRAKE_DEMAND(!is_rigid_vs_deformable);
    barycentric_coordinates_A_ =
        std::get<std::vector<Vector4<T>>>(barycentric_coordinates_A);
  } else {
    DRAKE_DEMAND(is_rigid_vs_deformable);
    // Move the data from the variant and pad the data to Vector4<T>.
    const std::vector<Vector3<T>>& barycentric_coordinates_A_3 =
        std::get<std::vector<Vector3<T>>>(barycentric_coordinates_A);
    PadDataToVector4<T>(barycentric_coordinates_A_3,
                        &barycentric_coordinates_A_, T(0.0));
  }
  DRAKE_DEMAND(num_contact_points ==
               static_cast<int>(barycentric_coordinates_A_.size()));
  DRAKE_DEMAND(num_contact_points ==
               static_cast<int>(contact_vertex_indexes_A_.size()));
  DRAKE_DEMAND(contact_vertex_indexes_B_.has_value() ==
               barycentric_coordinates_B_.has_value());
  if (contact_vertex_indexes_B_.has_value()) {
    DRAKE_DEMAND(!is_rigid_vs_deformable);
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
  /* Ensure that in rigid deformable contact, the normal is pointing from rigid
   to deformable (i.e. from B into A). */
  const double sign_flip = is_rigid_vs_deformable ? -1.0 : 1.0;
  for (int i = 0; i < num_contact_points; ++i) {
    nhats_W_.emplace_back(sign_flip * contact_mesh_W_.face_normal(i));
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
    PolygonSurfaceMesh<T> contact_mesh_W, std::vector<T> pressure,
    std::vector<Vector3<int>> contact_vertex_indexes,
    std::vector<Vector3<T>> barycentric_coordinates) {
  const auto iter = contact_participations_.find(deformable_id);
  DRAKE_THROW_UNLESS(iter != contact_participations_.end());
  DRAKE_DEMAND(static_cast<int>(pressure.size()) == contact_mesh_W.num_faces());
  DRAKE_DEMAND(static_cast<int>(contact_vertex_indexes.size()) ==
               contact_mesh_W.num_faces());
  DRAKE_DEMAND(static_cast<int>(barycentric_coordinates.size()) ==
               contact_mesh_W.num_faces());
  iter->second.Participate(participating_vertices);
  contact_surfaces_.emplace_back(
      deformable_id, rigid_id, std::move(contact_mesh_W), std::move(pressure),
      std::move(contact_vertex_indexes), std::move(barycentric_coordinates),
      std::nullopt, std::nullopt);
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
