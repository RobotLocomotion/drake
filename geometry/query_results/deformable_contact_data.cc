#include "drake/geometry/query_results/deformable_contact_data.h"

#include <algorithm>
#include <set>
#include <utility>

namespace drake {
namespace geometry {
namespace internal {

using multibody::contact_solvers::internal::PartialPermutation;

template <typename T>
DeformableContactData<T>::DeformableContactData(GeometryId deformable_id,
                                                int num_vertices)
    : deformable_id_(deformable_id), participation_(num_vertices, false) {}

template <typename T>
void DeformableContactData<T>::Append(
    GeometryId rigid_id, const std::unordered_set<int>& participating_vertices,
    PolygonSurfaceMesh<T>&& contact_mesh_W,
    std::vector<T>&& penetration_distances,
    std::vector<Vector4<T>>&& barycentric_centroids) {
  DRAKE_DEMAND(contact_mesh_W.num_faces() ==
               static_cast<int>(penetration_distances.size()));
  DRAKE_DEMAND(contact_mesh_W.num_faces() ==
               static_cast<int>(barycentric_centroids.size()));
  rigid_ids_.emplace_back(rigid_id);

  for (int v : participating_vertices) {
    if (!participation_[v]) {
      ++num_vertices_in_contact_;
      participation_[v] = true;
    }
  }

  for (int i = 0; i < contact_mesh_W.num_faces(); ++i) {
    /* The normal of the face lies in the direction of the contact frame's
     z-axis. */
    const Vector3<T>& Cz_W = contact_mesh_W.face_normal(i);
    constexpr int kZAxis = 2;
    auto R_WC = math::RotationMatrix<T>::MakeFromOneUnitVector(Cz_W, kZAxis);
    R_CWs_.emplace_back(R_WC.transpose());
  }

  contact_meshes_W_.emplace_back(std::move(contact_mesh_W));
  signed_distances_.insert(signed_distances_.end(),
                           make_move_iterator(penetration_distances.begin()),
                           make_move_iterator(penetration_distances.end()));
  barycentric_coordinates_.insert(
      barycentric_coordinates_.end(),
      make_move_iterator(barycentric_centroids.begin()),
      make_move_iterator(barycentric_centroids.end()));
}

template <typename T>
PartialPermutation DeformableContactData<T>::CalcVertexPermutation() const {
  /* Build the partial permutation. */
  int permuted_vertex_index = -1;  // We'll pre-increment before using.
  std::vector<int> permuted_vertex_indexes(participation_.size(), -1);
  for (int v = 0; v < static_cast<int>(participation_.size()); ++v) {
    if (participation_[v]) {
      permuted_vertex_indexes[v] = ++permuted_vertex_index;
    }
  }
  PartialPermutation permutation(std::move(permuted_vertex_indexes));

  /* Extend the partial permutation to a full permutation. */
  for (int i = 0; i < permutation.domain_size(); ++i) {
    /* The call to permutation.push() only conditionally adds i. */
    permutation.push(i);
  }

  return permutation;
}

template class DeformableContactData<double>;

}  // namespace internal
}  // namespace geometry
}  // namespace drake
