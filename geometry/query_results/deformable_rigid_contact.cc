#include "drake/geometry/query_results/deformable_rigid_contact.h"

#include <algorithm>
#include <iterator>
#include <set>
#include <utility>

namespace drake {
namespace geometry {
namespace internal {

using multibody::contact_solvers::internal::PartialPermutation;

template <typename T>
DeformableRigidContact<T>::DeformableRigidContact(GeometryId deformable_id,
                                                int num_vertices)
    : deformable_id_(deformable_id), participation_(num_vertices, false) {}

template <typename T>
void DeformableRigidContact<T>::Append(
    GeometryId rigid_id, const std::unordered_set<int>& participating_vertices,
    PolygonSurfaceMesh<T>&& contact_mesh_W, std::vector<T>&& signed_distances,
    std::vector<int>&& tetrahedra_indexes,
    std::vector<Vector4<T>>&& barycentric_coordinates) {
  DRAKE_DEMAND(contact_mesh_W.num_faces() ==
               static_cast<int>(signed_distances.size()));
  DRAKE_DEMAND(contact_mesh_W.num_faces() ==
               static_cast<int>(barycentric_coordinates.size()));
  rigid_ids_.emplace_back(rigid_id);

  for (int v : participating_vertices) {
    DRAKE_DEMAND(0 <= v && v < static_cast<int>(participation_.size()));
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
                           std::make_move_iterator(signed_distances.begin()),
                           std::make_move_iterator(signed_distances.end()));
  tetrahedra_indexes_.insert(
      tetrahedra_indexes_.end(),
      std::make_move_iterator(tetrahedra_indexes.begin()),
      std::make_move_iterator(tetrahedra_indexes.end()));
  barycentric_coordinates_.insert(
      barycentric_coordinates_.end(),
      std::make_move_iterator(barycentric_coordinates.begin()),
      std::make_move_iterator(barycentric_coordinates.end()));
}

template <typename T>
PartialPermutation DeformableRigidContact<T>::CalcVertexPermutation() const {
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
  for (int i = 0; i < static_cast<int>(participation_.size()); ++i) {
    /* The call to permutation.push() only conditionally adds i. */
    permutation.push(i);
  }

  return permutation;
}

template class DeformableRigidContact<double>;

}  // namespace internal
}  // namespace geometry
}  // namespace drake
