#include "drake/geometry/query_results/deformable_contact_data.h"

#include <algorithm>
#include <set>
#include <utility>

namespace drake {
namespace geometry {
namespace internal {
namespace {

using multibody::contact_solvers::internal::PartialPermutation;

/* Computes the partial permutation that maps the vertex indexes of
 `mesh` to the permuted vertex indexes as described by
 `DeformableContactData::vertex_permutation()` as well as the number of
 vertices participating in contact.
 @param[in] participating_tetrahedra
   A vector of indexes of tetrahedra of `deformable_geometry` whose vertices
   participate in the permutation.
 @param[in] mesh
   The topology of the mesh. */
std::pair<PartialPermutation, int> CalcVertexPermutation(
    const std::vector<int>& participating_tetrahedra,
    const VolumeMesh<double>& mesh) {
  constexpr int kNumVerticesInTetrahedron =
      VolumeMesh<double>::kVertexPerElement;

  std::vector<int> permuted_vertex_indexes(mesh.num_vertices(), -1);
  /* Maintain a set of indexes whose corresponding vertices participate in
   contact. */
  std::set<int> sorted_vertex_indexes;
  for (const int tet_in_contact : participating_tetrahedra) {
    DRAKE_DEMAND(0 <= tet_in_contact && tet_in_contact < mesh.num_elements());
    for (int k = 0; k < kNumVerticesInTetrahedron; ++k) {
      sorted_vertex_indexes.emplace(mesh.element(tet_in_contact).vertex(k));
    }
  }

  /* Build the partial permutation. */
  int permuted_vertex_index = -1;  // We'll pre-increment before using.
  for (int v : sorted_vertex_indexes) {
    permuted_vertex_indexes[v] = ++permuted_vertex_index;
  }
  PartialPermutation permutation(permuted_vertex_indexes);

  /* Extend the partial permutation to a full permutation. */
  for (int i = 0; i < permutation.domain_size(); ++i) {
    /* The call to permutation.push() only conditionally adds i. */
    permutation.push(i);
  }
  return {permutation, sorted_vertex_indexes.size()};
}

}  // namespace

template <typename T>
DeformableContactData<T>::DeformableContactData(
    std::vector<DeformableRigidContactSurface<T>>&& contact_surfaces,
    const VolumeMesh<double>& deformable_geometry) {
  // TODO(xuchenhan-tri): Consider using a factory function to handle early
  // exit.
  if (contact_surfaces.empty()) {
    return;
  }

  deformable_id_ = contact_surfaces[0].deformable_id();
  num_contact_points_ = 0;
  for (const auto& c : contact_surfaces) {
    /* All contact surfaces should involve the same deformable body. */
    DRAKE_DEMAND(deformable_id_ == c.deformable_id());
    num_contact_points_ += c.num_contact_points();
  }

  /* Build per contact surface and per contact point quantities. */
  std::vector<int> participating_tetrahedra;
  participating_tetrahedra.reserve(num_contact_points_);
  signed_distances_.reserve(num_contact_points_);
  barycentric_coordinates_.reserve(num_contact_points_);
  rigid_ids_.reserve(contact_surfaces.size());
  for (auto& c : contact_surfaces) {
    rigid_ids_.emplace_back(c.rigid_id());
    const auto [contact_surface_mesh_W, penetration_distances,
                tetrahedron_indices, barycentric_centroids, R_CWs] =
        c.release_data();
    contact_surface_mesh_W_.emplace_back(std::move(contact_surface_mesh_W));
    signed_distances_.insert(signed_distances_.end(),
                             make_move_iterator(penetration_distances.begin()),
                             make_move_iterator(penetration_distances.end()));
    participating_tetrahedra.insert(
        participating_tetrahedra.end(),
        make_move_iterator(tetrahedron_indices.begin()),
        make_move_iterator(tetrahedron_indices.end()));
    barycentric_coordinates_.insert(
        barycentric_coordinates_.end(),
        make_move_iterator(barycentric_centroids.begin()),
        make_move_iterator(barycentric_centroids.end()));
    R_CWs_.insert(R_CWs_.end(), make_move_iterator(R_CWs.begin()),
                  make_move_iterator(R_CWs.end()));
  }

  std::tie(vertex_permutation_, num_vertices_in_contact_) =
      CalcVertexPermutation(std::move(participating_tetrahedra),
                            deformable_geometry);
}

template class DeformableContactData<double>;

}  // namespace internal
}  // namespace geometry
}  // namespace drake
