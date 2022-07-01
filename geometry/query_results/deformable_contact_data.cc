#include "drake/geometry/query_results/deformable_contact_data.h"

#include <algorithm>
#include <iostream>
#include <set>
#include <utility>

namespace drake {
namespace geometry {
namespace internal {

using multibody::contact_solvers::internal::PartialPermutation;

namespace {

/* Computes the partial permutation that maps the vertex indexes of
 `mesh` to the permuted vertex indexes as described by
 `DeformableContactData::permuted_vertex_indexes()` as well as the number of
 vertices participating in contact.
 @param[in] participating_tetrahedra
   A vector of indexes of tetrahedra of `deformable_geometry` whose vertices
   "participate in the permutation". See PartialPermutation.
 @param[in] mesh
   The topology of the mesh. */
std::pair<PartialPermutation, int> CalcVertexPermutation(
    std::vector<int> participating_tetrahedra, const VolumeMesh<double>& mesh) {
  constexpr int kNumVerticesInTetrahedron =
      VolumeMesh<double>::kVertexPerElement;

  std::vector<int> permuted_vertex_indexes(mesh.num_vertices(), -1);
  int num_vertices_in_contact = 0;
  /* Build the permutation for vertices in contact. */
  for (const int tet_in_contact : participating_tetrahedra) {
    DRAKE_DEMAND(0 <= tet_in_contact && tet_in_contact < mesh.num_elements());
    std::set<int> sorted_vertex_indexes;
    for (int k = 0; k < kNumVerticesInTetrahedron; ++k) {
      sorted_vertex_indexes.emplace(mesh.element(tet_in_contact).vertex(k));
    }
    for (int v : sorted_vertex_indexes) {
      if (permuted_vertex_indexes[v] == -1) {
        permuted_vertex_indexes[v] = num_vertices_in_contact++;
      }
    }
  }
  PartialPermutation permutation(permuted_vertex_indexes);

  /* Extend the partial permutation to a full permutation. */
  for (int i = 0; i < permutation.domain_size(); ++i) {
    if (!permutation.participates(i)) {
      permutation.push(i);
    }
  }
  return {permutation, num_vertices_in_contact};
}

}  // namespace

template <typename T>
DeformableContactData<T>::DeformableContactData(
    std::vector<std::unique_ptr<DeformableRigidContactSurface<T>>>&&
        contact_surfaces,
    const VolumeMesh<double>& deformable_geometry) {
  // TODO(xuchenhan-tri): Consider using a factory function to handle early
  // exit.
  if (contact_surfaces.empty()) {
    return;
  }

  deformable_id_ = contact_surfaces[0]->deformable_id();
  num_contact_points_ = 0;
  for (const auto& contact_surface : contact_surfaces) {
    /* All contact surfaces should involve the same deformable body. */
    DRAKE_DEMAND(deformable_id_ == contact_surface->deformable_id());
    num_contact_points_ += contact_surface->num_contact_points();
  }

  /* Build per contact surface and per contact point quantities. */
  std::vector<int> participating_tetrahedra;
  participating_tetrahedra.reserve(num_contact_points_);
  signed_distances_.reserve(num_contact_points_);
  barycentric_coordinates_.reserve(num_contact_points_);
  rigid_ids_.reserve(num_contact_surfaces());
  for (int i = 0; i < static_cast<int>(contact_surfaces.size()); ++i) {
    DeformableRigidContactSurface<T>& contact_surface = *contact_surfaces[i];
    rigid_ids_.emplace_back(contact_surface.rigid_id());
    contact_surface_mesh_W_.emplace_back(
        std::move(contact_surface.release_contact_surface_mesh()));
    for (int j = 0; j < contact_surface.num_contact_points(); ++j) {
      participating_tetrahedra.emplace_back(
          contact_surface.tetrahedron_indices()[j]);
      signed_distances_.emplace_back(contact_surface.penetration_distance(j));
      barycentric_coordinates_.emplace_back(
          contact_surface.barycentric_centroids()[j]);
    }
  }

  std::tie(vertex_permutation_, num_vertices_in_contact_) =
      CalcVertexPermutation(std::move(participating_tetrahedra),
                            deformable_geometry);
}

template class DeformableContactData<double>;

}  // namespace internal
}  // namespace geometry
}  // namespace drake
