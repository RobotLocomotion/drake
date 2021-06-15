#include "drake/multibody/fixed_fem/dev/deformable_contact_data.h"

#include <set>
#include <utility>

namespace drake {
namespace multibody {
namespace fixed_fem {
namespace internal {

template <typename T>
DeformableContactData<T>::DeformableContactData(
    std::vector<DeformableRigidContactPair<T>> contact_pairs,
    const geometry::VolumeMesh<T>& deformable_mesh)
    : contact_pairs_(std::move(contact_pairs)),
      permuted_vertex_indexes_(deformable_mesh.num_vertices(), -1),
      permuted_to_original_indexes_(deformable_mesh.num_vertices()) {
  num_contact_points_ = 0;
  if (!contact_pairs_.empty()) {
    CalcParticipatingVertices(deformable_mesh);
    for (const auto& contact_pair : contact_pairs_) {
      num_contact_points_ += contact_pair.num_contact_points();
    }
  } else {
    std::iota(std::begin(permuted_vertex_indexes_),
              std::end(permuted_vertex_indexes_), 0);
    std::iota(std::begin(permuted_to_original_indexes_),
              std::end(permuted_to_original_indexes_), 0);
  }
}

template <typename T>
void DeformableContactData<T>::CalcParticipatingVertices(
    const geometry::VolumeMesh<T>& deformable_mesh) {
  constexpr int kNumVerticesInTetrahedron =
      geometry::VolumeMesh<T>::kVertexPerElement;
  /* Accumulate indexes of all vertices participating in contact into an
   ordered set. This order leads to the stability of the permutation (see the
   second property in permuted_vertex_indexes()). */
  std::set<int> participating_vertices;
  for (int i = 0; i < num_contact_pairs(); ++i) {
    const DeformableContactSurface<T>& contact_surface =
        contact_pairs_[i].contact_surface;
    for (int j = 0; j < contact_surface.num_polygons(); ++j) {
      geometry::VolumeElementIndex tet_in_contact =
          contact_surface.polygon_data(j).tet_index;
      for (int k = 0; k < kNumVerticesInTetrahedron; ++k) {
        participating_vertices.insert(
            deformable_mesh.element(tet_in_contact).vertex(k));
      }
    }
  }
  num_vertices_in_contact_ = participating_vertices.size();

  /* Builds the permuted_vertex_indexes_. All entries are already initialized
   to -1 in the constructor. */
  int new_index = 0;
  /* Vertices participating in contact. */
  for (int old_index : participating_vertices) {
    permuted_vertex_indexes_[old_index] = new_index++;
  }
  /* Vertices not participating in contact. */
  for (int i = 0; i < static_cast<int>(permuted_vertex_indexes_.size()); ++i) {
    if (permuted_vertex_indexes_[i] == -1) {
      permuted_vertex_indexes_[i] = new_index++;
    }
  }
  /* Sanity check that the old and new indexes go up to the same number. */
  DRAKE_DEMAND(new_index == static_cast<int>(permuted_vertex_indexes_.size()));

  /* Build the inverse map. */
  for (int i = 0; i < static_cast<int>(permuted_vertex_indexes_.size()); ++i) {
    permuted_to_original_indexes_[permuted_vertex_indexes_[i]] = i;
  }
}

}  // namespace internal
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fixed_fem::internal::DeformableContactData)
