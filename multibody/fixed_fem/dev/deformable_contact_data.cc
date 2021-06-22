#include "drake/multibody/fixed_fem/dev/deformable_contact_data.h"

#include <algorithm>
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

  /* Build the permutation for vertices in contact first. */
  num_vertices_in_contact_ = 0;
  for (int i = 0; i < num_contact_pairs(); ++i) {
    const DeformableContactSurface<T>& contact_surface =
        contact_pairs_[i].contact_surface;
    for (int j = 0; j < contact_surface.num_polygons(); ++j) {
      const geometry::VolumeElementIndex tet_in_contact =
          contact_surface.polygon_data(j).tet_index;
      for (int k = 0; k < kNumVerticesInTetrahedron; ++k) {
        const int v = deformable_mesh.element(tet_in_contact).vertex(k);
        if (permuted_vertex_indexes_[v] == -1) {
          permuted_vertex_indexes_[v] = num_vertices_in_contact_;
          permuted_to_original_indexes_[num_vertices_in_contact_] = v;
          ++num_vertices_in_contact_;
        }
      }
    }
  }

  /* Sort the participating vertices in the inverse permutation for stability.
   */
  std::sort(permuted_to_original_indexes_.begin(),
            permuted_to_original_indexes_.begin() + num_vertices_in_contact_);
  /* Fix the original permutation. */
  for (int i = 0; i < num_vertices_in_contact_; ++i) {
    permuted_vertex_indexes_[permuted_to_original_indexes_[i]] = i;
  }

  /* Add the remaining vertices to the permutations. */
  int index = num_vertices_in_contact_;
  for (int i = 0; i < static_cast<int>(permuted_vertex_indexes_.size()); ++i) {
    if (permuted_vertex_indexes_[i] == -1) {
      permuted_vertex_indexes_[i] = index;
      permuted_to_original_indexes_[index] = i;
      ++index;
    }
  }

  DRAKE_DEMAND(index == static_cast<int>(permuted_to_original_indexes_.size()));
}

}  // namespace internal
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fixed_fem::internal::DeformableContactData)
