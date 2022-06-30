#include "drake/geometry/query_results/deformable_contact_data.h"

#include <algorithm>
#include <set>
#include <utility>

namespace drake {
namespace geometry {
namespace internal {

template <typename T>
DeformableContactData<T>::DeformableContactData(
    std::vector<DeformableRigidContactSurface<T>> contact_surfaces,
    const VolumeMesh<double>& deformable_geometry)
    : contact_surfaces_(std::move(contact_surfaces)),
      signed_distances_(contact_surfaces_.size()),
      permuted_vertex_indexes_(deformable_geometry.num_vertices(), -1),
      permuted_to_original_indexes_(deformable_geometry.num_vertices()) {
  num_contact_points_ = 0;
  // TODO(xuchenhan-tri): Consider using a factory function to handle early
  // exit.
  if (contact_surfaces_.empty()) {
    std::iota(std::begin(permuted_vertex_indexes_),
              std::end(permuted_vertex_indexes_), 0);
    std::iota(std::begin(permuted_to_original_indexes_),
              std::end(permuted_to_original_indexes_), 0);
    return;
  }

  /* All contact surfaces should involve the same deformable body. */
  deformable_id_ = contact_surfaces_[0].deformable_id();
  for (const auto& contact_surface : contact_surfaces_) {
    DRAKE_DEMAND(deformable_id_ == contact_surface.deformable_id());
  }

  CalcParticipatingVertices(deformable_geometry);
  for (const auto& contact_surface : contact_surfaces_) {
    num_contact_points_ += contact_surface.num_contact_points();
  }

  /* Build signed_distances_. */
  for (int i = 0; i < static_cast<int>(contact_surfaces_.size()); ++i) {
    std::vector<T>& phi = signed_distances_[i];
    const DeformableRigidContactSurface<T>& contact_surface =
        contact_surfaces_[i];
    phi.resize(contact_surface.num_contact_points(), 0);
    for (int j = 0; j < contact_surface.num_contact_points(); ++j) {
      phi[j] = contact_surface.penetration_distance(j);
    }
  }
}

template <typename T>
void DeformableContactData<T>::CalcParticipatingVertices(
    const geometry::VolumeMesh<double>& deformable_geometry) {
  constexpr int kNumVerticesInTetrahedron =
      geometry::VolumeMesh<T>::kVertexPerElement;

  /* Build the permutation for vertices in contact first. */
  num_vertices_in_contact_ = 0;
  for (int i = 0; i < num_contact_surfaces(); ++i) {
    for (const int tet_in_contact :
         contact_surfaces_[i].tetrahedron_indices()) {
      DRAKE_DEMAND(0 <= tet_in_contact &&
                   tet_in_contact < deformable_geometry.num_elements());
      for (int k = 0; k < kNumVerticesInTetrahedron; ++k) {
        const int v = deformable_geometry.element(tet_in_contact).vertex(k);
        if (permuted_vertex_indexes_[v] == -1) {
          permuted_vertex_indexes_[v] = num_vertices_in_contact_;
          permuted_to_original_indexes_[num_vertices_in_contact_] = v;
          ++num_vertices_in_contact_;
        }
      }
    }
  }

  /* Sort the participating vertices in the inverse permutation for stability.

   Before the sorting below, for the example in the documentation of
   permuted_vertex_indexes(), possibly we have the content of
   permuted_to_original_indexes_ like this:

   |   index i   |   permuted_to_original_indexes_[i]   |   Participating   |
   |             |                                      |   in contact      |
   | :---------: | :----------------------------------: | :---------------: |
   |      0      |                   5                  |       yes         |
   |      1      |                   1                  |       yes         |
   |      2      |                   2                  |       yes         |

   The entries {5,1,2} could be in any order depending on how each mesh
   element lists its vertices. For example, it could be {5,1,2}, {1,2,5}, or
   {2,5,1}, etc.

   We sort the entries in permuted_to_original_indexes_, so that it does not
   depend on the order of vertices listed in each tetrahedron. For the above
   example, the sorted entries would be:

   |   index i   |   permuted_to_original_indexes_[i]   |   Participating   |
   |             |                                      |   in contact      |
   | :---------: | :----------------------------------: | :---------------: |
   |      0      |                   1                  |       yes         |
   |      1      |                   2                  |       yes         |
   |      2      |                   5                  |       yes         |
   */
  std::sort(permuted_to_original_indexes_.begin(),
            permuted_to_original_indexes_.begin() + num_vertices_in_contact_);

  /* Fix the original permutation.

   We reset the value of permuted_vertex_indexes_ from the sorted
   permuted_to_original_indexes_. For the above example, the
   permuted_vertex_indexes_ would become:

   |   index i   |   permuted_vertex_indexes_   |   Participating   |
   |             |                              |   in contact      |
   | :---------: | :--------------------------: | :---------------: |
   |      0      |             -1               |       no          |
   |<==   1   ==>|<==           0            ==>|<==    yes      ==>|
   |<==   2   ==>|<==           1            ==>|<==    yes      ==>|
   |      3      |             -1               |       no          |
   |      4      |             -1               |       no          |
   |<==   5   ==>|<==           2            ==>|<==    yes      ==>|
   */
  for (int i = 0; i < num_vertices_in_contact_; ++i) {
    permuted_vertex_indexes_[permuted_to_original_indexes_[i]] = i;
  }

  /* Add the remaining vertices to the permutations.

   For the remaining entries in permuted_vertex_indexes_ with the
   default value -1, we assign the new values by counting incrementally. For
   the above example, it would become:

   |   index i   |   permuted_vertex_indexes_   |   Participating   |
   |             |                              |   in contact      |
   | :---------: | :--------------------------: | :---------------: |
   |<==   0   ==>|<==           3            ==>|<==    no       ==>|
   |      1      |              0               |       yes         |
   |      2      |              1               |       yes         |
   |<==   3   ==>|<==           4            ==>|<==    no       ==>|
   |<==   4   ==>|<==           5            ==>|<==    no       ==>|
   |      5      |              2               |       yes         |

   As we update permuted_vertex_indexes_, we also update
   permuted_to_original_indexes_ accordingly. For the above example, it would
   become:

   |   index i   |   permuted_to_original_indexes_[i]   |   Participating   |
   |             |                                      |   in contact      |
   | :---------: | :----------------------------------: | :---------------: |
   |      0      |                   1                  |       yes         |
   |      1      |                   2                  |       yes         |
   |      2      |                   5                  |       yes         |
   |<==   3   ==>|<==                0               ==>|<==    no       ==>|
   |<==   4   ==>|<==                3               ==>|<==    no       ==>|
   |<==   5   ==>|<==                4               ==>|<==    no       ==>|
   */
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

template class DeformableContactData<double>;

}  // namespace internal
}  // namespace geometry
}  // namespace drake
