#include "drake/geometry/query_results/deformable_contact_data.h"

#include <algorithm>
#include <set>
#include <utility>

namespace drake {
namespace geometry {
namespace internal {

template <typename T>
DeformableContactData<T>::DeformableContactData(
    std::vector<DeformableRigidContactPair<T>> contact_pairs,
    const ReferenceDeformableGeometry& deformable_geometry)
    : contact_pairs_(std::move(contact_pairs)),
      signed_distances_(contact_pairs_.size()),
      permuted_vertex_indexes_(deformable_geometry.mesh().num_vertices(), -1),
      permuted_to_original_indexes_(deformable_geometry.mesh().num_vertices()) {
  num_contact_points_ = 0;
  if (contact_pairs_.empty()) {
    std::iota(std::begin(permuted_vertex_indexes_),
              std::end(permuted_vertex_indexes_), 0);
    std::iota(std::begin(permuted_to_original_indexes_),
              std::end(permuted_to_original_indexes_), 0);
    return;
  }

  /* All contact pairs should involve the same deformable body. */
  deformable_body_index_ = contact_pairs_[0].deformable_id;
  for (const auto& contact_pair : contact_pairs_) {
    DRAKE_DEMAND(deformable_body_index_ == contact_pair.deformable_id);
  }

  CalcParticipatingVertices(deformable_geometry.mesh());
  for (const auto& contact_pair : contact_pairs_) {
    num_contact_points_ += contact_pair.num_contact_points();
  }

  /* Build signed_distances_. */
  for (int i = 0; i < static_cast<int>(contact_pairs_.size()); ++i) {
    const DeformableContactSurface<T>& contact_surface =
        contact_pairs_[i].contact_surface;
    std::vector<T>& phi = signed_distances_[i];
    phi.resize(contact_surface.num_polygons(), 0);
    for (int j = 0; j < contact_surface.num_polygons(); ++j) {
      const ContactPolygonData<T>& polygon_data =
          contact_surface.polygon_data(j);
      const Vector4<T>& barycentric_coord = polygon_data.b_centroid;
      const auto& tet_index = polygon_data.tet_index;
      phi[j] = deformable_geometry.signed_distance_field().Evaluate(
          tet_index, barycentric_coord);
    }
  }
}

template <typename T>
void DeformableContactData<T>::CalcParticipatingVertices(
    const geometry::VolumeMesh<double>& deformable_mesh) {
  constexpr int kNumVerticesInTetrahedron =
      geometry::VolumeMesh<T>::kVertexPerElement;

  /* Build the permutation for vertices in contact first. */
  num_vertices_in_contact_ = 0;
  for (int i = 0; i < num_contact_pairs(); ++i) {
    const DeformableContactSurface<T>& contact_surface =
        contact_pairs_[i].contact_surface;
    for (int j = 0; j < contact_surface.num_polygons(); ++j) {
      const int tet_in_contact = contact_surface.polygon_data(j).tet_index;
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

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class DeformableContactData)

}  // namespace internal
}  // namespace geometry
}  // namespace drake
