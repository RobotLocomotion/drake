#pragma once

#include <array>
#include <map>
#include <vector>

#include "drake/common/reset_on_copy.h"
#include "drake/geometry/proximity/sorted_triplet.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {

using internal::SortedTriplet;

/** %VolumeMeshTopology represents the topology of a tetrahedral volume mesh.
 @tparam T  The underlying scalar type for coordinates, e.g., double or
            AutoDiffXd. Must be a valid Eigen scalar.
 */
template <class T>
class VolumeMeshTopology {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(VolumeMeshTopology);

  explicit VolumeMeshTopology(const VolumeMesh<T>* mesh) : mesh_(mesh) {
    tetrahedra_neighbors_.resize(mesh_->num_elements(),
                                 std::array<int, 4>{-1, -1, -1, -1});

    // Get the cannonical representation of face f of element e.
    auto get_face = [](const VolumeElement& e, int f) {
      int a = e.vertex((f + 1) % 4);
      int b = e.vertex((f + 2) % 4);
      int c = e.vertex((f + 3) % 4);

      return SortedTriplet<int>(a, b, c);
    };

    // Maps faces to the first tet we encountered containing that tet.
    std::map<SortedTriplet<int>, int> face_to_tet;

    // Establish neighbors for all tets.
    for (int tet_index = 0; tet_index < mesh_->num_elements(); ++tet_index) {
      const VolumeElement& e = mesh_->element(tet_index);

      for (int face_index = 0; face_index < 4; ++face_index) {
        SortedTriplet<int> face = get_face(e, face_index);

        // If we've seen this face before, we now know the two tets that
        // neighbor on `face`.
        if (face_to_tet.contains(face)) {
          int neighbor_tet_index = face_to_tet.at(face);

          // Set the neighbor of the current tet at `face`'s index to the
          // neighbor tet index.
          tetrahedra_neighbors_[tet_index][face_index] = neighbor_tet_index;

          // Find the index of `face` in the neighbor tet.
          int matching_face_index = -1;
          for (int neighbor_face_index = 0; neighbor_face_index < 4;
               ++neighbor_face_index) {
            if (get_face(mesh_->element(neighbor_tet_index),
                         neighbor_face_index) == face) {
              matching_face_index = neighbor_face_index;
              break;
            }
          }
          DRAKE_ASSERT(matching_face_index != -1);
          // Set the neighbor of the neighboring tet at `face`'s matching index
          // to the current tet index.
          tetrahedra_neighbors_[neighbor_tet_index][matching_face_index] =
              tet_index;

        } else {
          // We haven't seen `face` before, so just map it to the current tet.
          face_to_tet[face] = tet_index;
        }
      }
    }
  }

  /**
   Returns a const reference to the underlying mesh.
   @pre mesh_ != nullptr
   */
  const VolumeMesh<T>& mesh() const {
    DRAKE_DEMAND(mesh_ != nullptr);
    return *mesh_;
  }

  /**
   Returns the index of `i`-th neighbor of tet `e` (i.e. the tet across from
   vertex `i`).
   @param e The index of the element.
   @param i The index of the neighbor
   @pre `e ∈ [0, mesh().num_elements())`.
   @pre `i ∈ [0, 3]`.
  */
  int neighbor(int e, int i) const {
    DRAKE_DEMAND(0 <= e && e < mesh_->num_elements());
    DRAKE_DEMAND(0 <= i && i < 4);
    return tetrahedra_neighbors_[e][i];
  }

 private:
  // We use `reset_on_copy` so that the default copy constructor resets
  // the pointer to null when a VolumeMeshTopology is copied.
  reset_on_copy<const VolumeMesh<T>*> mesh_;

  // Stores the index of the neighboring tetrahedra of the element at index i.
  // The index stored at index j is the neighbor across for vertex j, or in
  // other terms the tet that shares face {0, 1, 2, 3} / {i}. -1 is used to
  // represent the absence of a neighbor on a face (i.e. a boundary face).
  std::vector<std::array<int, 4>> tetrahedra_neighbors_;
};

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class VolumeMeshTopology);

}  // namespace geometry
}  // namespace drake
