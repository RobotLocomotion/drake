#pragma once

#include <array>
#include <vector>

#include "drake/geometry/proximity/sorted_triplet.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

/* %VolumeMeshTopology represents the topology of a tetrahedral volume mesh.
 */
class VolumeMeshTopology {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(VolumeMeshTopology);

  template <typename T>
  explicit VolumeMeshTopology(const VolumeMesh<T>& mesh) {
    Initialize(mesh.tetrahedra());
  }

  ~VolumeMeshTopology();

  /* Returns the index of `i`-th neighbor of tet `e` (i.e. the tet across from
   vertex `i`). If tet `e` does not have a neighbor across from `i` (i.e. face
   `i` is a boundary face), returns -1.
   @param e The index of the element.
   @param i The index of the neighbor
   @pre `e ∈ [0, mesh().num_elements())`.
   @pre `i ∈ [0, 3]`.
   */
  int neighbor(int e, int i) const {
    DRAKE_DEMAND(0 <= e && e < ssize(tetrahedra_neighbors_));
    DRAKE_DEMAND(0 <= i && i < 4);
    return tetrahedra_neighbors_[e][i];
  }

  /* Returns true if `topology` is bit-wise equal to `this` (i.e. member
   containers are exact copies of each other). Note: this method is not intended
   to distinguish whether two `VolumeMeshTopology` are *topologically*
   equivalent to each other (i.e. whether they describe the same adjacency graph
   up to isomorphism).
   */
  bool Equal(const VolumeMeshTopology& topology) const;

 private:
  /* Calculates the adjacency information for all tetrahedra in `elements`. */
  void Initialize(const std::vector<VolumeElement>& elements);

  /* Stores the index of the neighboring tetrahedra of the element at index i.
   The index stored at index j is the neighbor across for vertex j, or in
   other terms the tet that shares face {0, 1, 2, 3} / {i}. -1 is used to
   represent the absence of a neighbor on a face (i.e. a boundary face). */
  std::vector<std::array<int, 4>> tetrahedra_neighbors_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
