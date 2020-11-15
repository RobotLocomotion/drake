#include "drake/geometry/proximity/meshing_utilities.h"

#include <unordered_set>

namespace drake {
namespace geometry {
namespace internal {

using std::unordered_set;

std::vector<VolumeElement> SplitToTetrahedra(
    VolumeVertexIndex v0, VolumeVertexIndex v1, VolumeVertexIndex v2,
    VolumeVertexIndex v3, VolumeVertexIndex v4, VolumeVertexIndex v5,
    VolumeVertexIndex v6, VolumeVertexIndex v7) {
  std::vector<VolumeElement> elements;
  elements.reserve(6);
  VolumeVertexIndex previous = v1;
  for (const VolumeVertexIndex& next : {v2, v3, v7, v4, v5, v1}) {
    // Allow distinct vertex indices only.
    if (unordered_set<VolumeVertexIndex>({previous, next, v0, v6}).size() ==
        4) {
      elements.emplace_back(previous, next, v0, v6);
    }
    previous = next;
  }
  return elements;
}


}  // namespace internal
}  // namespace geometry
}  // namespace drake
