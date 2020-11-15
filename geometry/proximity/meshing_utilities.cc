#include "drake/geometry/proximity/meshing_utilities.h"

#include <unordered_set>

namespace drake {
namespace geometry {
namespace internal {

using std::unordered_set;

std::vector<VolumeElement> SplitToTetrahedra(
    const VolumeVertexIndex v0, const VolumeVertexIndex v1,
    const VolumeVertexIndex v2, const VolumeVertexIndex v3,
    const VolumeVertexIndex v4, const VolumeVertexIndex v5,
    const VolumeVertexIndex v6, const VolumeVertexIndex v7) {
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

std::vector<VolumeElement> SplitTriangularPrismToTetrahedra(
    const VolumeVertexIndex v0, const VolumeVertexIndex v1,
    const VolumeVertexIndex v2, const VolumeVertexIndex v3,
    const VolumeVertexIndex v4, const VolumeVertexIndex v5) {
  std::vector<VolumeElement> elements;
  elements.reserve(3);
  VolumeVertexIndex previous = v3;
  for (const VolumeVertexIndex& next : {v4, v1, v2}) {
      elements.emplace_back(previous, next, v0, v5);
    previous = next;
  }
  return elements;
}

std::vector<VolumeElement> SplitPyramidToTetrahedra(
    const VolumeVertexIndex v0, const VolumeVertexIndex v1,
    const VolumeVertexIndex v2, const VolumeVertexIndex v3,
    const VolumeVertexIndex v4) {
  std::vector<VolumeElement> elements;
  elements.reserve(2);
  VolumeVertexIndex previous = v3;
  for (const VolumeVertexIndex& next : {v4, v1}) {
    elements.emplace_back(previous, next, v0, v2);
    previous = next;
  }
  return elements;
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
