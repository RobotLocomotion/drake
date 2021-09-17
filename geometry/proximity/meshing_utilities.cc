#include "drake/geometry/proximity/meshing_utilities.h"

#include <unordered_set>

namespace drake {
namespace geometry {
namespace internal {

void Append(const std::vector<VolumeElement>& new_elements,
            std::vector<VolumeElement>* mesh_elements) {
  mesh_elements->insert(mesh_elements->end(), new_elements.begin(),
                        new_elements.end());
}

using std::unordered_set;

std::vector<VolumeElement> SplitTriangularPrismToTetrahedra(int v0, int v1,
                                                            int v2, int v3,
                                                            int v4, int v5) {
  std::vector<VolumeElement> elements;
  elements.reserve(3);
  int previous = v3;
  for (int next : {v4, v1, v2}) {
    elements.emplace_back(previous, next, v0, v5);
    previous = next;
  }
  return elements;
}

std::vector<VolumeElement> SplitPyramidToTetrahedra(int v0, int v1, int v2,
                                                    int v3, int v4) {
  std::vector<VolumeElement> elements;
  elements.reserve(2);
  int previous = v3;
  for (int next : {v4, v1}) {
    elements.emplace_back(previous, next, v0, v2);
    previous = next;
  }
  return elements;
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
