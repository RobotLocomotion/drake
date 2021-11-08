#pragma once

#include <vector>

#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

/* Appends all of the VolumeElements in `new_elements` into `mesh_elements` */
void Append(const std::vector<VolumeElement>& new_elements,
            std::vector<VolumeElement>* mesh_elements);

/* Subdivides a triangular prism into three tetrahedra such that they share the
 diagonal v0,v5 (see ordering below) of the rectangular face v0,v2,v5,v3. The
 other two face diagonals are v0,v4 and v1,v5.

 We assume the input vertex indices are in this ordering:
   1. Three vertices v0,v1,v2 of the "bottom" face are in counterclockwise
      order when look from "above" the prism.
   2. Three vertices v3,v4,v5 of the "top" face match v0,v1,v2, respectively.

                       v3
                      /|
                     / |\
                    /  | \
                   /   |  \
                  /    v0  \
                 /    /\    \
                v4-----------v5
                |   /    \   |
                |  /      \  |
                | /        \ |
                |/          \|
                v1-----------v2
 */
std::vector<VolumeElement> SplitTriangularPrismToTetrahedra(int v0, int v1,
                                                            int v2, int v3,
                                                            int v4, int v5);

/* Subdivide a pyramid into two tetrahedra sharing the diagonal v0,v2.

                       v4
                   ／  /\ ＼
                v0-----------v1
                |   /    \   |
                |  /      \  |
                | /        \ |
                |/          \|
                v3-----------v2
 */
std::vector<VolumeElement> SplitPyramidToTetrahedra(int v0, int v1, int v2,
                                                    int v3, int v4);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
