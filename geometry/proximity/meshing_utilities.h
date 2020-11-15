#pragma once

#include <vector>

#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

/* Subdivides a virtual cube to have up to six tetrahedra such that they
 share the diagonal v0v6 (see ordering below). We allow repeated vertices in
 the virtual cube and will skip tetrahedra with repeated vertices.

 We assume the input vertex indices (represented as VolumeVertexIndex) are in
 this ordering:
   1. Four vertices v0,v1,v2,v3 of the "bottom" face in counterclockwise
      order when look from "above" the cube.
   2. Four vertices v4,v5,v6,v7 of the "top" face matching v0,v1,v2,v3,
      respectively.

                       v4-----------v7
                      /|           /|
                     / |          / |
                    /  |         /  |
                   /   |        /   |
                  /    v0------/----v3
                 /    /       /    /
                v5-----------v6   /
                |   /        |   /
                |  /         |  /
                | /          | /
                |/           |/
                v1-----------v2

 Examples of cases with repeated vertices.
 - If v0 == v3 and v1 == v2, we will have a prism instead of a cube and will
   get 3 tetrahedra instead of 6 tetrahedra.
 - If v0 == v1 == v2 == v3, we will have a pyramid instead of a cube and will
   get 2 tetrahedra instead of 6 tetrahedra.

 @note   This function is purely topological because it takes only
 VolumeVertexIndex into account without attempting to access coordinates of
 the vertices.
 */
std::vector<VolumeElement> SplitToTetrahedra(
    VolumeVertexIndex v0, VolumeVertexIndex v1, VolumeVertexIndex v2,
    VolumeVertexIndex v3, VolumeVertexIndex v4, VolumeVertexIndex v5,
    VolumeVertexIndex v6, VolumeVertexIndex v7);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
