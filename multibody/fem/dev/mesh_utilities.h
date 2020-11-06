#pragma once

#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/volume_mesh.h"
namespace drake {
namespace multibody {
namespace fem {
/** Creates a tetrahedron mesh of an axis-aligned rectangular block. The block
 lies in the octant with all positive coordinates in the world-frame and has one
 vertex at the origin. The mesh has `nx`+1, `ny`+1, and `nz`+1 vertices in the
 x, y, z direction respectively. The distance between consecutive vertices along
 each axis is equal to `h`. The vertices of each tetrahedron in the mesh is
 ordered such that the first three vertices define a triangle with its
 right-handed normal pointing towards the fourth vertex.
 @tparam_nonsymbolic_scalar T. */
template <typename T>
geometry::VolumeMesh<T> CreateRectangularBlockTetMesh(int nx, int ny, int nz,
                                                      T h);
}  // namespace fem
}  // namespace multibody
}  // namespace drake
