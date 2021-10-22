#pragma once

#include <algorithm>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/sorted_pair.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

/* Generates a tetrahedral volume mesh approximating a given capsule by medial
 axis (MA) subdivision. The capsule is subdivided into blocks using its MA,
 and then the blocks are subdivided into tetrahedra. Using this mesh with
 MeshFieldLinear, we can represent the distance-to-boundary function φ of the
 capsule (which can be scaled by a constant elastic modulus to create a
 hydroelastic pressure field) accurately and economically. However, this mesh
 may not be suitable for other kinds of pressure fields, e.g., a squared
 distance field.

 The capsule is parameterized by two parameters (radius, length). Its length
 describes the extent of the cylindrical portion of the capsule aligned with
 the Z axis. The radius describes the radius of the two hemi-spherical "caps"
 of the capsule, symmetric around the Z axis. The capsule's MA is the set of all
 points having more than one closest point on the capsule's boundary. (See
 https://en.wikipedia.org/wiki/Medial_axis for more about the medial axis.) For
 any parameterization of the capsule, the MA consists of a line segment between
 the points M₀ = (0, 0, -0.5 * length) and M₁ = (0, 0, 0.5 * length):



                        Z (axis of rotation)
                        ↑
                        |
     ___             _______
      |           -          -
   radius      ／               ＼
      |       /                  \
     ___     +       M₁ o         +
      |      |          ║         |
      |      |          ║         |
      |      |          ║         |
      |      |          ║         |
   length    |          o         |  ----→ X
      |      |          ║         |
      |      |          ║         |
      |      |          ║         |
      |      |          ║         |
     ___     +       M₀ o         +
              \                  /
                ＼             ／
                   - _______ -

                        |-radius--|

 The following picture files show examples of meshes generated from
 MakeCapsuleVolumeMesh(). The files are distributed with the source code.

 | geometry/proximity/images/capsule_mesh_medial_axis.png |
 | Meshes of a capsule viewed from the outside. |

 | geometry/proximity/images/capsule_mesh_medial_axis_slice.png |
 | x = 0 slice of the capsule mesh, showing the triangulation. |

 The output mesh will have these properties:

   1. The generated vertices are unique. There are no repeating vertices in
      the list of vertex coordinates.
   2. The generated tetrahedra are _conforming_. Two tetrahedra intersect in
      their shared face, or shared edge, or shared vertex, or not at all.
      There is no partial overlapping of two tetrahedra.
   3. The number of vertices per one circular rim of the capsule is ⌈2πr/h⌉,
      where r = capsule.radius(), and h is resolution_hint.

 @param[in] capsule
     The capsule shape specification.
 @param[in] resolution_hint
     The target length of each edge of the mesh on each of the bottom and the
     top circular rims of the capsule. The coarsest possible mesh (a triangular
     prism with tetrahedral caps) is guaranteed for any value of
     `resolution_hint` greater than or equal to 2·π·radius/3. The finest
     possible mesh (with 1 million tetrahedra) is guaranteed for any value of
     `resolution_hint` less than or equal to 2·π·radius/706.
 @retval volume_mesh
 @tparam_nonsymbolic_scalar
 */
template <typename T>
VolumeMesh<T> MakeCapsuleVolumeMesh(const Capsule& capsule,
                                    double resolution_hint);

// Creates a surface mesh for the given `capsule`; the level of
// tessellation is guided by the `resolution_hint` parameter in the same way
// as MakeCapsuleVolumeMesh.
//
// @param[in] capsule
//    Specification of the parameterized capsule the output surface mesh
//    should approximate.
// @param[in] resolution_hint
//    The positive characteristic edge length for the mesh. The coarsest
//    possible mesh (a triangular prism with tetrahedral caps) is guaranteed
//    for any value of `resolution_hint` greater than or equal to 2·π·radius/3.
//    The finest possible mesh is guaranteed for any value of `resolution_hit`
//    less than or equal to 2·π·radius/706.
// @returns The triangulated surface mesh for the given capsule.
// @pre resolution_hint is positive.
// @tparam T
//    The Eigen-compatible scalar for representing the mesh vertex positions.
template <typename T>
TriangleSurfaceMesh<T> MakeCapsuleSurfaceMesh(const Capsule& capsule,
                                              double resolution_hint) {
  DRAKE_DEMAND(resolution_hint > 0.0);
  return ConvertVolumeToSurfaceMesh<T>(
      MakeCapsuleVolumeMesh<T>(capsule, resolution_hint));
}
}  // namespace internal
}  // namespace geometry
}  // namespace drake
