#pragma once

/* @file Detects if a tetrahedral mesh evaluates to zero signed-distance
 anywhere in its interior. A hydroelastic pressure field built
 on a mesh with this defect will have zero pressure in its interior.
 Because signed distance is defined at vertices and interpolated everywhere
 else, detection focuses on simplices (tetrahedra, triangles, edges).
 A simplex is problematic, if all of its vertices lie on the boundary, and
 if the simplex intersects the mesh's interior.
 For example, a mesh with only a single tetrahedron is problematic since all its
 vertices lie on the mesh's boundary.
 */

#include <vector>

#include "drake/common/sorted_pair.h"
#include "drake/geometry/proximity/sorted_triplet.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

/* Detects problematic tetrahedra.

 A tetrahedron is problematic, if all of its vertices lie on the mesh's
 boundary.

 @return a list of indices of the problematic tetrahedra in the tetrahedral
         mesh.
 */
std::vector<int> DetectNullTetrahedron(const VolumeMesh<double>& mesh_M);

/* Detects problematic triangles.

 A triangle is problematic, if all of its vertices lie on the mesh's boundary,
 and if the triangle intersects the mesh's interior.

 @return a list of triple indices of vertices of problematic triangles in the
         tetrahedral mesh.
 */
std::vector<SortedTriplet<int>> DetectNullInteriorTriangle(
    const VolumeMesh<double>& mesh_M);

/* Detects problematic edges.

 An edge is problematic, if all of its vertices lie on the mesh's boundary,
 and if the edge intersects the mesh's interior.

 @return a list of double indices of vertices of problematic edges in the
         tetrahedral mesh.
 */
std::vector<SortedPair<int>> DetectNullInteriorEdge(
    const VolumeMesh<double>& mesh_M);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
