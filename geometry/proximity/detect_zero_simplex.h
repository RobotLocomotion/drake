#pragma once

/* @file
 Detects if a tetrahedral mesh evaluates to zero signed-distance anywhere in its
 interior. A hydroelastic pressure field built on a mesh with this defect will
 have zero pressure in its interior.  Because signed distance is defined at
 vertices and interpolated everywhere else, detection focuses on positive
 dimensional simplices (tetrahedra, triangles, edges).  A simplex is
 problematic, if all of its vertices lie on the boundary, and if the simplex
 intersects the mesh's interior.  For example, a mesh with only a single
 tetrahedron is problematic since all its vertices lie on the mesh's boundary.

 A tetrahedral mesh may have other problems. For example, a tetrahedron with
 all vertices at the same distance from the boundary. A hydroelastic pressure
 field on such a tetrahedron will have zero pressure gradient. In this
 implementation, we are not detecting them yet.
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
 boundary. The interpolated signed distance evaluates to zero inside
 such a tetrahedron.

 @return a list of indices of the problematic tetrahedra in the tetrahedral
         mesh.
 */
std::vector<int> DetectTetrahedronWithAllBoundaryVertices(
    const VolumeMesh<double>& mesh_M);

/* Detects problematic triangles.

 A triangle is problematic, if all of its vertices lie on the mesh's boundary,
 and if the triangle intersects the mesh's interior. The interpolated signed
 distance evaluates to zero inside such an interior triangle.

 @return a list of triple indices of vertices of problematic triangles in the
         tetrahedral mesh.
 */
std::vector<SortedTriplet<int>> DetectInteriorTriangleWithAllBoundaryVertices(
    const VolumeMesh<double>& mesh_M);

/* Detects problematic edges.

 An edge is problematic, if all of its vertices lie on the mesh's boundary,
 and if the edge intersects the mesh's interior. The interpolated signed
 distance evaluates to zero inside such an interior edge.

 @return a list of double indices of vertices of problematic edges in the
         tetrahedral mesh.
 */
std::vector<SortedPair<int>> DetectInteriorEdgeWithAllBoundaryVertices(
    const VolumeMesh<double>& mesh_M);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
