#pragma once

/* @file Detect problematic simplices (tetrahedron, triangle, edge) in a
 tetrahedral mesh. They cause problems in contact simulation because they
 interpolate distance-to-boundary as zero even though they are not on the
 boundary surface.
 */

#include <vector>

#include "drake/common/sorted_pair.h"
#include "drake/geometry/proximity/sorted_triplet.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

/* Detects any tetrahedron with its four vertices on the boundary surface.
 In such a tetrahedron, the interpolated distance-to-boundary evaluates to zero.

 Return a list of indices of tetrahedra in the mesh.
 */
std::vector<int> DetectNullTetrahedron(const VolumeMesh<double>& mesh_M);

/* Detects any interior triangle with its three vertices on the boundary
 surface. On such a triangle, the interpolated distance-to-boundary evaluates to
 zero. In one of the two tetrahedra sharing such a triangle, the gradient of the
 approximated signed distance is in the wrong direction.
 */
std::vector<SortedTriplet<int>> DetectNullInteriorTriangle(
    const VolumeMesh<double>& mesh_M);

/* Detect any interior edge with its two vertices on the boundary surface.
 On such an edge, the interpolated distance-to-boundary evaluates to zero.
 In some of the tetrahedra sharing such an edge, the gradient of the
 approximated signed distance is in the wrong direction.
 */
std::vector<SortedPair<int>> DetectNullInteriorEdge(
    const VolumeMesh<double>& mesh_M);

// TODO(DamrongGuoy): Remove the following functions when no longer needed or
//  move it somewhere else.  For now, I use it to visually verify that we
//  can detect the correct tetrahedra.

/* Creates a tetrahedral mesh of the specified tetrahedron indices of the
 given mesh.

 @note The entries in tetrahedron_indices will be sorted and uniquified, so
       the order of the output tetrahedra may not be the same as the order of
       the given indices.

 @pre Each entry in tetrahedron_indices is a valid index.
 */
VolumeMesh<double> CreateSubMesh(const VolumeMesh<double>& mesh_M,
                                 const std::vector<int>& tetrahedron_indices);

VolumeMesh<double> RemoveUnusedVertices(const VolumeMesh<double>& mesh_M);

VolumeMesh<double> CutZSubMesh(const VolumeMesh<double>& mesh_M,
                              double min_z, double max_z);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
