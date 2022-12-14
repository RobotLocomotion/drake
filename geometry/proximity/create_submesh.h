#pragma once

#include <vector>

#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

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

