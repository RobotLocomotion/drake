#pragma once

#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

/* Generates a tetrahedral non-convex volume mesh by reading a VTK file
 specified by the `mesh` specification.

 @param[in] mesh  The mesh specification.
 @retval  volume_mesh
 @tparam_nonsymbolic_scalar

 @throw std::exception if a tetrahedral element has negative volume.
 */
template <typename T>
VolumeMesh<T> MakeNonConvexVolumeMeshFromVtk(const Mesh& mesh);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
