#pragma once

#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

/* Creates a VolumeMesh of a possibly non-convex object from a VTK file
 containing its tetrahedral mesh. It complements MakeConvexVolumeMesh().

 @param[in] mesh  The mesh specification containing VTK file name.
 @retval  volume_mesh
 @tparam_nonsymbolic_scalar

 @throw std::exception if a tetrahedral element has negative volume.
 */
template <typename T>
VolumeMesh<T> MakeNonConvexVolumeMeshFromVtk(const Mesh& mesh);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
