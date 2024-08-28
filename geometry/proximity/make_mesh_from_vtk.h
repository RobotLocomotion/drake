#pragma once

#include <filesystem>
#include <string>

#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

/* Creates a VolumeMesh of a possibly non-convex object from a VTK file
 containing its tetrahedral mesh. It complements MakeConvexVolumeMesh().

 @param[in] mesh  The mesh specifying the VTK data to use.
 @retval  volume_mesh
 @tparam_nonsymbolic_scalar

 @throw std::exception if `mesh` doesn't specify a .vtk extension.
        std::exception if a tetrahedral element has non-positive volume.
        std::exception if the `mesh` specification refers to non-VTK file.
        std::exception if the VTK file does not contain a tetrahedral mesh.
        For example, it will throw if `mesh` refers to Obj file instead of
        VTK file, or the VTK file has a triangle mesh or a quadrilateral mesh
        instead of a tetrahedral mesh.
 */
template <typename T>
VolumeMesh<T> MakeVolumeMeshFromVtk(const Mesh& mesh);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
