#pragma once

#include <string>

#include "drake/geometry/proximity/mesh_field_linear.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/volume_mesh_field.h"

namespace drake {
namespace geometry {
namespace internal {

/** @name Export Volume/Surface Mesh and Field for visualization in ParaView.
 These functions export VolumeMesh, SurfaceMesh, VolumeMeshFieldLinear, or
 SurfaceMeshFieldLinear to VTK files (legacy, serial format) for
 visualization in ParaView. The file format is described in:
 https://vtk.org/wp-content/uploads/2015/04/file-formats.pdf
 */
//@{

/**
 Writes VolumeMesh to VTK file.
 @param file_name  A file name with absolute path or relative path.
 @param mesh       A tetrahedral volume mesh.
 @param title      Name of the data set will be written on the second line of
                   the VTK file.
 @throws std::runtime_error if unable to create the file.
 */
void WriteVolumeMeshToVtk(const std::string& file_name,
                          const VolumeMesh<double>& mesh,
                          const std::string& title);

/**
 Writes SurfaceMesh to VTK file.
 @param file_name  A file name with absolute path or relative path.
 @param mesh       A triangulated surface mesh.
 @param title      Name of the data set will be written on the second line of
                   the VTK file.
 @throws std::runtime_error if unable to create the file.
 */
void WriteSurfaceMeshToVtk(const std::string& file_name,
                           const SurfaceMesh<double>& mesh,
                           const std::string& title);

/**
 Writes VolumeMeshFieldLinear to VTK file.
 @param file_name  A file name with absolute path or relative path.
 @param field      A scalar field defined on a tetrahedral mesh.
 @param title      Name of the data set will be written on the second line of
                   the VTK file.
 @note `field.name()` will be written to VTK file with space ' ' replaced by
       underscore '_' on the "SCALARS field.name()" line.
 @throws std::runtime_error if unable to create the file.
 */
void WriteVolumeMeshFieldLinearToVtk(
    const std::string& file_name,
    const VolumeMeshFieldLinear<double, double>& field,
    const std::string& title);

/**
 Writes SurfaceMeshFieldLinear to VTK file.
 @param file_name  A file name with absolute path or relative path.
 @param field      A scalar field defined on a triangulated surface mesh.
 @param title      Name of the data set will be written on the second line of
                   the VTK file.
 @note `field.name()` will be written to VTK file with space ' ' replaced by
       underscore '_' on the "SCALARS field.name()" line.
 @throws std::runtime_error if unable to create the file.
 */
void WriteSurfaceMeshFieldLinearToVtk(
    const std::string& file_name,
    const SurfaceMeshFieldLinear<double, double>& field,
    const std::string& title);

//@}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
