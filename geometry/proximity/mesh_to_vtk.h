#pragma once

#include <string>

#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/triangle_surface_mesh_field.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/volume_mesh_field.h"

namespace drake {
namespace geometry {
namespace internal {

/* @name Export Volume/Surface Mesh and Field for visualization in ParaView.
 These functions export VolumeMesh, TriangleSurfaceMesh, VolumeMeshFieldLinear,
 or TriangleSurfaceMeshFieldLinear to VTK files (legacy, serial format) for
 visualization in ParaView. The file format is described in:
 https://vtk.org/wp-content/uploads/2015/04/file-formats.pdf
 */
//@{

/*
 Writes VolumeMesh to VTK file.
 @param file_name  A file name with absolute path or relative path.
 @param mesh       A tetrahedral volume mesh.
 @param title      Name of the data set will be written on the second line of
                   the VTK file.
 @throws std::exception if unable to create the file.
 */
void WriteVolumeMeshToVtk(const std::string& file_name,
                          const VolumeMesh<double>& mesh,
                          const std::string& title);

/*
 Writes TriangleSurfaceMesh to VTK file.
 @param file_name  A file name with absolute path or relative path.
 @param mesh       A triangulated surface mesh.
 @param title      Name of the data set will be written on the second line of
                   the VTK file.
 @throws std::exception if unable to create the file.
 */
void WriteSurfaceMeshToVtk(const std::string& file_name,
                           const TriangleSurfaceMesh<double>& mesh,
                           const std::string& title);

/*
 Writes VolumeMeshFieldLinear to VTK file.
 @param file_name  A file name with absolute path or relative path.
 @param field_name Name of the field quantity to be written to the VTK file.
 @param field      A scalar field defined on a tetrahedral mesh.
 @param title      Name of the data set will be written on the second line of
                   the VTK file.
 @note `field.name()` will be written to VTK file with space ' ' replaced by
       underscore '_' on the "SCALARS field.name()" line.
 @throws std::exception if unable to create the file.
 */
void WriteVolumeMeshFieldLinearToVtk(
    const std::string& file_name, const std::string& field_name,
    const VolumeMeshFieldLinear<double, double>& field,
    const std::string& title);

/*
 Writes TriangleSurfaceMeshFieldLinear to VTK file.
 @param file_name  A file name with absolute path or relative path.
 @param field_name Name of the field quantity to be written to the VTK file.
 @param field      A scalar field defined on a triangulated surface mesh.
 @param title      Name of the data set will be written on the second line of
                   the VTK file.
 @note `field.name()` will be written to VTK file with space ' ' replaced by
       underscore '_' on the "SCALARS field.name()" line.
 @throws std::exception if unable to create the file.
 */
void WriteTriangleSurfaceMeshFieldLinearToVtk(
    const std::string& file_name, const std::string& field_name,
    const TriangleSurfaceMeshFieldLinear<double, double>& field,
    const std::string& title);

//@}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
