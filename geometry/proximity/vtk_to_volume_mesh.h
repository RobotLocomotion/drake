#pragma once

#include <functional>
#include <istream>
#include <string>
#include <string_view>

#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

/* @name Import tetrahedral volume meshes from VTK files for deformable
 geometries. These internal functions import VolumeMesh from a subset of VTK
 files (legacy, serial, ASCII, UNSTRUCTURED_GRID). They only support a limited
 number of tags and ignore the rest. The file format is described in:
 https://vtk.org/wp-content/uploads/2015/04/file-formats.pdf

 For simplicity, there are two overloads (taking a file name or taking an input
 stream), so we can set up and verify unit tests conveniently. It also lets
 users "inline" the meshes in their code without managing file systems.
 */
//@{

/* Reads VolumeMesh from VTK file.
 It looks for lines like these to describe positions of vertices and the four
 vertices of each tetrahedron.

      POINTS 4 float
      0.0 0.0 0.0
      1.0 0.0 0.0
      0.0 1.0 0.0
      0.0 0.0 1.0

      CELLS 1 5
      4 0 1 2 3

      CELL_TYPES 1
      10

 Other sections like POINT_DATA and CELL_DATA for field variables are ignored.

 @param filename    A file name with absolute path or relative path.
 @param on_warning  An optional callback that will receive warning message(s)
                    encountered while reading the mesh. When not provided,
                    drake::log() will be used.
 @throws std::exception if `filename` doesn't have a valid file path, or the
                    file has no tetrahedron.
 @return tetrahedral volume mesh

 @warn It has limited error checking and supports only a subset of VTK files.
 */
VolumeMesh<double> ReadVtkToVolumeMesh(
    const std::string& filename,
    std::function<void(std::string_view)> on_warning = {});

/* Overload of @ref ReadVtkToVolumeMesh(const std::string&) with the VTK file
 given in std::istream.
 @throws std::exception if `input_stream` is null.
 */
VolumeMesh<double> ReadVtkToVolumeMesh(
    std::istream* input_stream,
    std::function<void(std::string_view)> on_warning = {});

//@}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
