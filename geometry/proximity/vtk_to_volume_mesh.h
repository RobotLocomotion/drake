#pragma once

#include <string>

#include "drake/geometry/in_memory_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

/* Instantiates a VolumeMesh from VTK file data.

 The VTK file data is a subset of VTK files (legacy, serial, ASCII,
 UNSTRUCTURED_GRID). A limited number of tags are supported; the rest ignored.
 The file format is described in:
 https://vtk.org/wp-content/uploads/2015/04/file-formats.pdf

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

 Other sections like POINT_DATA and CELL_DATA, e.g. per-vertex pressure value
 or per-tetrahedron velocity field are ignored.

 @param mesh_source      The source of mesh data to parse.
 @param scale            An optional scale to coordinates.
 @return tetrahedral volume mesh

 @note Error handling from parsing the file is performed by VTK library.

 @throw  std::exception if the file does not exist or unsupported.
         std::exception for non-positive scale factors. */
VolumeMesh<double> ReadVtkToVolumeMesh(const MeshSource& mesh_source,
                                       double scale = 1.0);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
