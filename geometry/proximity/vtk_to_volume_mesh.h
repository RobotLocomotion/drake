#pragma once

#include <string>

#include "drake/geometry/mesh_source.h"
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
 @param scale            An optional scale to coordinates. Negative scales can
                         be used to mirror the geometry over axes. Zero scale
                         is considered valid for this function, but will likely
                         produce a degenerate mesh when used in Drake.
 @return tetrahedral volume mesh

 @note Negative scales will cause the geometry to mirror itself over the scale
 factor's corresponding axis. Depending on how many scale values are negative,
 the "winding" of the tets would get reversed (so what was previously considered
 inside is now outside). This function prevents this inside/outside inversion by
 changing the ordering of the per-tet vertex ordering (maintaining the original
 meshes definition of "inside" and "outside", even when the mesh has been
 mirrored.) This means that the ordering of vertices within each tet may not
 match the ordering of the input file; the index order will be permuted; indices
 [0 1 2 3] would become [2 1 0 3].

 @note Error handling from parsing the file is performed by VTK library.

 @throw  std::exception if the file does not exist or unsupported. */
VolumeMesh<double> ReadVtkToVolumeMesh(const MeshSource& mesh_source,
                                       const Eigen::Vector3d& scale = {1, 1,
                                                                       1});

}  // namespace internal
}  // namespace geometry
}  // namespace drake
