#pragma once

#include <istream>
#include <string>
#include <vector>

#include "drake/geometry/proximity/surface_mesh.h"

namespace drake {
namespace geometry {

/**
 Constructs a surface mesh from a Wavefront .obj file and optionally scales
 coordinates by the given scale factor. Polygons will be triangulated if they
 are not triangles already. All objects in the .obj file will be merged into
 the surface mesh. See https://en.wikipedia.org/wiki/Wavefront_.obj_file for
 the file format.
 @param filename
     A valid file name with absolute path or relative path.
 @param scale
     An optional scale to coordinates.
 @throws std::runtime_error if `filename` doesn't have a valid file path, or the
     file has no faces.
 @return surface mesh
 */
SurfaceMesh<double> ReadObjToSurfaceMesh(const std::string& filename,
                                         double scale = 1.0);

/**
 Overload of @ref ReadObjToSurfaceMesh(const std::string&, double) with the
 Wavefront .obj file given in std::istream.
 */
SurfaceMesh<double> ReadObjToSurfaceMesh(std::istream* input_stream,
                                         double scale = 1.0);
}  // namespace geometry
}  // namespace drake
