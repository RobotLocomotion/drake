#pragma once

#include <istream>
#include <string>
#include <vector>

#include "drake/geometry/proximity/surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

/**
 Constructs a surface mesh from a Wavefront .obj file and optionally scales
 coordinates by the given scale factor. Polygons will be triangulated if they
 are not triangles already.
 @param filename
     A valid file name with absolute path or relative path.
 @param scale
     An optional scale to coordinates.
 @throws std::runtime_error if `filename` doesn't have a valid file path or the
     .obj file doesn't define a single object. This can happen if it is
     empty, if there are multiple object-name statements (e.g., "o object
     name"), or if there are faces defined outside a single object-name
     statement.
 @return surface mesh
 @note
     We only support .obj files with a single object.
 */
SurfaceMesh<double> ReadObjToSurfaceMesh(const std::string& filename,
                                         double scale = 1.0);

/**
 Overload of @ref ReadObjToSurfaceMesh(const std::string&, double) with the
 Wavefront .obj file given in std::istream.
 */
SurfaceMesh<double> ReadObjToSurfaceMesh(std::istream* input_stream,
                                         double scale = 1.0);
}  // namespace internal
}  // namespace geometry
}  // namespace drake
