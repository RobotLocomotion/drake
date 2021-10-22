#pragma once

#include <functional>
#include <istream>
#include <string>
#include <string_view>
#include <vector>

#include "drake/geometry/proximity/triangle_surface_mesh.h"

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
 @param on_warning
     An optional callback that will receive warning message(s) encountered
     while reading the mesh.  When not provided, drake::log() will be used.
 @throws std::exception if `filename` doesn't have a valid file path, or the
     file has no faces.
 @return surface mesh
 */
TriangleSurfaceMesh<double> ReadObjToTriangleSurfaceMesh(
    const std::string& filename,
    double scale = 1.0,
    std::function<void(std::string_view)> on_warning = {});

/**
 Overload of @ref ReadObjToTriangleSurfaceMesh(const std::string&, double) with
 the Wavefront .obj file given in std::istream.
 */
TriangleSurfaceMesh<double> ReadObjToTriangleSurfaceMesh(
    std::istream* input_stream,
    double scale = 1.0,
    std::function<void(std::string_view)> on_warning = {});

}  // namespace geometry
}  // namespace drake
