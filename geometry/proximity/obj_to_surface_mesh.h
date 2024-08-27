#pragma once

#include <functional>
#include <istream>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "drake/common/diagnostic_policy.h"
#include "drake/geometry/in_memory_mesh.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"

// TODO(SeanCurtis-TRI): This should distill further and combine with
// geometry/proximity/read_obj.* to further constrain the amount of ad hoc
// obj parsing we do.

namespace drake {
namespace geometry {
namespace internal {

/* Creates a triangle mesh from the obj data contained in the `input_stream`.
 Parsing will continue through warnings. If the given diagnostic policy isn't
 configured to stop for errors, std::nullopt is returned.
 @pre `diagnostic` has both warning and error handlers defined. */
std::optional<TriangleSurfaceMesh<double>> DoReadObjToSurfaceMesh(
    const MeshSource& source, double scale,
    const drake::internal::DiagnosticPolicy& diagnostic);

}  // namespace internal

/** Constructs a surface mesh from a Wavefront .obj file and optionally scales
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
 @return surface mesh */
TriangleSurfaceMesh<double> ReadObjToTriangleSurfaceMesh(
    const std::string& filename, double scale = 1.0,
    std::function<void(std::string_view)> on_warning = {});

/** Overload of @ref ReadObjToTriangleSurfaceMesh(const std::string&, double)
 with the Wavefront .obj file given in std::istream. */
TriangleSurfaceMesh<double> ReadObjToTriangleSurfaceMesh(
    std::istream* input_stream, double scale = 1.0,
    std::function<void(std::string_view)> on_warning = {},
    std::string_view description = "from_stream");

/** Overload of @ref ReadObjToTriangleSurfaceMesh(const std::string&, double)
 with the Wavefront .obj in a Mesh shape specification. */
TriangleSurfaceMesh<double> ReadObjToTriangleSurfaceMesh(
    const MeshSource& source, double scale,
    std::function<void(std::string_view)> on_warning = {});

}  // namespace geometry
}  // namespace drake
