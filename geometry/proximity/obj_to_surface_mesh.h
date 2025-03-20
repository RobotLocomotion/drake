#pragma once

#include <filesystem>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "drake/common/diagnostic_policy.h"
#include "drake/geometry/mesh_source.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"

// TODO(SeanCurtis-TRI): This should distill further and combine with
// geometry/proximity/read_obj.* to further constrain the amount of ad hoc
// obj parsing we do.

namespace drake {
namespace geometry {
namespace internal {

/* Creates a triangle mesh from the obj data contained in the `source`.
 Parsing will continue through warnings. If the given diagnostic policy isn't
 configured to stop for errors, std::nullopt is returned.
 @pre `diagnostic` has both warning and error handlers defined. */
std::optional<TriangleSurfaceMesh<double>> DoReadObjToSurfaceMesh(
    const MeshSource& source, const Eigen::Vector3d& scale,
    const drake::internal::DiagnosticPolicy& diagnostic);

}  // namespace internal

/** Constructs a surface mesh from a Wavefront .obj file and optionally scales
 coordinates by the given scale factor. Polygons will be triangulated if they
 are not triangles already. All objects in the .obj file will be merged into
 the surface mesh. See https://en.wikipedia.org/wiki/Wavefront_.obj_file for
 the file format.
 @param filename
     A valid file name with absolute path or relative path.
 @param scale3
     A scale to coordinates.
 @param on_warning
     An optional callback that will receive warning message(s) encountered
     while reading the mesh.  When not provided, drake::log() will be used.
 @throws std::exception if there is an error reading the mesh data.
 @return surface mesh */
TriangleSurfaceMesh<double> ReadObjToTriangleSurfaceMesh(
    const std::filesystem::path& filename, const Eigen::Vector3d& scale3,
    std::function<void(std::string_view)> on_warning = {});

/** Variant that allows defining uniform scaling from a single scalar value. */
TriangleSurfaceMesh<double> ReadObjToTriangleSurfaceMesh(
    const std::filesystem::path& filename, double scale = 1.0,
    std::function<void(std::string_view)> on_warning = {});

/** Overload of @ref ReadObjToTriangleSurfaceMesh(const std::filesystem::path&,
 double) with the Wavefront .obj in a Mesh shape specification. */
TriangleSurfaceMesh<double> ReadObjToTriangleSurfaceMesh(
    const MeshSource& mesh_source, const Eigen::Vector3d& scale3,
    std::function<void(std::string_view)> on_warning = {});

/** Variant that allows defining uniform scaling from a single scalar value. */
TriangleSurfaceMesh<double> ReadObjToTriangleSurfaceMesh(
    const MeshSource& mesh_source, double scale = 1.0,
    std::function<void(std::string_view)> on_warning = {});

}  // namespace geometry
}  // namespace drake
