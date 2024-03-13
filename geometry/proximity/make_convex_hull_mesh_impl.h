#pragma once

#include <filesystem>

#include "drake/geometry/proximity/polygon_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

/* Creates a polygonal mesh representing the convex hull of the vertices
 contained in the named `mesh_file` (scaled with the given `scale` value).

 @param mesh_file   A path to a valid mesh file to bound.
 @param scale       All vertices will be multiplied by this value prior to
                    computation.

 @throws if `mesh_file` references anything but an .obj, .vtk volume mesh, or
         .gltf.
 @throws if the referenced mesh data is degenerate (insufficient number of
            vertices, co-linear or coincident vertices, etc.) All of the
            vertices lying on a plane is *not* degenerate.
 @throws if there is an unforeseen error in computing the convex hull. */
PolygonSurfaceMesh<double> MakeConvexHull(const std::filesystem::path mesh_file,
                                          double scale);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
