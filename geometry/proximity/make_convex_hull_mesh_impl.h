#pragma once

#include <filesystem>
#include <string>

#include "drake/geometry/proximity/polygon_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

/* @group Convex Hull for Meshes

 These functions create a convex hull (represented by a PolygonSurfaceMesh) for
 mesh data. The functions differ in where the mesh data comes from. Otherwise,
 their parameters and semantics (documented here) are the same.

 The convex hull is build upon *all* of the vertex values in the mesh data
 (regardless of how the mesh is organized or even if it includes vertices that
 are not otherwise incorporated in faces).

 The vertex positions can be scaled uniformly around the origin of the mesh
 data's canonical frame.

 These functions throw an exception if:
   - the mesh data comes from an unsupported format,
   - the mesh data is ill formed,
   - the referenced mesh data is degenerate (insufficient number of vertices,
     co-linear or coincident vertices, etc.) All of the vertices lying on a
     plane is *not* degenerate, or
   - there is an unforeseen error in computing the convex hull. */
//@{

/* The mesh data is specified by a URL. Supports a file:// URL for all supported
 mesh types and a data: URL only for OBJ files (i.e., the prefix must be
 "data:model/obj," and the subsequent data stream should be the ASCII contents
 of an obj file). (Future versions will provide support for other supported
 file types.) */
PolygonSurfaceMesh<double> MakeConvexHullFromUrl(std::string_view geometry_url,
                                                 double scale);

// TODO(SeanCurtis-TRI): Consider nuking this in favor of the URL flavor.
/* The mesh data is specified by a path to an on-disk file of supported type.
 Equivalent to calling MakeConvexHullFromUrl() with the path encoded in a file
 URL. */
PolygonSurfaceMesh<double> MakeConvexHull(const std::filesystem::path mesh_file,
                                          double scale);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
