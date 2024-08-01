#pragma once

#include <filesystem>
#include <istream>
#include <string>

#include "drake/geometry/in_memory_mesh.h"
#include "drake/geometry/proximity/polygon_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

/* @group Convex Hull for Meshes

 These functions create a convex hull (represented by a PolygonSurfaceMesh) for
 mesh data. The functions differ in where the mesh data comes from. Otherwise,
 their parameters and semantics (documented here) are the same.

 The convex hull is built upon *all* of the vertex values in the mesh data
 (regardless of how the mesh is organized or even if it includes vertices that
 are not otherwise incorporated in faces).

 The vertex positions can be scaled uniformly around the origin of the mesh
 data's canonical frame.

 The mesh can also be "inflated" by a given margin amount δ. If margin is zero,
 no inflation is applied and the convex hull of the original set of vertices is
 computed.

 With "inflation", we mean the process of moving each of the faces in the convex
 hull an amount δ along its (outwards) normal. This effectively grows or
 "inflates" the convex hull, producing an additional layer of thickness δ all
 around the convex hull of the original set of vertices.

 @note Geometry inflation by a margin δ is meant to support speculative
 constraints (contact constraints before contact actually happens) for
 "compliant" hydroelastic. Since the hydroelastic model is not meant for thin
 objects, inflation of planar (zero thickness) meshes is not implemented. Margin
 is ignored for planar meshes.

 These functions throw an exception if:
   - the mesh data comes from an unsupported format,
   - the mesh data is ill formed,
   - the referenced mesh data is degenerate (insufficient number of vertices,
     co-linear or coincident vertices, etc.) All of the vertices lying on a
     plane is *not* degenerate,
   - there is an unforeseen error in computing the convex hull,
   - `scale` is not strictly positive, or
   - `margin` is negative.
 */
//@{

// TODO(SeanCurtis-TRI): Before merging this for real, either support all mesh
// file types, or document that it's .obj only.
/* The mesh data is specified by the contents of a mesh file interpreted
 according to the provided `extension`. */
PolygonSurfaceMesh<double> MakeConvexHullFromContents(
    const InMemoryMesh& mesh, std::string_view extension, double scale,
    double margin = 0);

/* The mesh data is specified by a path to an on-disk file of supported type. */
PolygonSurfaceMesh<double> MakeConvexHull(const std::filesystem::path mesh_file,
                                          double scale, double margin = 0);

//@}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
