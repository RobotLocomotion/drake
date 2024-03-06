#pragma once

#include "drake/geometry/proximity/polygon_surface_mesh.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

/** Creates a triangle mesh from a polygon mesh. The triangle mesh has the
 exact same vertices as the polygon mesh. To whatever degree the polygon
 mesh has duplicate vertices, the resulting triangle mesh will have the same.
 The triangles will likewise mirror the winding present in the polygon mesh.

 @pre Each polygon in poly_mesh is convex. */
TriangleSurfaceMesh<double> MakeTriangleFromPolygonMesh(
    const PolygonSurfaceMesh<double>& poly_mesh);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
