#pragma once

#include "drake/geometry/proximity/polygon_surface_mesh.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {

/** Creates a mesh representing the convex hull of a Convex or Mesh shape.

 @param shape   The shape to bound.

 @throws if `shape` is not an instance of Mesh or Convex.
 @throws if the mesh references anything but an obj. */
PolygonSurfaceMesh<double> MakeConvexHull(const Shape& shape);

}  // namespace geometry
}  // namespace drake
