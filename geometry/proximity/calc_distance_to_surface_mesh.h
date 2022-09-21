#pragma once

#include "drake/geometry/proximity/triangle_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

// TODO(xuchenhan-tri): This uses an inefficient O(number of triangles)
// algorithm and can be accelerated with a BVH.
/* Computes the shortest unsigned distance between the point p_WQ and any point
 on the surface mesh `mesh_W`. The returned valued is non-negative.
 @pre Each element in `mesh_W` has non-zero area. */
double CalcDistanceToSurfaceMesh(const Vector3<double>& p_WQ,
                                 const TriangleSurfaceMesh<double> mesh_W);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
