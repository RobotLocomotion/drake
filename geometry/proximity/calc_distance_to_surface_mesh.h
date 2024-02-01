#pragma once

#include "drake/geometry/proximity/bvh.h"
#include "drake/geometry/proximity/obb.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

/* Computes the shortest unsigned distance between the point p_WQ and any point
 on the surface mesh `mesh_W`. The returned value is non-negative.
 The algorithm is accelerated with the bounding volume hierarchy `bvh_W` of
 the surface mesh.
 @pre Each element in `mesh_W` has non-zero area. */
double CalcDistanceToSurfaceMesh(
    const Vector3<double>& p_WQ, const TriangleSurfaceMesh<double>& mesh_W,
    const Bvh<Obb, TriangleSurfaceMesh<double>>& bvh_W);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
