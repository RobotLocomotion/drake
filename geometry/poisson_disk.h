#pragma once

#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

/* Generates a set of samples within the given shape. A point is deemed as
 "inside" the shape if its signed distance to the shape is non-positive. The
 generated set of points are such that no two samples are closer than a
 user-specified distance. See [Bridson, 2007] for details.

 [Bridson, 2007] Bridson, Robert. "Fast Poisson disk sampling in arbitrary
 dimensions." SIGGRAPH sketches 10.1 (2007): 1.

 @pre radius > 0.
 @note The algorithm guarantees no point can be sampled in the AABB that's more
 than `radius` away from all existing points. */
std::vector<Vector3<double>> PoissonDiskSampling(double radius,
                                                 const Shape& shape);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
