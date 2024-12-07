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

 @note The algorithm guarantees no point can be sampled in the AABB that's more
 than `radius` away from all existing points.
 @throws std::exception unless radius > 0.
 @throws std::exception if there is an error from poisson_disk_sampling library.
 @throws std::exception if the derived type of Shape hasn't provided an
 implementation (yet). */
std::vector<Vector3<double>> PoissonDiskSampling(double radius,
                                                 const Shape& shape);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
