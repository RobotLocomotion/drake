#pragma once

#include "drake/common/drake_export.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal DRAKE_NO_EXPORT {

using Eigen::Vector3d;

/* Computes the signed distance to the capsule's surface from point P. The point
 is measured and expressed in the capsule's canonical frame C. Negative values
 for points *inside* the capsule, positive value for points outside.  */
double CalcDistanceToSurface(const Capsule& capsule, const Vector3d& p_CP);

// clang-format off
}  // namespace internal
// clang-format on
}  // namespace geometry
}  // namespace drake
