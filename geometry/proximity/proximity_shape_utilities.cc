#include "drake/geometry/proximity/proximity_shape_utilities.h"

namespace drake {
namespace geometry {
namespace internal {

double CalcDistanceToSurface(const Capsule& capsule, const Vector3d& p_CP) {
  const double half_length = capsule.length() / 2;
  const double z = std::clamp(p_CP.z(), -half_length, half_length);
  const Vector3d p_CQ(0, 0, z);
  return (p_CQ - p_CP).norm() - capsule.radius();
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
