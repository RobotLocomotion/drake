#pragma once

#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

/* Helper function for detecting overlap between two boxes A and B. Each box is
 defined in its own canonical frame. The box is aligned with the frame and
 described by a vector describing its "half size" (half its extent along each of
 its frame's axes). Another way to think about the half size is that it's the
 position vector from the box's origin to its most positive corner: p_BoC_B.

 @param half_size_a   The half size of box A.
 @param half_size_b   The half size of box B.
 @param X_AB          The relative pose between boxes A and B. */
bool BoxesOverlap(const Vector3<double>& half_size_a,
                  const Vector3<double>& half_size_b,
                  const math::RigidTransformd& X_AB);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
