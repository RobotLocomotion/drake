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
 position vector from the box's center to its most positive corner (expressed
 in the box's canonical frame).

 @param half_size_a   The half size of box A expressed in A's canonical frame.
 @param half_size_b   The half size of box B expressed in B's canonical frame.
 @param X_AB          The relative pose between boxes A and B. */
bool BoxesOverlap(const Vector3<double>& half_size_a,
                  const Vector3<double>& half_size_b,
                  const math::RigidTransformd& X_AB);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
