#pragma once

#include "drake/planning/collision_checker.h"

namespace drake {
namespace planning {
namespace internal {

/* Computes the gradient ∂𝐀/∂q of an unspecifed distance function "𝐀".
 The value of 𝐀 is based on a weighted combination of CollisionChecker's
 RobotClearance data, and so its gradient points `q` into a "clearer" state,
 reducing the likelihood of collision.

 𝐀 reconciles multiple simultaneous potential collisions via a penalty method.
 The closer to penetration, the more weight that potential collision has. No
 measurements greater than `max_clearance` contribute to the calculation, and
 any distances less than `max_penetration` are all treated equally, regardless
 of the degree of penetration.


 @param checker             The checker used to calculate clearance data.
 @param q                   The robot configuration at which we compute ∂𝐀/∂q.
 @param max_penetration     The bottom of the range; a non-positive value
                            representing the level of penetration that saturates
                            the weight. Penetration occurs inside an object
                            where ϕ < 0. Setting the value to zero will give all
                            penetration equal weight, regardless of depth.
                            Making this value *more* negative won't increase the
                            weight of deep penetrations, but it will reduce the
                            weight of shallow penetrations.
 @param max_clearance       The top of the range; a non-negative value beyond
                            which possible collisions do not contribute. Two
                            objects are "clear" where ϕ > 0. Setting the value
                            to zero will only consider collisions and not nearby
                            objects. As the value increases, more and more
                            distant objects will influence the calculation.
 @param context             An optional collision checker context. If none is
                            provided, the checker's context for the current
                            thread is used.
 @retval ∂𝐀/∂q the gradient indicating increased robot clearance.
 @pre q.size() == checker.GetZeroConfiguration().size().
 @pre max_penetration <= 0.
 @pre max_clearance >= 0.
 @pre max_clearance > max_penetration.
 @pre if context != nullptr, it is a context managed by checker. */
Eigen::VectorXd ComputeCollisionAvoidanceDisplacement(
    const CollisionChecker& checker, const Eigen::VectorXd& q,
    double max_penetration, double max_clearance,
    CollisionCheckerContext* context = nullptr);

// TODO(jwnimmer-tri) Before we promote the above function out of the internal
// namespace, consider changing its optional `context` parameter above into a
// distinct function named ComputeContextCollisionAvoidanceDisplacement, for
// consistency with how CollisionChecker names its member functions.

}  // namespace internal
}  // namespace planning
}  // namespace drake
