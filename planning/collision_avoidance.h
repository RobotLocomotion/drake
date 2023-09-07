#pragma once

#include <optional>

#include "drake/planning/collision_checker.h"

namespace drake {
namespace planning {
namespace internal {

/* Computes the gradient ∇f = (∂f/∂q)ᵀ of an unspecifed distance function "f".

 "f" combines the distances ϕᵢ of the given collision checker's RobotClearance
 data, reconciling multiple simultaneous potential collisions via a penalty
 method. The closer to penetration, the more weight that potential collision
 has. No measurements greater than `max_clearance` contribute to the
 calculation, and any distances less than `max_penetration` are all treated
 equally, regardless of the degree of penetration.

 Therefore, the gradient ∇f points `q` into a "clearer" state, reducing the
 likelihood of collision.

 @param checker             The checker used to calculate clearance data.
 @param q                   The robot configuration at which we compute ∂f/∂q.
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
 @param context_number      An optional context number to specify the implicit
                            context to use.
 @param context             An optional collision checker context.
 @note Either a context, a context number, or neither may be provided. If
       neither is provided, the implicit context corresponding to the current
       OpenMP thread is used.
 @pre q.size() == checker.GetZeroConfiguration().size().
 @pre max_penetration <= 0.
 @pre max_clearance >= 0.
 @pre max_clearance > max_penetration.
 @pre if context != nullptr, it is a context managed by checker.
 @pre context != nullptr and context_number != nullopt cannot be combined.

 @ingroup planning_collision_checker */
Eigen::VectorXd ComputeCollisionAvoidanceDisplacement(
    const CollisionChecker& checker, const Eigen::VectorXd& q,
    double max_penetration, double max_clearance,
    std::optional<int> context_number = std::nullopt,
    CollisionCheckerContext* context = nullptr);

// TODO(jwnimmer-tri) Before we promote the above function out of the internal
// namespace, consider renaming "Displacement" to "Gradient" to better reflect
// the units on the return type.

// TODO(jwnimmer-tri) Before we promote the above function out of the internal
// namespace, consider changing its optional `context` parameter above into a
// distinct function named ComputeContextCollisionAvoidanceDisplacement, for
// consistency with how CollisionChecker names its member functions.

}  // namespace internal
}  // namespace planning
}  // namespace drake
