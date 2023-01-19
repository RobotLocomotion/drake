#pragma once

#include "drake/planning/collision_checker.h"

namespace anzu {
namespace planning {

/** Computes a displacement in configuration space, Δq, that moves the provided
 configuration `q` into a "clearer" state, reducing the likelihood of collision.

 The displacement vector Δq = Σᵢ(wᵢ⋅Jqᵣ_ϕᵢ), where

   - The iᵗʰ term comes from the iᵗʰ clearance measurement on the robot. It may
     measure the clearance between robot and environment or robot and robot.
   - The number of measurements depends on how the robot is represented
     geometrically in `checker`.
   - ϕᵢ is the iᵗʰ measured distance and Jq_ϕᵢ is the partial derivative of ϕᵢ
     with respect to qᵣ, the robot state.
   - wᵢ is the weight for iᵗʰ measurement, defined as wᵢ = (c - ϕᵢ) / (c - p),
     where c = `max_clearance` and p = `max_penetration` and c > p.

 Intuitively, the displacement vector Δq tries to reconcile multiple
 simultaneous potential collisions via a penalty method. The closer to
 penetration, the more weight that potential collision has. No measurements
 greater than `max_clearance` contribute to the calculation, and any distances
 less than `max_penetration` are all treated equally, regardless of the degree
 of penetration.

 @param checker             The checker used to define ϕᵢ and Jqᵣ_ϕᵢ.
 @param q                   The robot configuration at which we compute Δq.
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
 @retval Δq the configuration displacement to increase robot clearance.
 @pre q.size() == checker.GetZeroConfiguration().size().
 @pre max_penetration <= 0.
 @pre max_clearance >= 0.
 @pre max_clearance > max_penetration.
 @pre if context != nullptr, it is a context managed by checker. */
Eigen::VectorXd ComputeCollisionAvoidanceDisplacement(
    const drake::planning::CollisionChecker& checker, const Eigen::VectorXd& q,
    double max_penetration, double max_clearance,
    drake::planning::CollisionCheckerContext* context = nullptr);

}  // namespace planning
}  // namespace anzu
