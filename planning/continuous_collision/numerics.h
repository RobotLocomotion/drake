#pragma once

/** @file
Single home of the numerical accounting used everywhere (the numerical policy).

Let φ̂ be the oracle's reported signed distance at the node's representative
configuration, τ the oracle accuracy contract (|φ̂ − φ_true| ≤ τ on the
at-or-above-threshold branch), Δ the motion bound for the node, m the
effective threshold (margin + padding), and ε the certificate slack.

 - Certified:          φ̂ − τ − Δ > m + ε   (sound by the displacement lemma:
                       every configuration on the node keeps clearance > m).
 - Definite violation: φ̂ + τ < m           (the true clearance at an exactly
                       on-trajectory configuration is below threshold).
 - Otherwise the pair is gray and drives subdivision.

The certificate is mathematical modulo τ and ε: the library does not use
directed rounding (that hardening is a future extension); ε defaults
to 1e-9 m which dominates the accumulated FP error of the w/λ/dot-product
expression depths involved. */

namespace drake {
namespace planning {
namespace continuous_collision {

/** True iff the pair is certified on the whole node.
@ingroup planning_collision_checker */
inline bool IsCertified(double phi_hat, double tau, double motion_bound,
                        double threshold, double slack) {
  return phi_hat - tau - motion_bound > threshold + slack;
}

/** True iff the representative configuration is a definite violation.
@ingroup planning_collision_checker */
inline bool IsDefiniteViolation(double phi_hat, double tau, double threshold) {
  return phi_hat + tau < threshold;
}

}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
