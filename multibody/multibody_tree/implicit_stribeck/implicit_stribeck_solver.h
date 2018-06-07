#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace implicit_stribeck {

namespace internal {
/// This struct implements an internal (thus within `internal::`) detail of the
/// implicit Stribeck solver. The implicit Stribeck solver performs a
/// Newton-Raphson iteration, and at each k-th iteration, it computes a
/// tangential velocity update Δvₜᵏ. A standard Newton-Raphson strategy would
/// compute the tangential velocity at the next iteration (k+1) as
/// vₜᵏ⁺¹ = vₜᵏ + αΔvₜᵏ, where 0 < α < 1, is a coefficient typically obtained
/// with a line search algorithm to improve the convergence rate. Line search
/// works very well for smooth problems. However, even though the implicit
/// Stribeck is solving the root of a continuous function, this function has
/// very steep gradients only within the very small regions close to where the
/// tangential velocities are zero. These regions are circles in ℝ² of radius
/// equal to the stiction tolerance of the solver vₛ. For short, we refer to
/// these circular regions as the "stiction regions" and to their boundaries as
/// the "stiction circles". We refer to the region of ℝ² outside the stiction
/// region around the origin as the "sliding region".
/// The implicit Stribeck solver uses the following modified Stribeck function
/// describing the functional dependence of the Stribeck coefficient of friction
/// `μₛ`with slip speed: <pre>
///     μₛ(x) = ⌈ μ⋅x⋅(2.0 - x),  x  < 1
///             ⌊ μ            ,  x >= 1
/// </pre>
/// where x corresponds to the dimensionless slip speed `x = ‖vₜ‖ / vₛ` and
/// μ is the Coulomb's law coefficint of friction. The implicit Stribeck solver
/// makes no distinction between static and dynamic coefficients of friction and
/// therefore a single coefficient μ needs to be specified.
/// Since the Stribeck function used for the friction coefficient has a very
/// steep gradients only within the stiction region and zero outside of it,
/// gradients (that is the Jacobian of the residual) in the Newton-Raphson
/// iteration performed by the implicit Stribeck solver are large within
/// these small circular regions (stiction) and small outside them (slip).
/// We say that gradients within these stiction circles are "strong" and
/// gradients outside these regions are "weak".
/// These regions are so small comparatively to the velocity scales dealt with
/// by the implicit Stribeck solver, that effectively, the Newton-Raphson
/// iteration would only "see" a fixed dynamic coefficient of friction and it
/// would never be able to predict stiction. That is, if search direction Δvₜᵏ
/// computed within the Newton-Raphson is not limited in some way, the iteration
/// would never fall within the stiction regions where gradients are "strong" to
/// guide the convergence of the solution, to either stiction or sliding.
///
/// The remedy to this situation is to limit changes in the tangential
/// velocities at each iteration. Since the problem is purely geometric, it is
/// possible to devise a strategy that is appropriate for this particular
/// problem. We use the strategy outlined in [Uchida et al., 2015] and provide
/// particulars to our implementation below.
///
/// LimitDirectionChange implements a specific strategy with knowledge of the
/// implicit Stribeck iteration procedure. It is important to note that the
/// implicit Stribeck uses "soft norms" to avoid divisions by zero. That is,
/// friction forces are computed according to: <pre>
///   fₜ(vₜ) = -μ(‖vₜ‖ₛ) vₜ/‖vₜ‖ₛ
/// </pre>
/// where, to avoid the singularity at zero velocity, we use a "soft norm"
/// `‖vₜ‖ₛ = sqrt(vₜᵀvₜ + εᵥ²)`, with εᵥ a small fraction of vₛ. Due to the
/// use of soft norms, the gradient of `fₜ` with `vₜ` is now well defined,
/// but it goes to zero as `vₜ` approaches the origin. Therefore, gradients
/// are also "weak" in the neighborhood of `‖vₜ‖ₛ ≲ εᵥ`.
/// Due to this, External forcing (either from applied forces or from coupling
/// with other friction forces) has the potential to, mistakenly, force a
/// transition from stiction to sliding. The solver will most likely recover
/// from this, but this will result in a larger number of iterations.
/// LimitDirectionChange considers any tangential velocity vₜ (or change Δvₜ)
/// to be approximately zero if x = ‖vₜ‖/vₛ is smaller than `tolerance`
/// (see docs below, this is a dimensionless number << 1). We define
/// `εᵥ = tolerance⋅vₛ` (with units of m/s).
///
/// In what follows we list a number of special scenarios dealt with by
/// LimitDirectionChange. We use the observations made above.
///  - Transition from ‖vₜ‖ ≈ 0 (stiction) to ‖vₜ‖/vₛ > 1 (sliding). Since we
///    are in a region of "weak" gradients, we limit the update to
///    vₜᵏ⁺¹ = vₜᵏ/‖vₜᵏ‖⋅vₛ/2. i.e. we make it fall within the stiction
///    region of strong gradients. We do allow however transition to sliding
///    from the stiction region (presumably in the next iteration for large
///    enough forcing).
///  - Transition from sliding ‖vₜᵏ‖/vₛ > 1 to an almost perfect stiction with
///    ‖vₜᵏ⁺¹‖ < εᵥ. In an attempt to avoid weak gradients for the next
///    iteration, we impose the limit vₜᵏ⁺¹ = vₜᵏ/‖vₜᵏ‖⋅vₛ/2, placing the
///    velocity "in the same direction where it came from", within the stiction
///    region, but where gradients are strong.
///  - Velocity change Δvₜᵏ intersects the stiction circle. This situation
///    implies that most likely a stiction transition could happen but the pure
///    Newton-Raphson would miss it. This situation is outlined in
///    [Uchida et al., 2015]. In this case LimitDirectionChange computes α so
///    that vₜᵏ⁺¹ =  vₜᵏ + αΔvₜᵏ is the closest vector to the origin. This
///    corresponds to the geometric condition dot(vₜᵏ⁺¹, Δvₜᵏ) = 0.
///  - Velocity change Δvₜᵏ does not intersect the stiction circle, i.e.
///    changes happen in a region away from stiction (within the sliding
///    region). However, large angular changes (measured by the angle
///    `θ = acos(vₜᵏ⁺¹⋅vₜᵏ/(‖vₜᵏ⁺¹‖‖vₜᵏ‖))` between `vₜᵏ⁺¹` and `vₜᵏ`)
///    might indicate a solution that is attempting to reach a stiction region.
///    In order to aid convergence, we limit the angle change to θₘₐₓ, and
///    therefore (see [Uchida et al., 2015]) we compute α so that
///    `θₘₐₓ = acos(vₜᵏ⁺¹⋅vₜᵏ/(‖vₜᵏ⁺¹‖‖vₜᵏ‖))`.
///
/// Uchida, T.K., Sherman, M.A. and Delp, S.L., 2015.
///   Making a meaningful impact: modelling simultaneous frictional collisions
///   in spatial multibody systems. Proc. R. Soc. A, 471(2177), p.20140859.
///
/// %LimitDirectionChange implements the algorithm described above. We place it
/// inside a struct so that we can use Eigen::Ref arguments allowing different
/// scalar types T.
template <typename T>
struct DirectionChangeLimiter {
  /// Implements the limiting algorithm described in the documentation above.
  /// @param[in] v the k-th iteration tangential velocity vₜᵏ, in m/s.
  /// @param[in] dv the k-th iteration tangential velocity update Δvₜᵏ, in m/s.
  /// @param[in] cos_theta_max precomputed value of cos(θₘₐₓ).
  /// @param[in] v_stiction the stiction tolerance vₛ, in m/s.
  /// @param[in] tolerance a value << 1 used to determine when ‖vₜ‖ ≈ 0.
  /// @retval α the limit in [0, 1] so that vₜᵏ⁺¹ = vₜᵏ + αΔvₜᵏ.
  static T CalcAlpha(const Eigen::Ref<const Vector2<T>>& v,
                     const Eigen::Ref<const Vector2<T>>& dv,
                     double cos_theta_max, double v_stiction, double tolerance);
};
}  // namespace internal

}  // namespace implicit_stribeck
}  // namespace multibody
}  // namespace drake
