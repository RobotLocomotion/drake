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
/// Newton-Raphson iteration, and at each kth iteration, it computes a
/// tangential velocity update Δvₜᵏ. One Newton strategy would be to compute
/// the tangential velocity at the next iteration (k+1) as
/// vₜᵏ⁺¹ = vₜᵏ + αΔvₜᵏ, where 0 < α < 1, is a coefficient obtained by line
/// search to improve convergence.
/// Line search works very well for smooth problems. However, even though the
/// implicit Stribeck is solving the root of a continuous function, this
/// function has very steep gradients only within the very small regions close
/// to where the tangential velocities are zero. These regions are circles in
/// ℝ² of radius equal to the stiction tolerance of the solver vₛ. We refer to
/// these circular regions as the "stiction regions" and to their boundaries as
/// the "stiction circles". We refer to the region of ℝ² outside the stiction
/// region around the origin as the "sliding region".
/// The implicit Stribeck solver uses the following modified Stribeck function
/// describing the functional dependence of the Stribeck coefficient of friction
/// μₛ with slip speed: <pre>
///     μₛ(x) = ⌈ μ⋅x⋅(2.0 - x),  x  < 1
///             ⌊ μ            ,  x >= 1
/// </pre>
/// where x corresponds to the dimensionless slip speed x = ‖vₜ‖ / vₛ and
/// μ is the Coulomb's law coefficient of friction. The implicit Stribeck solver
/// makes no distinction between static and dynamic coefficients of friction and
/// therefore a single coefficient μ needs to be specified.
/// The Stribeck function is highly nonlinear and difficult to solve with a
/// conventional Newton-Raphson method. However, it can be partitioned into
/// regions based on how well the local gradients can be used to find a
/// solution. We'll describe the algorithm below in terms of "strong" gradients
/// (∂μ/∂v >> 0) and "weak" gradients (∂μ/∂v ≈ 0). Roughly, the gradients are
/// strong during stiction and weak during sliding.
/// These regions are so small compared to the velocity scales dealt with
/// by the implicit Stribeck solver, that effectively, the Newton-Raphson
/// iterate would only "see" a fixed dynamic coefficient of friction and it
/// would never be able to predict stiction. That is, if search direction Δvₜᵏ
/// computed by the Newton-Raphson algorithm is not limited in some way, the
/// iteration would never fall within the stiction regions where gradients
/// are "strong" to guide the convergence of the solution, to either stiction
/// or sliding.
///
/// The remedy to this situation is to limit changes in the tangential
/// velocities at each iteration. The situation described above, in which an
/// update  Δvₜᵏ "misses" the stiction circle can be described in purely
/// geometric terms. We exploit this fact to devise a strategy that is
/// appropriate for this particular problem. We use the methodology outlined in
/// [Uchida et al., 2015] and describe particulars to our implementation below.
///
/// LimitDirectionChange implements a specific strategy with knowledge of the
/// implicit Stribeck iteration procedure. It is important to note that the
/// implicit Stribeck uses "soft norms" to avoid divisions by zero. That is,
/// friction forces are computed according to: <pre>
///   fₜ(vₜ) = -μ(‖vₜ‖ₛ) vₜ/‖vₜ‖ₛ
/// </pre>
/// where, to avoid the singularity at zero velocity, we use a "soft norm"
/// ‖vₜ‖ₛ = sqrt(vₜᵀvₜ + εᵥ²), with εᵥ a small fraction of vₛ. Due to the
/// use of soft norms, the gradient of fₜ with vₜ is now well defined,
/// but it goes to zero as vₜ approaches the origin. Therefore, gradients
/// are also "weak" in the neighborhood of ‖vₜ‖ₛ ≲ εᵥ.
/// Due to this, external forcing (either from applied forces or from coupling
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
///  - LimitDirectionChange first deals with the case ‖vₜ‖ < εᵥ to avoid
///    divisions by zero in the subsequent cases. It essentially clips vₜᵏ⁺¹
///    to have magnitude vₛ/2 when the update Δvₜᵏ ≠ 0. For small updates
///    Δvₜᵏ leading to vₜᵏ⁺¹ within the stiction region, we take  α = 1.
///    See implementation notes for CalcAlpha() for further details.
///  - Transition from ‖vₜ‖ < εᵥ (stiction) to ‖vₜ‖/vₛ > 1 (sliding). Since
///    we are in a region of "weak" gradients (due to "norm softening",
///    see discussion above), we limit the update to vₜᵏ⁺¹ = vₜᵏ/‖vₜᵏ‖⋅vₛ/2.
///    In other words, if the speed would grow too fast, we cap it at vₛ/2
///    so that at least two Newton iterations are required to go from near-0
///    sticking to sliding.
///  - Transition from sliding ‖vₜᵏ‖/vₛ > 1 to an almost perfect stiction with
///    ‖vₜᵏ⁺¹‖ < εᵥ. In an attempt to avoid weak gradients for the next
///    iteration, we impose the limit vₜᵏ⁺¹ = vₜᵏ/‖vₜᵏ‖⋅vₛ/2, placing the
///    velocity "in the same direction where it came from", within the stiction
///    region, but where gradients are strong.
///  - Velocity change Δvₜᵏ intersects the stiction circle. To be more precise,
///    the line connecting vₜᵏ and vₜᵏ + Δvₜᵏ crosses the stiction region.
///    This situation implies that most likely a stiction transition could
///    happen but the pure Newton-Raphson would miss it. This situation is
///    outlined in [Uchida et al., 2015]. In this case LimitDirectionChange
///    computes α so that vₜᵏ⁺¹ =  vₜᵏ + αΔvₜᵏ is the closest vector to the
///    origin. This corresponds to the geometric condition
///    dot(vₜᵏ⁺¹, Δvₜᵏ) = 0.
///  - Velocity change Δvₜᵏ does not intersect the stiction circle, i.e.
///    changes happen in a region away from stiction (within the sliding
///    region). However, large angular changes (measured by the angle
///    θ = acos(vₜᵏ⁺¹⋅vₜᵏ/(‖vₜᵏ⁺¹‖‖vₜᵏ‖)) between vₜᵏ⁺¹ and vₜᵏ)
///    might indicate a solution that is attempting to reach a stiction region.
///    In order to aid convergence, we limit the angle change to θₘₐₓ, and
///    therefore (see [Uchida et al., 2015]) we compute α so that
///    θₘₐₓ = acos(vₜᵏ⁺¹⋅vₜᵏ/(‖vₜᵏ⁺¹‖‖vₜᵏ‖)).
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
  /// @param[in] relative_tolerance a value << 1 used to determine when
  /// ‖vₜ‖ ≈ 0. Typical values lie withing the 10⁻³ - 10⁻² range. This allows
  /// us to compute `εᵥ = tolerance⋅vₛ` (in m/s) which defines a "small
  /// tangential velocity scale". This value is used to compute "soft norms"
  /// (see class's documentation) and to detect values close to
  /// zero, ‖vₜ‖ < εᵥ. A value close to one could cause the solver to miss
  /// transitions from/to stiction.
  /// @retval α the limit in [0, 1] so that vₜᵏ⁺¹ = vₜᵏ + αΔvₜᵏ.
  static T CalcAlpha(const Eigen::Ref<const Vector2<T>>& v,
                     const Eigen::Ref<const Vector2<T>>& dv,
                     double cos_theta_max, double v_stiction,
                     double relative_tolerance);

  /// Helper method for detecting when the line connecting v with v1 = v + dv
  /// crosses the stiction region, a circle of radius `v_stiction`.
  /// All other input arguments are quantities already precomputed by
  /// CalcAlpha() and thus we reuse them.
  /// @param alpha when this method returns `true` (zero crossing), a
  /// coefficient in `(0, 1]` so that `v_alpha = v + alpha * dv` is the closest
  /// vector to the origin. It is not set when the method returns `false`.
  /// @returns `true` if the line connecting v with v1 = v + dv crosses the
  /// stiction region.
  static bool CrossesTheStictionRegion(
      const Eigen::Ref<const Vector2<T>>& v,
      const Eigen::Ref<const Vector2<T>>& dv,
      const T& v_dot_dv, const T& dv_norm, const T& dv_norm2,
      double epsilon_v, double v_stiction, T* alpha);

  /// Helper method to solve the quadratic equation aα² + bα + c = 0 for the
  /// very particular case we know we have real roots (Δ = b² - 4ac > 0) and we
  /// are interested in the smallest positive root.
  static T SolveQuadraticForTheSmallestPositiveRoot(
      const T& a, const T& b, const T& c);
};
}  // namespace internal

}  // namespace implicit_stribeck
}  // namespace multibody
}  // namespace drake
