#pragma once

#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
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
///     μₛ(x) = ⌈ μ x (2 - x),  x  < 1
///             ⌊ μ          ,  x >= 1
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
///
/// - LimitDirectionChange first deals with the case ‖vₜ‖ < εᵥ to avoid
///   divisions by zero in the subsequent cases. It essentially clips vₜᵏ⁺¹
///   to have magnitude vₛ/2 when the update Δvₜᵏ ≠ 0. For small updates
///   Δvₜᵏ leading to vₜᵏ⁺¹ within the stiction region, we take  α = 1.
///   See implementation notes for CalcAlpha() for further details.
/// - Transition from ‖vₜ‖ < εᵥ (stiction) to ‖vₜ‖/vₛ > 1 (sliding). Since
///   we are in a region of "weak" gradients (due to "norm softening",
///   see discussion above), we limit the update to vₜᵏ⁺¹ = vₜᵏ/‖vₜᵏ‖⋅vₛ/2.
///   In other words, if the speed would grow too fast, we cap it at vₛ/2
///   so that at least two Newton iterations are required to go from near-0
///   sticking to sliding.
/// - Transition from sliding ‖vₜᵏ‖/vₛ > 1 to an almost perfect stiction with
///   ‖vₜᵏ⁺¹‖ < εᵥ. In an attempt to avoid weak gradients for the next
///   iteration, we impose the limit vₜᵏ⁺¹ = vₜᵏ/‖vₜᵏ‖⋅vₛ/2, placing the
///   velocity "in the same direction where it came from", within the stiction
///   region, but where gradients are strong.
/// - Velocity change Δvₜᵏ intersects the stiction circle. To be more precise,
///   the line connecting vₜᵏ and vₜᵏ + Δvₜᵏ crosses the stiction region.
///   This situation implies that most likely a stiction transition could
///   happen but the pure Newton-Raphson would miss it. This situation is
///   outlined in [Uchida et al., 2015]. In this case LimitDirectionChange
///   computes α so that vₜᵏ⁺¹ =  vₜᵏ + αΔvₜᵏ is the closest vector to the
///   origin. This corresponds to the geometric condition
///   dot(vₜᵏ⁺¹, Δvₜᵏ) = 0.
/// - Velocity change Δvₜᵏ does not intersect the stiction circle, i.e.
///   changes happen in a region away from stiction (within the sliding
///   region). However, large angular changes (measured by the angle
///   θ = acos(vₜᵏ⁺¹⋅vₜᵏ/(‖vₜᵏ⁺¹‖‖vₜᵏ‖)) between vₜᵏ⁺¹ and vₜᵏ)
///   might indicate a solution that is attempting to reach a stiction region.
///   In order to aid convergence, we limit the angle change to θₘₐₓ, and
///   therefore (see [Uchida et al., 2015]) we compute α so that
///   θₘₐₓ = acos(vₜᵏ⁺¹⋅vₜᵏ/(‖vₜᵏ⁺¹‖‖vₜᵏ‖)).
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
  /// ‖vₜ‖ ≈ 0. Typical values lie within the 10⁻³ - 10⁻² range. This allows
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

/// The result from ImplicitStribeckSolver::SolveWithGuess() used to report the
/// success or failure of the solver.
enum class ImplicitStribeckSolverResult {
  /// Successful computation.
  kSuccess = 0,

  /// The maximum number of iterations was reached.
  kMaxIterationsReached = 1,

  /// The linear solver used within the Newton-Raphson loop failed.
  /// This might be caused by a divergent iteration that led to an invalid
  /// Jacobian matrix.
  kLinearSolverFailed = 2
};

/// These are the parameters controlling the iteration process of the
/// ImplicitStribeckSolver solver.
struct ImplicitStribeckSolverParameters {
  /// The stiction tolerance vₛ for the slip velocity in the Stribeck
  /// function, in m/s. Roughly, for an externally applied tangential forcing
  /// fₜ and normal force fₙ, under "stiction", the slip velocity will be
  /// approximately vₜ ≈ vₛ fₜ/(μfₙ). In other words, the maximum slip
  /// error of the Stribeck approximation occurs at the edge of the friction
  /// cone when fₜ = μfₙ and vₜ = vₛ.
  /// The default of 0.1 mm/s is a very tight value that for most problems of
  /// interest in robotics will result in simulation results with negligible
  /// slip velocities introduced by the Stribeck approximation when in stiction.
  double stiction_tolerance{1.0e-4};  // 0.1 mm/s

  /// The maximum number of iterations allowed for the Newton-Raphson
  /// iterative solver.
  int max_iterations{100};

  /// The tolerance to monitor the convergence of the tangential velocities.
  /// This number specifies a tolerance relative to the value of the
  /// stiction_tolerance and thus it is dimensionless. Using a tolerance
  /// relative to the value of the stiction_tolerance is necessary in order
  /// to capture transitions to stiction that would require an accuracy in the
  /// value of the tangential velocities smaller than that of the
  /// "Stribeck stiction region" (the circle around the origin with radius
  /// stiction_tolerance).
  /// A value close to one could cause the solver to miss transitions from/to
  /// stiction. Small values approaching zero will result in a higher number of
  /// iterations needed to attain the desired level of convergence.
  /// Typical values lie within the 10⁻³ - 10⁻² range.
  double relative_tolerance{1.0e-2};

  /// (Advanced) ImplicitStribeckSolver limits large angular changes between
  /// tangential velocities at two successive iterations vₜᵏ⁺¹ and vₜᵏ. This
  /// change is measured by the angle θ = acos(vₜᵏ⁺¹⋅vₜᵏ/(‖vₜᵏ⁺¹‖‖vₜᵏ‖)).
  /// To aid convergence, ImplicitStribeckSolver, limits this angular change to
  /// `theta_max`. Please refer to the documentation for ImplicitStribeckSolver
  /// for further details.
  ///
  /// Small values of `theta_max` will result in a larger number of iterations
  /// of the solver for situations in which large angular changes occur (sudden
  /// transients or impacts). Values of `theta_max` close to π/2 allow for a
  /// faster convergence for problems with sudden transitions to/from stiction.
  /// Large values of `theta_max` however might lead to non-convergence of the
  /// solver. We choose a conservative number by default that we found to work
  /// well in most practical problems of interest.
  double theta_max{M_PI / 3.0};
};

/// Struct used to store information about the iteration process performed by
/// ImplicitStribeckSolver.
struct ImplicitStribeckSolverIterationStats {
  /// (Internal) Used by ImplicitStribeckSolver to reset statistics.
  void Reset() {
    num_iterations = 0;
    // Clear does not change a std::vector "capacity", and therefore there's
    // no reallocation (or deallocation) that could affect performance.
    residuals.clear();
  }

  /// (Internal) Used by ImplicitStribeckSolver to update statistics.
  void Update(double iteration_residual) {
    ++num_iterations;
    residuals.push_back(iteration_residual);
  }

  /// The number of iterations performed by the last ImplicitStribeckSolver
  /// solve.
  int num_iterations{0};

  /// Returns the residual in the tangential velocities, in m/s. Upon
  /// convergence of the solver this value should be smaller than
  /// Parameters::tolerance times Parameters::stiction_tolerance.
  double vt_residual() const { return residuals.back();}

  /// (Advanced) Residual in the tangential velocities, in m/s. The k-th entry
  /// in this vector corresponds to the residual for the k-th Newton-Raphson
  /// iteration performed by the solver.
  /// After ImplicitStribeckSolver solved a problem, this vector will have size
  /// num_iterations.
  /// The last entry in this vector, `residuals[num_iterations-1]`, corresponds
  /// to the residual upon completion of the solver, i.e. vt_residual.
  std::vector<double> residuals;
};

/** @anchor implicit_stribeck_class_intro
%ImplicitStribeckSolver solves the equations below for mechanical systems
with contact using a modified Stribeck model of friction:
@verbatim
            q̇ = N(q) v
  (1)  M(q) v̇ = τ + Jₙᵀ(q) fₙ(q, v) + Jₜᵀ(q) fₜ(q, v)
@endverbatim
where `v ∈ ℝⁿᵛ` is the vector of generalized velocities, `M(q) ∈ ℝⁿᵛˣⁿᵛ` is
the mass matrix, `Jₙ(q) ∈ ℝⁿᶜˣⁿᵛ` is the Jacobian of normal separation
velocities, `Jₜ(q) ∈ ℝ²ⁿᶜˣⁿᵛ` is the Jacobian of tangent velocities,
`fₙ ∈ ℝⁿᶜ` is the vector of normal contact forces, `fₜ ∈ ℝ²ⁿᶜ` is the
vector of tangent friction forces and τ ∈ ℝⁿᵛ is a vector of generalized
forces containing all other applied forces (e.g., Coriolis, gyroscopic
terms, actuator forces, etc.) but contact forces.
This solver assumes a compliant law for the normal forces `fₙ(q, v)` and
therefore the functional dependence of `fₙ(q, v)` with q and v is stated
explicitly.

Since %ImplicitStribeckSolver uses a modified Stribeck model for friction,
we explicitly emphasize the functional dependence of `fₜ(q, v)` with the
generalized velocities. The functional dependence of `fₜ(q, v)` with the
generalized positions stems from its direct dependence with the normal
forces `fₙ(q, v)`.

%ImplicitStribeckSolver implements two different schemes. A "one-way
coupling scheme" which solves for the friction forces given the normal
forces are known. That is, normal forces affect the computation of the
friction forces however, the normal forces are kept constant during the
solution procedure.

A "two-way coupling scheme" treats both the normal and friction forces
implicitly in the generalized velocities resulting in a numerical strategy
in which normal forces affect friction forces and, conversely, friction forces
couple back to the computation of the normal forces.

The two-way coupled scheme provides a more stable and therefore robust
solution to the problem stated in Eq. (1) with just a small increase of the
computational cost compared to the one-way coupled scheme. The one-way
coupled scheme is however very useful for testing and analysis of the
solver.

@anchor one_way_coupling_scheme
<h2>One-Way Coupling Scheme</h2>

Equation (1) is discretized in time using a variation of the first order
semi-implicit Euler scheme from step s to step s+1 with time step `δt` as:
@verbatim
             qˢ⁺¹ = qˢ + δt N(qˢ) vˢ⁺¹
  (2)  M(qˢ) vˢ⁺¹ =
         M(qˢ) vˢ + δt [τˢ + Jₙᵀ(qˢ) fₙ(qˢ,vˢ) + Jₜᵀ(qˢ) fₜ(qˢ,vˢ⁺¹)]
@endverbatim
(We are using s for step counter rather than n to avoid Unicode-induced
confusion with the "normal direction" subscript n.)

Please see details in the @ref one_way_coupling_derivation
"Derivation of the one-way coupling scheme" section.
The equation for the generalized velocities in Eq. (2) is rewritten as:
@verbatim
  (3)  M vˢ⁺¹ = p* + δt [Jₙᵀ fₙ + Jₜᵀ fₜ(vˢ⁺¹)]
@endverbatim
where `p* = M vˢ + δt τˢ` is the generalized momentum that the
system would have in the absence of contact forces and, for simplicity, we
have only kept the functional dependencies in generalized velocities. Notice
that %ImplicitStribeckSolver uses a precomputed value of the normal forces.
These normal forces could be available for instance if
using a compliant contact approach, for which normal forces are a function
of the state.

@anchor two_way_coupling_scheme
<h2>Two-Way Coupling Scheme</h2>

Equation (1) is discretized in time using a variation on the
semi-implicit Euler scheme with time step `δt` as:
@verbatim
             qˢ⁺¹ = qˢ + δt N(qˢ) vˢ⁺¹
  (4)  M(qˢ) vˢ⁺¹ = M(qˢ) vˢ +
          δt [τˢ + Jₙᵀ(qˢ) fₙˢ⁺¹ + Jₜᵀ(qˢ) fₜ(fₙˢ⁺¹,vₜˢ⁺¹)]
@endverbatim
This implicit scheme variation evaluates Jacobian matrices Jₙ and Jₜ as
well as the kinematic mapping N(q) at the previous time step. In Eq. (4) we
have fₙˢ⁺¹ = fₙ(xˢ⁺¹, vₙˢ⁺¹) with xˢ⁺¹ = x(qˢ⁺¹), the signed _penetration_
distance (defined positive when bodies overlap) between contact point pairs
and the _separation_ velocities vₙˢ⁺¹ = Jₙ(qˢ) vˢ⁺¹ (= −ẋˢ⁺¹). Also the
functional dependence of fₜ with fₙ and vₜ is highlighted in Eq. (4). More
precisely, the two-way coupling scheme uses a normal force law for each contact
pair of the form:
@verbatim
  (5)  fₙ(x, vₙ) = k(vₙ)₊ x₊
  (6)      k(vₙ) = k (1 − d vₙ)₊
@endverbatim
where `x₊` is the positive part of x (x₊ = x if x ≥ 0 and x₊ = 0 otherwise)
and `k` and d are the stiffness and dissipation coefficients for a given contact
point, respectively.

The two-way coupling scheme uses a first order approximation to the signed
distance functions vector:
@verbatim
  (7)  xˢ⁺¹ ≈ xˢ − δt vₙˢ⁺¹ =  xˢ − δt Jₙ(qˢ) vˢ⁺¹
@endverbatim
where the minus sign is needed given that ẋ = dx/dt = −vₙ.
This approximation is used in Eq. (5) to obtain a numerical scheme that
implicitly couples normal forces through their functional dependence on the
signed penetration distance. Notice that, according to Eq. (5), normal forces
at each contact point are decoupled from each other. However their values are
coupled given the choice of a common variable, the generalized velocity v.

Equation (7) is used into Eq. (5) to obtain an expression of the normal
force in terms of the generalized velocity vˢ⁺¹ at the next time step:
@verbatim
  (8) fₙ(xˢ⁺¹, vₙˢ⁺¹) = k (1 − d vₙˢ⁺¹)₊ xˢ⁺¹₊
                      = k (1 − d Jₙ(qˢ) vˢ⁺¹)₊ (xˢ − δt Jₙ(qˢ) vˢ⁺¹)₊
                      = fₙ(vˢ⁺¹)
@endverbatim
Similarly, the friction forces fₜ can be written in terms of the next time
step generalized velocities using vₜˢ⁺¹ = Jₜ(qˢ) vˢ⁺¹ and using Eq. (8)
to substitute an expression for the normal force in terms of vˢ⁺¹. This
allows to re-write the tangential forces as a function of the generalized
velocities as:
@verbatim
  (9)  fₜ(fₙˢ⁺¹, vₜˢ⁺¹) = fₜ(fₙ(x(vˢ⁺¹), vₙ(vˢ⁺¹)), vₜ((vˢ⁺¹)))
                        = fₜ(vˢ⁺¹)
@endverbatim
where we write x(vˢ⁺¹) = xˢ − δt Jₙ(qˢ) vˢ⁺¹.
Finally, Eqs. (8) and (9) are used into Eq. (4) to obtain an expression in
vˢ⁺¹:
@verbatim
  (10)  M(qˢ) vˢ⁺¹ = p* + δt [Jₙᵀ(qˢ) fₙ(vˢ⁺¹) + Jₜᵀ(qˢ) fₜ(vˢ⁺¹)]
@endverbatim
with p* = `p* = M vˢ + δt τˢ` the generalized momentum that the system
would have in the absence of contact forces.

%ImplicitStribeckSolver uses a Newton-Raphson strategy to solve Eq. (10) in
the generalized velocities, limiting the iteration update with the scheme
described in @ref iteration_limiter.

@anchor iteration_limiter
<h2>Limits in the Iteration Updates</h2>

%ImplicitStribeckSolver solves for the generalized velocity at the next time
step `vˢ⁺¹` with either a one-way or two-way coupled scheme as described in the
 previous sections.
The solver uses a Newton-Raphson iteration to compute an update `Δvᵏ` at the
k-th Newton-Raphson iteration. Once `Δvᵏ` is computed, the solver limits the
change in the tangential velocities `Δvₜᵏ = Jₜᵀ Δvᵏ` using the approach
described in [Uchida et al., 2015]. This approach limits the maximum angle
change θ between two successive iterations in the tangential velocity.

Uchida, T.K., Sherman, M.A. and Delp, S.L., 2015.
  Making a meaningful impact: modelling simultaneous frictional collisions
  in spatial multibody systems. Proc. R. Soc. A, 471(2177), p.20140859.

@anchor one_way_coupling_derivation
<h2>Derivation of the one-way coupling scheme</h2>
In this section we provide a detailed derivation of the first order time
stepping approach in Eq. (2). We start from the continuous Eq. (1):
@verbatim
  (1)  M(q) v̇ = τ + Jₙᵀ(q) fₙ(q,v) + Jₜᵀ(q) fₜ(q,v)
@endverbatim
we can discretize Eq. (1) in time using a first order semi-implicit Euler
scheme in velocities:
@verbatim
  (11)  M(qˢ) vˢ⁺¹ = M(qˢ) vˢ +
          δt [τˢ⁺¹ + Jₙᵀ(qˢ) fₙ(qˢ,vˢ⁺¹) + Jₜᵀ(qˢ) fₜ(vˢ⁺¹)] + O₁(δt²)
@endverbatim
where the equality holds strictly since we included the leading terms in
`O(δt²)`. We use `τˢ⁺¹ = τ(tˢ, qˢ, vˢ⁺¹)` for brevity in Eq. (11).
When moving from the continuous Eq. (1) to the discrete version Eq. (11), we
lost the nice property that our compliant normal forces are decoupled from
the friction forces (both depend on the same unknown vˢ⁺¹ in Eq (11)). The
reason is that Eq. (11) includes an integration over a small interval of
size δt. To solve the discrete system in Eq. (11), we'd like to decouple the
normal forces from the tangential forces again, which will require a new
(though still valid) approximation.
To do so we will expand in Taylor series the term `fₙ(qˢ, vˢ⁺¹)`:
@verbatim
  (12)  fₙ(qˢ, vˢ⁺¹) = fₙ(qˢ,vˢ) + ∇ᵥfₙ(qˢ,vˢ) (vˢ⁺¹ − vˢ) + O₂(‖vˢ⁺¹ − vˢ‖²)
@endverbatim
The difference between `vˢ` and `vˢ⁺¹` can be written as:
@verbatim
  (13)  vˢ⁺¹ − vˢ = δtv̇ˢ + δtO₃(δt²) = O₄(δt)
@endverbatim
Substituting `vˢ⁺¹ − vˢ` from Eq. (13) into Eq. (12) we arrive to:
@verbatim
  (14)  fₙ(qˢ, vˢ⁺¹) = fₙ(qˢ,vˢ) + ∇ᵥfₙ(qˢ,vˢ) O₄(δt) + O₅(δt²)
                    = fₙ(qˢ,vˢ) + O₆(δt)
@endverbatim
where `O₅(δt²) = O₂(‖vˢ⁺¹ − vˢ‖²) = O₂(‖O₄(δt)‖²)`. A similar argument for
τˢ⁺¹ shows it also differs in O(δt) from τˢ = τ(tˢ, qˢ, vˢ).
We can now use Eq. (14) into Eq. (11) to arrive to:
@verbatim
  (15)  M(qˢ) vˢ⁺¹ = M(qˢ) vˢ +
        δt [τˢ + Jₙᵀ(qˢ) (fₙ(qˢ,vˢ) + O₆(δt)] + Jₜᵀ(qˢ) fₜ(vˢ⁺¹)) +
        O₁(δt²)
@endverbatim
which we can rewrite as:
@verbatim
  (16)  M(qˢ) vˢ⁺¹ = M(qˢ) vˢ +
      δt [τˢ + Jₙᵀ(qˢ) fₙ(qˢ, vˢ) + Jₜᵀ(qˢ) fₜ(vˢ⁺¹)] + O₇(δt²)
@endverbatim
with `O₇(δt²) = δt Jₙᵀ(qˢ) O₆(δt) + O₁(δt²)`.
That is, Eq. (16) introduces the same order of approximation as in the
semi-implicit method in Eq. (11).
Up to this point we have made no approximations but we instead propagated
the `O(⋅)` leading terms. Therefore the equalities in the equations above
are exact. To obtain an approximate time stepping scheme, we drop `O₇(δt²)`
(we neglect it) in Eq. (16) to arrive to a first order scheme:
@verbatim
  (17)  M(qˢ) vˢ⁺¹ = M(qˢ) vˢ +
                     δt [τˢ + Jₙᵀ(qˢ) fₙ(qˢ,vˢ) + Jₜᵀ(qˢ) fₜ(vˢ⁺¹)]
@endverbatim
Therefore, with the scheme in Eq. (17) we are able to decouple the
computation of (compliant) normal forces from that of friction forces.
A very important feature of this scheme however, is the explicit nature (in
the velocities v) of the term associated with the normal forces (usually
including dissipation in the normal direction), which will become unstable
for a sufficiently large time step. However, for most applications in
practice, the stability of the scheme is mostly determined by the explicit
update of normal forces with positions, that is, Eq. (17) is explicit in
positions through the normal forces `fₙ(qˢ, vˢ)`. For many common
applications, the explicit dependence of `τˢ(tˢ, qˢ, vˢ)` on the
previous time step velocities `vˢ` determines the overall stability of
the scheme, since this term can include velocity dependent contributions
such as control forces and dampers. Notice that Eq. (12) introduces an
expansion of `fₙ` with an order of approximation consistent with the
first order scheme as needed. Therefore, it propagates into a `O(δt²)`
term exactly as needed in Eq. (16).

@tparam T The type of mathematical object being added.
Instantiated templates for the following kinds of T's are provided:
- double
- AutoDiffXd

They are already available to link against in the containing library.
No other values for T are currently supported. */
template <typename T>
class ImplicitStribeckSolver {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ImplicitStribeckSolver)

  /// Instantiates a solver for a problem with `nv` generalized velocities.
  /// @throws std::exception if nv is non-positive.
  explicit ImplicitStribeckSolver(int nv);

  // TODO(amcastro-tri): submit a separate reformat PR changing /// by /**.
  /// Sets data for the problem to be solved as outlined by Eq. (3) in this
  /// class's documentation: <pre>
  ///   (3)  M v = p* + δt Jₙᵀ fₙ +  δt Jₜᵀ fₜ(v)
  /// </pre>
  /// Refer to this class's documentation for further details on the structure
  /// of the problem and the solution strategy.
  /// In the documented parameters below, `nv` is the number of generalized
  /// velocities and `nc` is the number of contact points.
  ///
  /// @param[in] M
  ///   The mass matrix of the system, of size `nv x nv`.
  /// @param[in] Jn
  ///   The normal separation velocities Jacobian, of size `nc x nv`.
  /// @param[in] Jt
  ///   The tangential velocities Jacobian, of size `2nc x nv`.
  /// @param[in] p_star
  ///   The generalized momentum the system would have at `s + 1` if contact
  ///   forces were zero.
  /// @param[in] fn
  ///   A vector of size `nc` containing the normal force at each contact point.
  /// @param[in] mu
  ///   A vector of size `nc` containing the friction coefficient at each
  ///   contact point. The solver makes no distinction between static and
  ///   dynamic coefficients of friction or, similarly, the solver assumes the
  ///   static and dynamic coefficients of friction are the same.
  ///
  /// @warning This method stores constant references to the matrices and
  /// vectors passed as arguments. Therefore
  ///   1. they must outlive this class and,
  ///   2. changes to the problem data invalidate any solution performed by this
  ///      solver. In such a case, SetOneWayCoupledProblemData() and
  ///      SolveWithGuess() must be invoked again.
  ///
  /// @throws std::exception if any of the data pointers are nullptr.
  /// @throws std::exception if the problem data sizes are not consistent as
  /// described above.
  /// @throws std::exception if SetTwoWayCoupledProblemData() was ever called on
  /// `this` solver.
  void SetOneWayCoupledProblemData(
      EigenPtr<const MatrixX<T>> M,
      EigenPtr<const MatrixX<T>> Jn, EigenPtr<const MatrixX<T>> Jt,
      EigenPtr<const VectorX<T>> p_star,
      EigenPtr<const VectorX<T>> fn, EigenPtr<const VectorX<T>> mu);

  /// Sets the problem data to solve the problem outlined in Eq. (10) in this
  /// class's documentation using a two-way coupled approach: <pre>
  ///   (10)  M(qˢ) vˢ⁺¹ = p* + δt [Jₙᵀ(qˢ) fₙ(vˢ⁺¹) + Jₜᵀ(qˢ) fₜ(vˢ⁺¹)]
  /// </pre>
  /// Refer to this class's documentation for further details on the structure
  /// of the problem and the solution strategy.
  /// In the documented parameters below, `nv` is the number of generalized
  /// velocities and `nc` is the number of contact points.
  ///
  /// @param[in] M
  ///   The mass matrix of the system, of size `nv x nv`.
  /// @param[in] Jn
  ///   The normal separation velocities Jacobian, of size `nc x nv`.
  /// @param[in] Jt
  ///   The tangential velocities Jacobian, of size `2nc x nv`.
  /// @param[in] p_star
  ///   The generalized momentum the system would have at `n + 1` if contact
  ///   forces were zero.
  /// @param[in] x0
  ///   The signed penetration distance at the previous time step. It is defined
  ///   positive when bodies overlap.
  /// @param[in] stiffness
  ///   A vector of size `nc` storing at each ith entry the stiffness
  ///   coefficient for the ith contact pair.
  /// @param[in] dissipation
  ///   A vector of size `nc` storing at each ith entry the dissipation
  ///   coefficient for the ith contact pair.
  /// @param[in] mu
  ///   A vector of size `nc` containing the friction coefficient at each
  ///   contact point. The solver makes no distinction between static and
  ///   dynamic coefficients of friction or, similarly, the solver assumes the
  ///   static and dynamic coefficients of friction are the same.
  ///
  /// @warning This method stores constant references to the matrices and
  /// vectors passed as arguments. Therefore
  ///   1. they must outlive this class and,
  ///   2. changes to the problem data invalidate any solution performed by this
  ///      solver. In such a case, SetOneWayCoupledProblemData() and
  ///      SolveWithGuess() must be invoked again.
  ///
  /// @throws std::exception if any of the data pointers are nullptr.
  /// @throws std::exception if the problem data sizes are not consistent as
  /// described above.
  /// @throws std::exception if SetOneWayCoupledProblemData() was ever called on
  /// `this` solver.
  // TODO(amcastro-tri): rework the entire math again to make phi to actually be
  // the signed distance function (instead of the signed penetration distance).
  void SetTwoWayCoupledProblemData(
      EigenPtr<const MatrixX<T>> M, EigenPtr<const MatrixX<T>> Jn,
      EigenPtr<const MatrixX<T>> Jt, EigenPtr<const VectorX<T>> p_star,
      EigenPtr<const VectorX<T>> x0, EigenPtr<const VectorX<T>> stiffness,
      EigenPtr<const VectorX<T>> dissipation, EigenPtr<const VectorX<T>> mu);

  /// Given an initial guess `v_guess`, this method uses a Newton-Raphson
  /// iteration to find a solution for the generalized velocities satisfying
  /// either Eq. (3) when one-way coupling is used or Eq. (10) when two-way
  /// coupling is used. See this class's documentation for further details.
  /// To retrieve the solution, please refer to @ref retrieving_the_solution.
  /// @returns kSuccess if the iteration converges. All other values of
  /// ImplicitStribeckSolverResult report different failure modes.
  /// Uses `this` solver accessors to retrieve the last computed solution.
  /// @warning Always verify that the return value indicates success before
  /// retrieving the computed solution.
  ///
  /// @param[in] dt The time step used advance the solution in time.
  /// @param[in] v_guess The initial guess used in by the Newton-Raphson
  /// iteration. Typically, the previous time step velocities.
  ///
  /// @throws std::logic_error if `v_guess` is not of size `nv`, the number of
  /// generalized velocities specified at construction.
  ImplicitStribeckSolverResult SolveWithGuess(
      double dt, const VectorX<T>& v_guess) const;

  /// @anchor retrieving_the_solution
  /// @name Retrieving the solution
  /// This methods allow to retrieve the solution stored in the solver after
  /// the last call to SolveWithGuess().
  /// @{

  /// Returns a constant reference to the most recent  vector of generalized
  /// friction forces.
  const VectorX<T>& get_generalized_friction_forces() const {
    return fixed_size_workspace_.mutable_tau_f();
  }

  /// Returns a constant reference to the most recent solution vector for normal
  /// separation velocities. This method returns an `Eigen::VectorBlock`
  /// referencing a vector of size `nc`.
  Eigen::VectorBlock<const VectorX<T>> get_normal_velocities() const {
    return variable_size_workspace_.vn();
  }

  /// Returns a constant reference to the most recent vector of generalized
  /// contact forces, including both friction and normal forces.
  const VectorX<T>& get_generalized_contact_forces() const {
    return fixed_size_workspace_.mutable_tau();
  }

  /// Returns a constant reference to the most recent vector of tangential
  /// forces. This method returns an `Eigen::VectorBlock` referencing a vector
  /// of size `nc`.
  Eigen::VectorBlock<const VectorX<T>> get_tangential_velocities() const {
    return variable_size_workspace_.vt();
  }

  /// Returns a constant reference to the most recent vector of generalized
  /// velocities.
  const VectorX<T>& get_generalized_velocities() const {
    return fixed_size_workspace_.mutable_v();
  }

  /// Returns a constant reference to the most recent vector of (repulsive)
  /// forces in the normal direction. That is, the normal force is positive when
  /// the bodies push each other apart. Otherwise the normal force is zero,
  /// since contact forces can only be repulsive.
  Eigen::VectorBlock<const VectorX<T>> get_normal_forces() const {
    return variable_size_workspace_.fn();
  }

  /// Returns a constant reference to the most recent vector of friction forces.
  /// These friction forces are defined in accordance to the tangential
  /// velocities Jacobian Jₜ as documented in
  /// @ref implicit_stribeck_class_intro "this class's documentation".
  Eigen::VectorBlock<const VectorX<T>> get_friction_forces() const {
    return variable_size_workspace_.ft();
  }

  /// @}

  /// Returns statistics recorded during the last call to SolveWithGuess().
  /// See IterationStats for details.
  const ImplicitStribeckSolverIterationStats& get_iteration_statistics() const {
    return statistics_;
  }

  /// Returns the current set of parameters controlling the iteration process.
  /// See Parameters for details.
  const ImplicitStribeckSolverParameters& get_solver_parameters() const {
    return parameters_;
  }

  /// Sets the parameters to be used by the solver.
  /// See Parameters for details.
  void set_solver_parameters(
      const ImplicitStribeckSolverParameters& parameters) {
    // cos_theta_max must be updated consistently with the new value of
    // theta_max.
    cos_theta_max_ = std::cos(parameters.theta_max);
    parameters_ = parameters;
  }

 private:
  // Helper class for unit testing.
  friend class ImplicitStribeckSolverTester;

  // Contains all the references that define the problem to be solved.
  // These references must remain valid at least from the time they are set with
  // SetOneWayCoupledProblemData() and until SolveWithGuess() returns.
  class ProblemDataAliases {
   public:
    // Sets the references to the data defining a one-way coupled problem.
    // This method throws an exception if SetTwoWayCoupledData() was previously
    // called on this object.
    void SetOneWayCoupledData(
        EigenPtr<const MatrixX<T>> M,
        EigenPtr<const MatrixX<T>> Jn, EigenPtr<const MatrixX<T>> Jt,
        EigenPtr<const VectorX<T>> p_star,
        EigenPtr<const VectorX<T>> fn, EigenPtr<const VectorX<T>> mu) {
      DRAKE_DEMAND(M != nullptr);
      DRAKE_DEMAND(Jn != nullptr);
      DRAKE_DEMAND(Jt != nullptr);
      DRAKE_DEMAND(p_star != nullptr);
      DRAKE_DEMAND(fn != nullptr);
      DRAKE_DEMAND(mu != nullptr);
      DRAKE_THROW_UNLESS(coupling_scheme_ == kInvalidScheme ||
          coupling_scheme_ == kOneWayCoupled);
      coupling_scheme_ = kOneWayCoupled;
      M_ptr_ = M;
      Jn_ptr_ = Jn;
      Jt_ptr_ = Jt;
      p_star_ptr_ = p_star;
      fn_ptr_ = fn;
      mu_ptr_ = mu;
    }

    // Sets the references to the data defining a two-way coupled problem.
    // This method throws an exception if SetOneWayCoupledData() was previously
    // called on this object.
    void SetTwoWayCoupledData(
        EigenPtr<const MatrixX<T>> M,
        EigenPtr<const MatrixX<T>> Jn, EigenPtr<const MatrixX<T>> Jt,
        EigenPtr<const VectorX<T>> p_star,
        EigenPtr<const VectorX<T>> x0,
        EigenPtr<const VectorX<T>> stiffness,
        EigenPtr<const VectorX<T>> dissipation, EigenPtr<const VectorX<T>> mu) {
      DRAKE_DEMAND(M != nullptr);
      DRAKE_DEMAND(Jn != nullptr);
      DRAKE_DEMAND(Jt != nullptr);
      DRAKE_DEMAND(p_star != nullptr);
      DRAKE_DEMAND(x0 != nullptr);
      DRAKE_DEMAND(stiffness != nullptr);
      DRAKE_DEMAND(dissipation != nullptr);
      DRAKE_DEMAND(mu != nullptr);
      DRAKE_THROW_UNLESS(coupling_scheme_ == kInvalidScheme ||
          coupling_scheme_ == kTwoWayCoupled);
      coupling_scheme_ = kTwoWayCoupled;
      M_ptr_ = M;
      Jn_ptr_ = Jn;
      Jt_ptr_ = Jt;
      p_star_ptr_ = p_star;
      x0_ptr_ = x0;
      stiffness_ptr_ = stiffness;
      dissipation_ptr_ = dissipation;
      mu_ptr_ = mu;
    }

    // Returns true if this class contains the data for a two-way coupled
    // problem.
    bool has_two_way_coupling_data() const {
      return coupling_scheme_ == kTwoWayCoupled;
    }

    Eigen::Ref<const MatrixX<T>> M() const { return *M_ptr_; }
    Eigen::Ref<const MatrixX<T>> Jn() const { return *Jn_ptr_; }
    Eigen::Ref<const MatrixX<T>> Jt() const { return *Jt_ptr_; }
    Eigen::Ref<const VectorX<T>> p_star() const { return *p_star_ptr_; }

    // For the one-way coupled scheme, it returns a constant reference to the
    // data for the normal forces. It aborts if called on data for the two-way
    // coupled scheme, see has_two_way_coupling_data().
    Eigen::Ref<const VectorX<T>> fn() const {
      DRAKE_DEMAND(fn_ptr_ != nullptr);
      return *fn_ptr_;
    }

    // For the two-way coupled scheme, it returns a constant reference to the
    // data for the penetration distance. It aborts if called on data for the
    // one-way coupled scheme, see has_two_way_coupling_data().
    Eigen::Ref<const VectorX<T>> x0() const {
      DRAKE_DEMAND(x0_ptr_ != nullptr);
      return *x0_ptr_;
    }

    // For the two-way coupled scheme, it returns a constant reference to the
    // data for the stiffness. It aborts if called on data for the
    // one-way coupled scheme, see has_two_way_coupling_data().
    Eigen::Ref<const VectorX<T>> stiffness() const {
      DRAKE_DEMAND(stiffness_ptr_ != nullptr);
      return *stiffness_ptr_;
    }

    // For the two-way coupled scheme, it returns a constant reference to the
    // data for the dissipation. It aborts if called on data for the
    // one-way coupled scheme, see has_two_way_coupling_data().
    Eigen::Ref<const VectorX<T>> dissipation() const {
      DRAKE_DEMAND(dissipation_ptr_ != nullptr);
      return *dissipation_ptr_;
    }

    Eigen::Ref<const VectorX<T>> mu() const {
      return *mu_ptr_;
    }

   private:
    enum {
      kInvalidScheme,
      kOneWayCoupled,
      kTwoWayCoupled
    } coupling_scheme_{kInvalidScheme};

    // The mass matrix of the system.
    EigenPtr<const MatrixX<T>> M_ptr_{nullptr};
    // The normal separation velocities Jacobian.
    EigenPtr<const MatrixX<T>> Jn_ptr_{nullptr};
    // The tangential velocities Jacobian.
    EigenPtr<const MatrixX<T>> Jt_ptr_{nullptr};
    // The generalized momentum vector **before** contact is applied.
    EigenPtr<const VectorX<T>> p_star_ptr_{nullptr};
    // Normal force at each contact point. fn_ptr_ is nullptr for two-way
    // coupled problems.
    EigenPtr<const VectorX<T>> fn_ptr_{nullptr};
    // Penetration distance. Positive when there is penetration and negative
    // when there is separation, i.e. it is minus the signed distance function.
    // x0_ptr_ is nullptr for one-way coupled problems.
    EigenPtr<const VectorX<T>> x0_ptr_{nullptr};
    // Stiffness in the normal direction. nullptr for one-way coupled problems.
    EigenPtr<const VectorX<T>> stiffness_ptr_{nullptr};
    // Damping in the normal direction. nullptr for one-way coupled problems.
    EigenPtr<const VectorX<T>> dissipation_ptr_{nullptr};
    // Friction coefficient for each contact point.
    EigenPtr<const VectorX<T>> mu_ptr_{nullptr};
  };

  // The solver's workspace allocated at construction time. Sizes only depend on
  // nv, the number of generalized velocities.
  // The size of the variables in this workspace MUST remain fixed throughout
  // the lifetime of the solver. Do not resize any of them!.
  class FixedSizeWorkspace {
   public:
    // Constructs a workspace with size only dependent on nv.
    explicit FixedSizeWorkspace(int nv) : J_ldlt_(nv), J_lu_(nv) {
      J_ldlt_.setZero();
      v_.setZero(nv);
      residual_.setZero(nv);
      Delta_v_.setZero(nv);
      J_.setZero(nv, nv);
      tau_f_.setZero(nv);
      tau_.setZero(nv);
    }
    VectorX<T>& mutable_v() { return v_; }
    VectorX<T>& mutable_residual() { return residual_; }
    MatrixX<T>& mutable_J() { return J_; }
    VectorX<T>& mutable_Delta_v() { return Delta_v_; }
    VectorX<T>& mutable_tau_f() { return tau_f_; }
    VectorX<T>& mutable_tau() { return tau_; }
    Eigen::LDLT<MatrixX<T>>& mutable_J_ldlt() { return J_ldlt_; }
    Eigen::PartialPivLU<MatrixX<T>>& mutable_J_lu() { return J_lu_; }

   private:
    // Vector of generalized velocities.
    VectorX<T> v_;
    // Newton-Raphson residual.
    VectorX<T> residual_;
    // Newton-Raphson Jacobian, i.e. Jᵢⱼ = ∂Rᵢ/∂vⱼ.
    MatrixX<T> J_;
    // Solution to Newton-Raphson update, i.e. Δv = −J⁻¹ R.
    VectorX<T> Delta_v_;
    // Vector of generalized forces due to friction.
    VectorX<T> tau_f_;
    // Vector of generalized forces (normal + friction forces).
    VectorX<T> tau_;
    // LDLT Factorization of the Newton-Raphson Jacobian J. Only used for
    // one-way coupled problems with symmetric Jacobian.
    Eigen::LDLT<MatrixX<T>> J_ldlt_;
    // LU Factorization of the Newton-Raphson Jacobian J. Only used for
    // two-way coupled problems with non-symmetric Jacobian.
    Eigen::PartialPivLU<MatrixX<T>> J_lu_;
  };

  // The variables in this workspace can change size with each invocation of
  // SetOneWayCoupledProblemData() since the number of contact points nc can
  // change.
  // The workspace only performs re-allocations if needed, meaning that previous
  // storage is re-used if large enough for the next problem data set.
  // This class provides accessors that return Eigen blocks of size consistent
  // with the data currently stored, even if the (maximum) capacity is larger
  // than the data size.
  class VariableSizeWorkspace {
   public:
    explicit VariableSizeWorkspace(int initial_nc, int nv) {
      ResizeIfNeeded(initial_nc, nv);
    }

    // Performs a resize of this workspace's variables only if the new size `nc`
    // is larger than capacity() in order to reuse previously allocated space.
    void ResizeIfNeeded(int nc, int nv) {
      nc_ = nc;
      nv_ = nv;
      if (capacity() >= nc) return;  // no-op if not needed.
      const int nf = 2 * nc;
      // Only reallocate if sizes from previous allocations are not sufficient.
      vn_.resize(nc);
      vt_.resize(nf);
      fn_.resize(nc);
      ft_.resize(nf);
      x_.resize(nc);
      Delta_vn_.resize(nc);
      Delta_vt_.resize(nf);
      t_hat_.resize(nf);
      v_slip_.resize(nc);
      mus_.resize(nc);
      dft_dv_.resize(nc);
      Gn_.resize(nc, nv);
    }

    // Returns the current (maximum) capacity of the workspace.
    int capacity() const {
      return vt_.size();
    }

    // Returns a constant reference to the vector of separation velocities in
    // the normal direction, of size nc.
    Eigen::VectorBlock<const VectorX<T>> vn() const {
      return vn_.segment(0, nc_);
    }

    // Returns a mutable reference to the vector of separation velocities in
    // the normal direction, of size nc.
    Eigen::VectorBlock<VectorX<T>> mutable_vn() {
      return vn_.segment(0, nc_);
    }

    // Returns a constant reference to the vector containing the tangential
    // velocities vₜ for all contact points, of size 2nc.
    Eigen::VectorBlock<const VectorX<T>> vt() const {
      return vt_.segment(0, 2 * nc_);
    }

    // Mutable version of vt().
    Eigen::VectorBlock<VectorX<T>> mutable_vt() {
      return vt_.segment(0, 2 * nc_);
    }

    // Returns a mutable reference to the vector containing the normal
    // velocity updates Δvₙ for all contact points, of size nc.
    Eigen::VectorBlock<VectorX<T>> mutable_Delta_vn() {
      return Delta_vn_.segment(0, nc_);
    }

    // Returns a mutable reference to the vector containing the tangential
    // velocity updates Δvₜ for all contact points, of size 2nc.
    Eigen::VectorBlock<VectorX<T>> mutable_Delta_vt() {
      return Delta_vt_.segment(0, 2 * nc_);
    }

    // Returns a constant reference to the vector containing the normal
    // contact forces fₙ for all contact points, of size nc.
    Eigen::VectorBlock<const VectorX<T>> fn() const {
      return fn_.segment(0, nc_);
    }

    // Returns a mutable reference to the vector containing the normal
    // contact forces fₙ for all contact points, of size nc.
    Eigen::VectorBlock<VectorX<T>> mutable_fn() {
      return fn_.segment(0, nc_);
    }

    // Returns a mutable reference to the vector containing the penetration
    // depths for all contact points, of size nc.
    Eigen::VectorBlock<VectorX<T>> mutable_x() {
      return x_.segment(0, nc_);
    }

    // Returns a constant reference to the vector containing the tangential
    // friction forces fₜ for all contact points. fₜ has size 2nc since it
    // stores the two tangential components of the friction force for each
    // contact point. Refer to ImplicitStribeckSolver for details.
    Eigen::VectorBlock<const VectorX<T>> ft() const {
      return ft_.segment(0, 2 * nc_);
    }

    // Returns a mutable reference to the vector containing the tangential
    // friction forces fₜ for all contact points, of size 2nc.
    Eigen::VectorBlock<VectorX<T>> mutable_ft() {
      return ft_.segment(0, 2 * nc_);
    }

    // Returns a mutable reference to the vector containing the tangential
    // directions t̂ᵏ for all contact points, of size 2nc.
    Eigen::VectorBlock<VectorX<T>> mutable_t_hat() {
      return t_hat_.segment(0, 2 * nc_);
    }

    // Returns a mutable reference to the vector containing the slip velocity
    // vₛ = ‖vₜ‖, at each contact point, of size nc.
    Eigen::VectorBlock<VectorX<T>> mutable_v_slip() {
      return v_slip_.segment(0, nc_);
    }

    // Returns a mutable reference to the vector containing the stribeck
    // friction, function of the slip velocity, at each contact point, of
    // size nc.
    Eigen::VectorBlock<VectorX<T>> mutable_mu() {
      return mus_.segment(0, nc_);
    }

    // Returns a mutable reference to the gradient Gn = ∇ᵥfₙ(xˢ⁺¹, vₙˢ⁺¹)
    // with respect to the generalized velocites v.
    Eigen::Block<MatrixX<T>> mutable_Gn() {
      return Gn_.block(0, 0, nc_, nv_);
    }

    // Returns a mutable reference to the vector storing ∂fₜ/∂vₜ (in ℝ²ˣ²)
    // for each contact point, of size nc.
    std::vector<Matrix2<T>>& mutable_dft_dvt() {
      return dft_dv_;
    }

   private:
    // The number of contact points. This determines sizes in this workspace.
    int nc_, nv_;
    VectorX<T> Delta_vn_;  // Δvₙᵏ = Jₙ Δvᵏ, in ℝⁿᶜ, for the k-th iteration.
    VectorX<T> Delta_vt_;  // Δvₜᵏ = Jₜ Δvᵏ, in ℝ²ⁿᶜ, for the k-th iteration.
    VectorX<T> vn_;        // vₙᵏ, in ℝⁿᶜ.
    VectorX<T> vt_;        // vₜᵏ, in ℝ²ⁿᶜ.
    VectorX<T> fn_;        // fₙᵏ, in ℝⁿᶜ.
    VectorX<T> ft_;        // fₜᵏ, in ℝ²ⁿᶜ.
    VectorX<T> x_;         // xˢ⁺¹ = xˢ − δt vₙˢ
    VectorX<T> t_hat_;     // Tangential directions, t̂ᵏ. In ℝ²ⁿᶜ.
    VectorX<T> v_slip_;    // vₛᵏ = ‖vₜᵏ‖, in ℝⁿᶜ.
    VectorX<T> mus_;       // (modified) Stribeck friction, in ℝⁿᶜ.
    // Vector of size nc storing ∂fₜ/∂vₜ (in ℝ²ˣ²) for each contact point.
    std::vector<Matrix2<T>> dft_dv_;
    MatrixX<T> Gn_;        // ∇ᵥfₙ(xˢ⁺¹, vₙˢ⁺¹), in ℝⁿᶜˣⁿᵛ
  };

  // Returns true if the solver is solving the two-way coupled problem.
  bool has_two_way_coupling() const {
    return problem_data_aliases_.has_two_way_coupling_data();
  }

  // Helper method to compute, into fn, the normal force at each contact
  // point pair according to the law:
  //   fₙ(x, vₙ) = k(vₙ)₊ x₊
  //       k(vₙ) = k (1 − d vₙ)₊
  // where `x₊` is max(x, 0) and k and d are the stiffness and
  // dissipation coefficients for a given contact point, respectively.
  // In addition, this method also computes the gradient
  // Gn = ∇ᵥfₙ(xˢ⁺¹, vₙˢ⁺¹).
  void CalcNormalForces(
      const Eigen::Ref<const VectorX<T>>& x,
      const Eigen::Ref<const VectorX<T>>& vn,
      const Eigen::Ref<const MatrixX<T>>& Jn,
      double dt,
      EigenPtr<VectorX<T>> fn,
      EigenPtr<MatrixX<T>> Gn) const;

  // Helper to compute fₜ(vₜ) = −vₜ/‖vₜ‖ₛ μ(‖vₜ‖ₛ) fₙ, where ‖vₜ‖ₛ
  // is the "soft norm" of vₜ. In addition this method computes
  // v_slip = ‖vₜ‖ₛ, t_hat = vₜ/‖vₜ‖ₛ and mu_stribeck = μ(‖vₜ‖ₛ).
  void CalcFrictionForces(
      const Eigen::Ref<const VectorX<T>>& vt,
      const Eigen::Ref<const VectorX<T>>& fn,
      EigenPtr<VectorX<T>> v_slip,
      EigenPtr<VectorX<T>> t_hat,
      EigenPtr<VectorX<T>> mu_stribeck,
      EigenPtr<VectorX<T>> ft) const;

  // Helper to compute gradient dft_dvt = −∇ᵥₜfₜ(vₜ), as a function of the
  // normal force fn, friction coefficient mu_vt (μ(‖vₜ‖)), tangent versor
  // t_hat and (current) slip velocity v_slip.
  // We define dft_dvt as minus the gradient of the friction forces, ie.
  // dft_dvt = −∇ᵥₜfₜ(vₜ), so that dft_dvt is PSD, which is convenient for
  // stability analysis of the time stepping method.
  void CalcFrictionForcesGradient(
      const Eigen::Ref<const VectorX<T>>& fn,
      const Eigen::Ref<const VectorX<T>>& mu_vt,
      const Eigen::Ref<const VectorX<T>>& t_hat,
      const Eigen::Ref<const VectorX<T>>& v_slip,
      std::vector<Matrix2<T>>* dft_dvt) const;

  // Helper method to compute the Newton-Raphson Jacobian, J = ∇ᵥR, as a
  // function of M, Jn, Jt, Gn, dft_dvt, t_hat, mu_vt and dt.
  void CalcJacobian(
      const Eigen::Ref<const MatrixX<T>>& M,
      const Eigen::Ref<const MatrixX<T>>& Jn,
      const Eigen::Ref<const MatrixX<T>>& Jt,
      const Eigen::Ref<const MatrixX<T>>& Gn,
      const std::vector<Matrix2<T>>& dft_dvt,
      const Eigen::Ref<const VectorX<T>>& t_hat,
      const Eigen::Ref<const VectorX<T>>& mu_vt, double dt,
      EigenPtr<MatrixX<T>> J) const;

  // Limit the per-iteration angle change between vₜᵏ⁺¹ and vₜᵏ for
  // all contact points. The angle change θ is defined by the dot product
  // between vₜᵏ⁺¹ and vₜᵏ as: cos(θ) = vₜᵏ⁺¹⋅vₜᵏ/(‖vₜᵏ⁺¹‖‖vₜᵏ‖).
  // We'll do so by computing a coefficient 0 < α ≤ 1 so that if the
  // generalized velocities are updated as vᵏ⁺¹ = vᵏ + α Δvᵏ then θ ≤ θₘₐₓ
  // for all contact points.
  T CalcAlpha(const Eigen::Ref<const VectorX<T>>& vt,
              const Eigen::Ref<const VectorX<T>>& Delta_vt) const;

  // Dimensionless modified Stribeck function defined as:
  // ms(s) = ⌈ mu * s * (2 − s),  s  < 1
  //         ⌊ mu              ,  s >= 1
  // where s corresponds to the dimensionless tangential speed
  // s = ‖vᵏ‖ / vₛ, where vₛ is the Stribeck stiction tolerance.
  // The solver uses this modified Stribeck function for two reasons:
  //   1. Static and dynamic friction coefficients are the same. This avoids
  //      regions of negative slope. If the slope is always positive the
  //      implicit update is unconditionally stable.
  //   2. Non-zero derivative at s = 0 (zero slip velocity). This provides a
  //      good strong gradient in the neighborhood to zero slip velocities that
  //      aids in finding a good solution update.
  static T ModifiedStribeck(const T& s, const T& mu);

  // Derivative of the dimensionless modified Stribeck function:
  // d/ds ms(s) = ⌈ mu * (2 * (1 − s)),  s  < 1
  //              ⌊ 0                 ,  s >= 1
  // where s corresponds to the dimensionless tangential speed
  // s = ‖v‖ / vₛ.
  static T ModifiedStribeckDerivative(const T& speed_BcAc, const T& mu);

  int nv_;  // Number of generalized velocities.
  int nc_;  // Number of contact points.

  // The parameters of the solver controlling the iteration strategy.
  ImplicitStribeckSolverParameters parameters_;
  ProblemDataAliases problem_data_aliases_;
  mutable FixedSizeWorkspace fixed_size_workspace_;
  mutable VariableSizeWorkspace variable_size_workspace_;

  // Precomputed value of cos(theta_max), used by DirectionChangeLimiter.
  double cos_theta_max_{std::cos(parameters_.theta_max)};

  // We save solver statistics such as number of iterations and residuals so
  // that we can report them if requested.
  mutable ImplicitStribeckSolverIterationStats statistics_;
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    struct ::drake::multibody::internal::DirectionChangeLimiter)

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::ImplicitStribeckSolver)
