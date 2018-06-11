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
/// equal to the stiction tolerance of the solver vₛ. We refer to
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
/// Since the Stribeck function used for the friction coefficient has very
/// steep gradients only within the stiction region and zero outside of it,
/// gradients (that is the Jacobian of the residual) in the Newton-Raphson
/// iteration performed by the implicit Stribeck solver are large within
/// these small circular regions (stiction) and small outside them (slip).
/// We say that gradients within these stiction circles are "strong" and
/// gradients outside these regions are "weak".
/// These regions are so small compared to the velocity scales dealt with
/// by the implicit Stribeck solver, that effectively, the Newton-Raphson
/// iteration would only "see" a fixed dynamic coefficient of friction and it
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
  /// @param[in] tolerance a value << 1 used to determine when ‖vₜ‖ ≈ 0.
  /// Typical values lie withing the 10⁻³ - 10⁻² range. This allows us to
  /// compute `εᵥ = tolerance⋅vₛ` (in m/s) which defines a "small tangential
  /// velocity scale". This value is used to compute "soft norms" (see class's
  /// documentation) and to detect values close to zero, ‖vₜ‖ < εᵥ.
  /// @retval α the limit in [0, 1] so that vₜᵏ⁺¹ = vₜᵏ + αΔvₜᵏ.
  static T CalcAlpha(const Eigen::Ref<const Vector2<T>>& v,
                     const Eigen::Ref<const Vector2<T>>& dv,
                     double cos_theta_max, double v_stiction, double tolerance);

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
enum ComputationInfo {
  /// Successful computation.
  Success = 0,

  /// The maximum number of iterations was reached.
  MaxIterationsReached = 1,

  /// The linear solver used within the Newton-Raphson loop failed.
  /// This might be caused by a divergent iteration that led to an invalid
  /// Jacobian matrix.
  LinearSolverFailed = 2
};

/// These are the paramters controlling the iteration process of the
/// ImplicitStribeckSolver solver.
struct Parameters {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Parameters);

  Parameters() = default;

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
  /// Typical value is about 1%.
  double tolerance{1.0e-2};

  double theta_max{M_PI / 3.0};
};

/// Struct used to store information about the iteration process performed by
/// ImplicitStribeckSolver.
struct IterationStats {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(IterationStats);

  IterationStats() = default;

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
};

/// %ImplicitStribeckSolver solves the equations below for mechanical systems
/// with contact using a modified Stribeck model of friction: <pre>
///             q̇ = N(q) v
///   (1)  M(q)⋅v̇ = τ + Jₙᵀ(q)⋅fₙ(q, v) + Jₜᵀ(q)⋅fₜ(v)
/// </pre>
/// where `v ∈ ℝⁿᵛ` is the vector of generalized velocities, `M(q) ∈ ℝⁿᵛˣⁿᵛ` is
/// the mass matrix, `Jₙᵀ(q) ∈ ℝⁿᶜˣⁿᵛ` is the Jacobian of normal separation
/// velocities, `Jₜᵀ(q) ∈ ℝ²ⁿᶜˣⁿᵛ` is the Jacobian of tangent velocities,
/// `fₙ ∈ ℝⁿᶜ` is the vector of normal contact forces, `fₜ ∈ ℝ²ⁿᶜ` is the
/// vector of tangent friction forces and τ ∈ ℝⁿᵛ is a vector of generalized
/// forces containing all other applied forces (e.g., Coriolis, gyroscopic
/// terms, actuator forces, etc.) but contact forces. Since
/// %ImplicitStribeckSolver uses a Stribeck model for friction, `fₜ(v)`
/// implicitly depends on the vector of generalized velocities.
///
/// Equations (1) is discretized in time using a first order semi-implicit Euler
/// scheme with time step `δt` as: <pre>
///              qⁿ⁺¹ = qⁿ + δt N(qⁿ)⋅vⁿ⁺¹
///   (2)  M(qⁿ)⋅vⁿ⁺¹ =
///            M(qⁿ)⋅vⁿ + δt (τⁿ + Jₙᵀ(qⁿ)⋅fₙ(qⁿ, vⁿ) + Jₜᵀ(qⁿ)⋅fₜ(vⁿ⁺¹))
/// </pre>
/// Please see details in the @ref time_discretization "Discretization in Time"
/// section.
/// The equation for the generalized velocities in Eq. (2) is rewritten as:
/// <pre>
///   (3)  M⋅vⁿ⁺¹ = p* + δt Jₜᵀ⋅fₜ(vⁿ⁺¹)
/// </pre>
/// where `p* = M⋅vⁿ + δt τⁿ + δt Jₙᵀ⋅fₙ` is the generalized momentum that the
/// system would have in the absence of friction forces and, for simplicity, we
/// have only kept the functional dependencies in generalized velocities. Notice
/// that %ImplicitStribeckSolver uses a precomputed value of the normal forces.
/// These normal forces could be available for instance if
/// using a compliant contact approach, for which normal forces are a function
/// of the state.
///
/// %ImplicitStribeckSolver is designed to solve, implicitly, the system in
/// Eq. (3) for the next time step vector of generalized velocities `vⁿ⁺¹`.
/// The solver uses a Newton-Raphson iteration to compute an update `Δvᵏ` at the
/// k-th Newton-Raphson iteration. Once `Δvᵏ` is computed, the solver limits the
/// change in the tangential velocities `Δvₜᵏ = Jₜᵀ⋅Δvᵏ` using the approach
/// described in [Uchida et al., 2015]. This approach limits the maximum angle
/// change θ between two successive iterations in the tangential velocity. The
/// maximum angle change between iterations is given by the parameter
/// Parameters::theta_max.
///
/// Uchida, T.K., Sherman, M.A. and Delp, S.L., 2015.
///   Making a meaningful impact: modelling simultaneous frictional collisions
///   in spatial multibody systems. Proc. R. Soc. A, 471(2177), p.20140859.
///
/// @anchor time_discretization
/// <h2>Discretization in Time</h2>
/// In this section we provide a detailed derivation of the first order time
/// stepping approach in Eq. (2). We start from the continuous Eq. (1): <pre>
///   (1)  M(q)⋅v̇ = τ + Jₙᵀ(q)⋅fₙ(q, v) + Jₜᵀ(q)⋅fₜ(v)
/// </pre>
/// we can discretize Eq. (1) in time using a first order semi-implicit Euler
/// scheme in velocities: <pre>
///   (4)  M(qⁿ)⋅vⁿ⁺¹ = M(qⁿ)⋅vⁿ +
///           δt (τⁿ + Jₙᵀ(qⁿ)⋅fₙ(qⁿ, vⁿ⁺¹) + Jₜᵀ(qⁿ)⋅fₜ(vⁿ⁺¹)) + O₁(δt²)
/// </pre>
/// where the equality holds strictly since we included the leading terms in
/// `O(δt²)`. We use `τⁿ = τ(tⁿ, qⁿ, vⁿ⁺¹)` for brevity in Eq. (4).
/// When moving from the continuous Eq. (1) to the discrete version Eq. (4), we
/// lost the nice property that our compliant normal forces are decoupled from
/// the friction forces (both depend on the same unknown vⁿ⁺¹ in Eq (4)). The
/// reason is that Eq. (4) includes an integration over a small interval of
/// size δt. To solve the discrete system in Eq. (4), we'd like to decouple the
/// normal forces from the tangential forces again, which will require a new
/// (though still valid) approximation.
/// To do so we will expand in Taylor series the term `fₙ(qⁿ, vⁿ⁺¹)`: <pre>
///   (5)  fₙ(qⁿ, vⁿ⁺¹) = fₙ(qⁿ, vⁿ) + ∇ᵥfₙ(qⁿ,vⁿ)⋅(vⁿ⁺¹-vⁿ) + O₂(‖vⁿ⁺¹-vⁿ‖²)
/// </pre>
/// The difference between `vⁿ` and `vⁿ⁺¹` can be written as: <pre>
///   (6)  vⁿ⁺¹-vⁿ = δtv̇ⁿ + δtO₃(δt²) = O₄(δt)
/// </pre>
/// Substituting `vⁿ⁺¹-vⁿ` from Eq. (6) into Eq. (5) we arrive to: <pre>
///   (7)  fₙ(qⁿ, vⁿ⁺¹) = fₙ(qⁿ, vⁿ) + ∇ᵥfₙ(qⁿ,vⁿ)⋅O₄(δt) + O₅(δt²)
///                     = fₙ(qⁿ, vⁿ) + O₆(δt)
/// </pre>
/// where `O₅(δt²) = O₂(‖vⁿ⁺¹-vⁿ‖²) = O₂(‖O₄(δt)‖²)`.
/// We can now use Eq. (7) into Eq. (4) to arrive to: <pre>
///   (8)  M(qⁿ)⋅vⁿ⁺¹ = M(qⁿ)⋅vⁿ +
///         δt (τⁿ + Jₙᵀ(qⁿ)⋅(fₙ(qⁿ, vⁿ) + O₆(δt)) + Jₜᵀ(qⁿ)⋅fₜ(vⁿ⁺¹)) +
///         O₁(δt²)
/// </pre>
/// which we can rewrite as: <pre>
///   (9)  M(qⁿ)⋅vⁿ⁺¹ = M(qⁿ)⋅vⁿ +
///       δt (τⁿ + Jₙᵀ(qⁿ)⋅fₙ(qⁿ, vⁿ) + Jₜᵀ(qⁿ)⋅fₜ(vⁿ⁺¹)) + O₇(δt²)
/// </pre>
/// with `O₇(δt²) = δt Jₙᵀ(qⁿ)⋅O₆(δt) + O₁(δt²)`.
/// That is, Eq. (9) introduces the same order of approximation as in the
/// semi-implicit method in Eq. (4).
/// Up to this point we have made no approximations but we instead propagated
/// the `O(⋅)` leading terms. Therefore the equalities in the equations above
/// are exact. To obtain an approximate time stepping scheme, we drop `O₇(δt²)`
/// (we neglect it) in Eq. (9) to arrive to a first order scheme:<pre>
///   (10)  M(qⁿ)⋅vⁿ⁺¹ = M(qⁿ)⋅vⁿ +
///                      δt (τⁿ + Jₙᵀ(qⁿ)⋅fₙ(qⁿ, vⁿ) + Jₜᵀ(qⁿ)⋅fₜ(vⁿ⁺¹))
/// </pre>
/// Therefore, with the scheme in Eq. (10) we are able to decouple the
/// computation of (compliant) normal forces from that of friction forces.
/// A very important feature of this scheme however, is the explicit nature (in
/// the velocities v) of the term associated with the normal forces (usually
/// including dissipation in the normal direction), which will become unstable
/// for a sufficiently large time step. However, for most applications in
/// practice, the stability of the scheme is mostly determined by the explicit
/// update of normal forces with positions, that is, Eq. (10) is explicit in
/// positions through the normal forces `fₙ(qⁿ, vⁿ)`. For many common
/// applications, the explicit dependence of `τⁿ(tⁿ, qⁿ, vⁿ)` on the
/// previous time step velocities `vⁿ` determines the overall stability of
/// the scheme, since this term can include velocity dependent contributions
/// such as control forces and dampers. Notice that Eq. (5) introduces an
/// expansion of `fₙ` with an order of approximation consistent with the
/// first order scheme as needed. Therefore, it propagates into a `O(δt²)`
/// term exactly as needed in Eq. (9).
///
/// @tparam T The type of mathematical object being added.
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
template <typename T>
class ImplicitStribeckSolver {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ImplicitStribeckSolver)

  /// Instantiates a solver for a problem with `nv` generalized velocities.
  /// @throws std::exception if nv is non-positive.
  explicit ImplicitStribeckSolver(int nv);

  /// Sets data for the problem to be solved as outlined by Eq. (3) in this
  /// class's documentation: <pre>
  ///   M⋅v = p* + δt Jₜᵀ⋅fₜ(v)
  /// </pre>
  /// Refer to this class's documentation for further details on the structure
  /// of the problem and the solution strategy.
  /// In the documented parameters below, `nv` is the number of generalized
  /// velocities and `nc` is the number of contact points.
  ///
  /// @param[in] M
  ///   The mass matrix of the system, of size `nv x nv`.
  /// @param[in] Jt
  ///   The tangential velocities Jacobian, of sice `2nc x nv`.
  /// @param[in] p_star
  ///   The generalized momentum the system would have at `n + 1` if friction
  ///   forces where zero.
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
  ///      solver. In such a case, SetProblemData() and SolveWithGuess() must be
  ///      invoked again.
  void SetProblemData(
      EigenPtr<const MatrixX<T>> M,
      EigenPtr<const MatrixX<T>> Jn, EigenPtr<const MatrixX<T>> Jt,
      EigenPtr<const VectorX<T>> p_star,
      EigenPtr<const VectorX<T>> fn, EigenPtr<const VectorX<T>> mu);

  /// The problem is two-way coupled: normal forces affect friction forces and,
  /// conversely, friction forces affect normal forces.
  void SetTwoWayCoupledProblemData(
      EigenPtr<const MatrixX<T>> M,
      EigenPtr<const MatrixX<T>> Jn, EigenPtr<const MatrixX<T>> Jt,
      EigenPtr<const VectorX<T>> p_star,
      EigenPtr<const VectorX<T>> phi0,
      EigenPtr<const VectorX<T>> stiffness, EigenPtr<const VectorX<T>> damping,
      EigenPtr<const VectorX<T>> mu);

  /// Solves for the generalized velocities satisfying Eq. (3) in this class's
  /// documentation. To retrieve the solution, please refer to
  /// @ref retrieving_the_solution.
  /// @returns ComputationInfo::Success if the iteration converges. All other
  /// values of ComputationInfo report different failure modes.
  /// Uses `this` solver accessors to retrieve the last computed solution.
  /// @warning Always perform the check on the returned ComputationInfo for the
  /// success of the solver before retrieving the computed solution.
  ///
  /// @param[in] dt The time step used advance the solution in time.
  /// @param[in] v_guess The initial guess used in by the Newton-Raphson
  /// iteration.
  ///
  /// @throws std::logic_error if `v_guess` is not of size `nv`, the number of
  /// generalized velocities specified at construction.
  ComputationInfo SolveWithGuess(double dt, const VectorX<T>& v_guess);

  /// @anchor retrieving_the_solution
  /// @name Retrieving the solution
  /// This methods allow to retrieve the solution stored in the solver after
  /// the last call to SolveWithGuess().
  /// @{

  /// Returns a constant reference to the last solved vector of generalized
  /// friction forces.
  const VectorX<T>& get_generalized_forces() const {
    return fixed_size_workspace_.tau_f;
  }

  /// Returns a constant reference to the last solved vector of tangential
  /// forces. This method returns an `Eigen::VectorBlock` referencing a vector
  /// of size `nc` in accordance to the data last set with SetProblemData().
  Eigen::VectorBlock<const VectorX<T>> get_tangential_velocities() const {
    return variable_size_workspace_.vt();
  }

  /// Returns a constant reference to the last solved vector of generalized
  /// velocities.
  const VectorX<T>& get_generalized_velocities() const {
    return fixed_size_workspace_.v;
  }
  /// @}

  /// Returns statistics recorded during the last call to SolveWithGuess().
  /// See IterationStats for details.
  const IterationStats& get_iteration_statistics() const {
    return statistics_;
  }

  /// Returns the current set of parameters controlling the iteration process.
  /// See Parameters for details.
  const Parameters& get_solver_parameters() const {
    return parameters_;
  }

  /// Sets the parameters to be used by the solver.
  /// See Parameters for details.
  void set_solver_parameters(const Parameters parameters) {
    parameters_ = parameters;
  }

 private:
  // Helper class for unit testing.
  friend class ImplicitStribeckSolverTester;

  // Contains all the references that define the problem to be solved.
  // These references must remain valid at least from the time they are set with
  // SetProblemData() and until SolveWithGuess() returns.
  struct ProblemDataAliases {
    // Sets the references to the data defining the problem.
    void Set(EigenPtr<const MatrixX<T>> M,
             EigenPtr<const MatrixX<T>> Jn, EigenPtr<const MatrixX<T>> Jt,
             EigenPtr<const VectorX<T>> p_star,
             EigenPtr<const VectorX<T>> fn, EigenPtr<const VectorX<T>> mu) {
      M_ptr = M;
      Jn_ptr = Jn;
      Jt_ptr = Jt;
      p_star_ptr = p_star;
      fn_ptr = fn;
      mu_ptr = mu;
    }

    void Set(EigenPtr<const MatrixX<T>> M,
             EigenPtr<const MatrixX<T>> Jn, EigenPtr<const MatrixX<T>> Jt,
             EigenPtr<const VectorX<T>> p_star,
             EigenPtr<const VectorX<T>> phi0,
             EigenPtr<const VectorX<T>> stiffness,
             EigenPtr<const VectorX<T>> damping,
             EigenPtr<const VectorX<T>> mu) {
      M_ptr = M;
      Jn_ptr = Jn;
      Jt_ptr = Jt;
      p_star_ptr = p_star;
      phi0_ptr = phi0;
      stiffness_ptr = stiffness;
      damping_ptr = damping;
      mu_ptr = mu;
    }

    // The mass matrix of the system.
    EigenPtr<const MatrixX<T>> M_ptr{nullptr};
    // The separation velocities Jacobian.
    EigenPtr<const MatrixX<T>> Jn_ptr{nullptr};
    // The tangential velocities Jacobian.
    EigenPtr<const MatrixX<T>> Jt_ptr{nullptr};
    // The generalized momementum vector **before** friction is applied.
    EigenPtr<const VectorX<T>> p_star_ptr{nullptr};
    // At each contact point ic, fn(ic) and mu(ic) store the normal contact
    // force and friction coefficient, respectively. Both have size nc, the
    // number of contact points.
    // fn_ptr is nullptr for two-way coupled problems.
    EigenPtr<const VectorX<T>> fn_ptr{nullptr};
    // Penetration distance. Positive when there is penetration and negative
    // when there is separation, i.e. it is minus the signed distance function.
    // phi0_ptr is nullptr for one-way coupled problems.
    EigenPtr<const VectorX<T>> phi0_ptr{nullptr};
    // Stiffness in the normal direction. nullptr for one-way coupled problems.
    EigenPtr<const VectorX<T>> stiffness_ptr{nullptr};
    // Damping in the normal direction. nullptr for one-way coupled problems.
    EigenPtr<const VectorX<T>> damping_ptr{nullptr};
    EigenPtr<const VectorX<T>> mu_ptr{nullptr};
  };

  // The solver's workspace allocated at construction time. Sizes only depend on
  // nv, the number of generalized velocities.
  // The size of the variables in this workspace MUST remain fixed throghout the
  // lifetime of the solver. Do not resize any of them!.
  struct FixedSizeWorkspace {
    // Constructs a workspace with size only dependent on nv.
    explicit FixedSizeWorkspace(int nv) : J_lu(nv) {
      v.resize(nv);
      residual.resize(nv);
      Delta_v.resize(nv);
      J.resize(nv, nv);
      J_ldlt = std::make_unique<Eigen::LDLT<MatrixX<T>>>(nv);
      tau_f.resize(nv);
      tau.resize(nv);
    }
    // Vector of generalized velocities.
    VectorX<T> v;
    // Newton-Raphson residual.
    VectorX<T> residual;
    // Newton-Raphson Jacobian, i.e. Jᵢⱼ = ∂Rᵢ/∂vⱼ.
    MatrixX<T> J;
    // Solution to Newton-Raphson update, i.e. Δv = -J⁻¹⋅R.
    VectorX<T> Delta_v;
    // Vector of generalized forces due to friction.
    VectorX<T> tau_f;
    // Vector of generalized forces (normal + friction forces).
    VectorX<T> tau;
    // LDLT Factorization of the Newton-Raphson Jacobian J.
    std::unique_ptr<Eigen::LDLT<MatrixX<T>>> J_ldlt;
    Eigen::PartialPivLU<MatrixX<T>> J_lu;
  };

  // The variables in this workspace can change size with each invocation of
  // SetProblemData() since the number of contact points nc can change.
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
      phi_.resize(nc);
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

    Eigen::VectorBlock<VectorX<T>> mutable_vn() {
      return vn_.segment(0, nc_);
    }

    /// Returns a constant reference to the vector containing the tangential
    /// velocities vₜ for all contact points, of size 2nc.
    Eigen::VectorBlock<const VectorX<T>> vt() const {
      return vt_.segment(0, 2 * nc_);
    }

    /// Mutable version of vt().
    Eigen::VectorBlock<VectorX<T>> mutable_vt() {
      return vt_.segment(0, 2 * nc_);
    }

    /// Returns a mutable reference to the vector containing the tangential
    /// velocity updates Δvₜ for all contact points, of size 2nc.
    Eigen::VectorBlock<VectorX<T>> mutable_delta_vt() {
      return Delta_vt_.segment(0, 2 * nc_);
    }

    /// Returns a mutable reference to the vector containing the normal
    /// contact forces fₙ for all contact points, of size nc.
    Eigen::VectorBlock<VectorX<T>> mutable_fn() {
      return fn_.segment(0, nc_);
    }

    Eigen::VectorBlock<VectorX<T>> mutable_phi() {
      return phi_.segment(0, nc_);
    }

    /// Returns a mutable reference to the vector containing the tangential
    /// friction forces fₜ for all contact points, of size 2nc.
    Eigen::VectorBlock<VectorX<T>> mutable_ft() {
      return ft_.segment(0, 2 * nc_);
    }

    /// Returns a mutable reference to the vector containing the tangential
    /// directions t̂ᵏ for all contact points, of size 2nc.
    Eigen::VectorBlock<VectorX<T>> mutable_t_hat() {
      return t_hat_.segment(0, 2 * nc_);
    }

    /// Returns a mutable reference to the vector containing the slip velocity
    /// , vₛ = ‖vₜ‖, at each contact point, of size nc.
    Eigen::VectorBlock<VectorX<T>> mutable_v_slip() {
      return v_slip_.segment(0, nc_);
    }

    /// Returns a mutable reference to the vector containing the stribeck
    /// friction, function of the slip velocity, at each contact point, of
    /// size nc.
    Eigen::VectorBlock<VectorX<T>> mutable_mu() {
      return mus_.segment(0, nc_);
    }

    Eigen::Block<MatrixX<T>> mutable_Gn() {
      return Gn_.block(0, 0, nc_, nv_);
    }

    /// Returns a mutable reference to the vector storing ∂fₜ/∂vₜ (in ℝ²ˣ²)
    /// for each contact pont, of size nc.
    std::vector<Matrix2<T>>& mutable_dft_dv() {
      return dft_dv_;
    }

   private:
    // The number of contact points. This determines sizes in this workspace.
    int nc_, nv_;
    VectorX<T> Delta_vt_;  // Δvₜᵏ = Jₜ⋅Δvᵏ, in ℝ²ⁿᶜ, for the k-th iteration.
    VectorX<T> vn_;        // vₙᵏ, in ℝⁿᶜ.
    VectorX<T> vt_;        // vₜᵏ, in ℝ²ⁿᶜ.
    VectorX<T> fn_;        // fₙᵏ, in ℝⁿᶜ.
    VectorX<T> ft_;        // fₜᵏ, in ℝ²ⁿᶜ.
    VectorX<T> phi_;       // φⁿ⁺¹ = φⁿ - δt vₙⁿ
    VectorX<T> t_hat_;     // Tangential directions, t̂ᵏ. In ℝ²ⁿᶜ.
    VectorX<T> v_slip_;    // vₛᵏ = ‖vₜᵏ‖, in ℝⁿᶜ.
    VectorX<T> mus_;       // (modified) Stribeck friction, in ℝⁿᶜ.
    // Vector of size nc storing ∂fₜ/∂vₜ (in ℝ²ˣ²) for each contact point.
    std::vector<Matrix2<T>> dft_dv_;
    MatrixX<T> Gn_;
  };

  // Returns true if the solver is solving the two-way coupled problem.
  bool two_way_coupling() const {
    return problem_data_aliases_.fn_ptr == nullptr;
  }

  void CalcNormalForces(
      const Eigen::Ref<const VectorX<T>>& phi,
      const Eigen::Ref<const VectorX<T>>& vn,
      const Eigen::Ref<const MatrixX<T>>& Jn,
      double dt,
      EigenPtr<VectorX<T>> fn_ptr,
      EigenPtr<MatrixX<T>> Gn_ptr);

  void CalcFrictionForces(const Eigen::Ref<const VectorX<T>>& vt,
                          const Eigen::Ref<const VectorX<T>>& fn);

  void CalcFrictionForcesGradient(
      const Eigen::Ref<const VectorX<T>>& fn,
      const Eigen::Ref<const VectorX<T>>& mus,
      const Eigen::Ref<const VectorX<T>>& t_hat,
      const Eigen::Ref<const VectorX<T>>& v_slip);

  void CalcJacobian(
      const Eigen::Ref<const MatrixX<T>>& M,
      const Eigen::Ref<const MatrixX<T>>& Jn,
      const Eigen::Ref<const MatrixX<T>>& Jt,
      const Eigen::Ref<const MatrixX<T>>& Gn,
      const std::vector<Matrix2<T>>& dft_dvt,
      const Eigen::Ref<const VectorX<T>>& t_hat,
      const Eigen::Ref<const VectorX<T>>& mus,
      double dt);

  // Dimensionless modified Stribeck function defined as:
  // ms(x) = ⌈ mu * x * (2.0 - x),  x  < 1
  //         ⌊ mu                ,  x >= 1
  // where x corresponds to the dimensionless tangential speed
  // x = ‖vᵏ‖ / vₛ, where vₛ is the Stribeck stiction tolerance.
  // The solver uses this modified Stribeck function for two reasons:
  //   1. Static and dynamic friction coefficients are the same. This avoids
  //      regions of negative slope. If the slope is always positive the
  //      implicit update is unconditionally stable.
  //   2. Non-zero derivative at x = 0 (zero slip velocity). This provides a
  //      good strong gradient in the neighborhood to zero slip velocities that
  //      aids in finding a good solution update.
  static T ModifiedStribeck(const T& x, const T& mu);

  // Derivative of the dimensionless modified Stribeck function:
  // ms(x) = ⌈ mu * (2 * (1 - x)),  x  < 1
  //         ⌊ 0                 ,  x >= 1
  // where x corresponds to the dimensionless tangential speed
  // x = v / v_stribeck.
  static T ModifiedStribeckDerivative(const T& speed_BcAc, const T& mu);

  int nv_;  // Number of generalized velocities.
  int nc_;  // Number of contact points.

  // The parameters of the solver controlling the iteration strategy.
  Parameters parameters_;
  ProblemDataAliases problem_data_aliases_;
  mutable FixedSizeWorkspace fixed_size_workspace_;
  mutable VariableSizeWorkspace variable_size_workspace_;

  // We save solver statistics such as number of iterations and residuals so
  // that we can report them if requested.
  mutable IterationStats statistics_;
};

}  // namespace implicit_stribeck
}  // namespace multibody
}  // namespace drake
