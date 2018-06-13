#include "drake/multibody/multibody_tree/implicit_stribeck/implicit_stribeck_solver.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/extract_double.h"

namespace drake {
namespace multibody {
namespace implicit_stribeck {
namespace internal {
template <typename T>
T DirectionChangeLimiter<T>::CalcAlpha(
    const Eigen::Ref<const Vector2<T>>& v,
    const Eigen::Ref<const Vector2<T>>& dv,
    double cos_theta_max, double v_stiction, double relative_tolerance) {
  DRAKE_ASSERT(v_stiction > 0);
  DRAKE_ASSERT(relative_tolerance > 0);
  DRAKE_ASSERT(dv.size() == v.size());

  using std::abs;
  using std::max;
  using std::min;
  using std::sqrt;

  // εᵥ is used to determine when a velocity is close to zero.
  const double epsilon_v = v_stiction * relative_tolerance;
  const double epsilon_v2 = epsilon_v * epsilon_v;

  const Vector2<T> v1 = v + dv;  // v_alpha = v + dv, when alpha = 1.

  // Case I: Quick exit for small changes in v.
  const T dv_norm2 = dv.squaredNorm();
  if (dv_norm2 < epsilon_v2) {
    return 1.0;
  }

  const T v_norm = v.norm();
  const T v1_norm = v1.norm();
  const T x = v_norm / v_stiction;    // Dimensionless slip v and v1.
  const T x1 = v1_norm / v_stiction;
  // From Case I, we know dv_norm > epsilon_v.
  const T dv_norm = sqrt(dv_norm2);

  // Case II: limit transition from stiction to sliding when x << 1.0 and
  // gradients might be close to zero (due to the "soft norms").
  if (x < relative_tolerance && x1 > 1.0) {
    // we know v1 != 0  since x1 > 1.0.
    // With v_alpha = v + alpha * dv, we make |v_alpha| = v_stiction / 2.
    // For this case dv ≈ v1 (v ≈ 0). Therefore:
    // alpha ≈ v_stiction / dv_norm / 2.
    return v_stiction / dv_norm / 2.0;
  }

  // Case III: Transition to an almost exact stiction from sliding.
  // We want to avoid v1 landing in a region of zero gradients so we force
  // it to land within the circle of radius v_stiction, at v_stribeck/2 in the
  // direction of v.
  if (x > 1.0 && x1 < relative_tolerance) {
    // In this case x1 is negligible compared to x. That is dv ≈ -v. For this
    // case we'll limit v + αdv = vₛ/2⋅v/‖v‖. Using v ≈ -dv, we arrive to
    // dv(α-1) = -vₛ/2⋅dv/‖dv‖ or:
    return 1.0 - v_stiction / 2.0 / dv_norm;
  }

  if (x < 1.0) {
    // Another quick exit. Two possibilities (both of which yield the same
    // action):
    // x1 < 1: we go from within the Stribeck circle back into it. Since this
    //         region has strong gradients, we allow it. i.e. alpha = 1.0
    // x1 > 1: If we go from a region of strong gradients (x < 1) to sliding
    //         (x1 > 1), we allow it. Notice that the case from weak gradients
    //         (close to zero) when x < relative_tolerance, was covered by
    //         Case II.
    return 1.0;
  } else {  // x > 1.0
    if (x1 < 1.0) {
      // Case IV:
      // From Case III we know that x1 > relative_tolerance, i.e x1 falls in a
      // region of strong gradients and thus we allow it.
      return 1.0;
    }

    // Case V:
    // Since v_stiction is so small, the next case very seldom happens. However,
    // it is a very common case for 1D-like problems for which tangential
    // velocities change in sign and the zero crossing might be missed.
    // Notice that since we reached this point, we know that:
    //  - x > 1.0 (we are within the scope of an if statement for x > 1)
    //  - x1 > 1.0 (we went through Case IV)
    //  - dv_norm > epsilon_v (we went through Case I, i.e. non-zero)
    // Here we are checking for the case when the line connecting v and v1
    // intersects the Stribeck circle. For this case we compute alpha so that
    // the update corresponds to the velocity closest to the origin.
    T alpha;
    const T v_dot_dv = v.dot(dv);
    if (CrossesTheStictionRegion(
        v, dv, v_dot_dv, dv_norm, dv_norm2, epsilon_v, v_stiction, &alpha)) {
      return alpha;
    }

    // If we are here we know:
    //  - x > 1.0
    //  - x1 > 1.0
    //  - dv_norm > epsilon_v
    //  - line connecting v with v1 never goes through the Stribeck circle.
    //
    // Case VI:
    // Therefore we know changes happen entirely outside the circle of radius
    // v_stiction. To avoid large jumps in the direction of v (typically during
    // strong impacts), we limit the maximum angle change between v to v1.
    // To this end, we find a scalar 0 < alpha < 1 such that
    // cos(θₘₐₓ) = v⋅(v+αdv)/(‖v‖‖v+αdv‖), see [Uchida et al., 2015].

    // First we compute the angle change when alpha = 1, between v1 and v.
    const T cos1 = v.dot(v1) / v_norm / v1_norm;

    // We allow angle changes theta < theta_max, and we take alpha = 1.0.
    // In particular, when v1 is exactly aligned with v (but we know it does not
    // cross through zero, i.e. cos(theta) > 0).
    if (cos1 > cos_theta_max) {
      return 1.0;
    } else {
      // we limit the angle change to theta_max so that:
      // cos(θₘₐₓ) = v⋅(v+αdv)/(‖v‖‖v+αdv‖)
      // if we square both sides of this equation, we arrive at a quadratic
      // equation with coefficients a, b, c, for α. The math below simply is the
      // algebra to compute coefficients a, b, c and solve the quadratic
      // equation.

      // All terms are made non-dimensional using v_stiction as the reference
      // scale.
      const T x_dot_dx = v_dot_dv / (v_stiction * v_stiction);
      const T dx = dv.norm() / v_stiction;
      const T x2 = x * x;
      const T dx4 = x2 * x2;
      const T cos_theta_max2 = cos_theta_max * cos_theta_max;

      // Form the terms of the quadratic equation aα² + bα + c = 0.
      const T a = x2 * dx * dx * cos_theta_max2 - x_dot_dx * x_dot_dx;
      const T b = 2 * x2 * x_dot_dx * (cos_theta_max2 - 1.0);
      const T c = dx4 * (cos_theta_max2 - 1.0);

      // Solve quadratic equation. We know, from the geometry of the problem,
      // that the roots to this problem are real. Thus, the discriminant of the
      // quadratic equation (Δ = b² - 4ac) must be positive.
      // We use a very specialized quadratic solver for this case where we know
      // there must exist a positive (i.e. real) root.
      alpha = SolveQuadraticForTheSmallestPositiveRoot(a, b, c);

      // The geometry of the problem tells us that α ≤ 1.0
      DRAKE_ASSERT(alpha <= 1.0);
      return alpha;
    }
  }

  // We should never reach this point.
  throw std::logic_error("Bug detected. An angle change case was missed.");
}

template <typename T>
bool DirectionChangeLimiter<T>::CrossesTheStictionRegion(
    const Eigen::Ref<const Vector2<T>>& v,
    const Eigen::Ref<const Vector2<T>>& dv,
    const T& v_dot_dv,  const T& dv_norm, const T& dv_norm2,
    double epsilon_v, double v_stiction, T* alpha_out) {
  DRAKE_ASSERT(dv_norm > 0);
  DRAKE_ASSERT(dv_norm2 > 0);
  DRAKE_ASSERT(epsilon_v > 0);
  DRAKE_ASSERT(v_stiction > 0);
  T& alpha = *alpha_out;
  if (v_dot_dv < 0.0) {  // Moving towards the origin.
    alpha = -v_dot_dv / dv_norm2;  // alpha > 0
    if (alpha < 1.0) {  // The update might be crossing the stiction region.
      const Vector2<T> v_alpha = v + alpha * dv;  // Note: v_alpha.dot(dv) = 0.
      const T v_alpha_norm = v_alpha.norm();
      if (v_alpha_norm < epsilon_v) {
        // v_alpha is almost zero.
        // This situation happens when dv ≈ -a v with a > 0. Therefore we cap
        // v_alpha to v_alpha = v / ‖v‖⋅vₛ/2. To do this, we "move" v_alpha
        // in the direction opposite of dv/‖dv‖ by magnitude vs/2, or similarly,
        // we subtract ‖dv‖vs/2 from the previously computed alpha.
        alpha -= v_stiction / 2.0 / dv_norm;
        DRAKE_ASSERT(0 < alpha && alpha <= 1);
        return true;  // Crosses the stiction region.
      } else if (v_alpha_norm < v_stiction) {
        // v_alpha falls within the Stribeck circle but its magnitude is
        // larger than epsilon_v.
        return true;  // Crosses the stiction region.
      }
    }
  }
  return false;
}

template <typename T>
T DirectionChangeLimiter<T>::SolveQuadraticForTheSmallestPositiveRoot(
    const T& a, const T& b, const T& c) {
  using std::abs;
  using std::max;
  using std::min;
  using std::sqrt;
  T alpha;
  // First determine if a = 0 (to machine epsilon). This comparison is fair
  // since a is dimensionless.
  if (abs(a) < std::numeric_limits<double>::epsilon()) {
    // There is only a single root to the, now linear, equation bα + c = 0.
    alpha = -c / b;
    // Note: a = 0, α > 0 => x_dot_dx = x * dx * cmin ≠ 0 => b ≠ 0
    // We assert this even though within the scope this method is called it is
    // not possible for b to be zero.
    DRAKE_ASSERT(abs(b) > std::numeric_limits<double>::epsilon());
  } else {
    // The determinant, Δ = b² - 4ac, of the quadratic equation.
    const T Delta = b * b - 4 * a * c;  // Uppercase, as in Δ.
    // Geometry tell us that a real solution does exist i.e. Delta > 0.
    DRAKE_DEMAND(Delta > 0);
    const T sqrt_Delta = sqrt(Delta);

    // To avoid loss of significance, when 4ac is relatively small compared
    // to b² (i.e. the square root of the discriminant is close to b), we use
    // Vieta's formula (α₁α₂ = c / a) to compute the second root given we
    // computed the first root without precision lost. This guarantees the
    // numerical stability of the method.
    const T numerator = -0.5 * (b + (b > 0.0 ? sqrt_Delta : -sqrt_Delta));
    const T alpha1 = numerator / a;
    const T alpha2 = c / numerator;

    // The geometry of the problem tells us that at least one must be
    // positive.
    DRAKE_DEMAND(alpha2 > 0 || alpha1 > 0);

    if (alpha2 > 0 && alpha1 > 0) {
      // This branch is triggered for large angle changes (typically close
      // to 180 degrees) between v1 and vt.
      alpha = min(alpha2, alpha1);
    } else {
      // This branch is triggered for small angles changes (typically
      // smaller than 90 degrees) between v1 and vt.
      alpha = max(alpha2, alpha1);
    }
  }
  return alpha;
}

}  // namespace internal

template <typename T>
ImplicitStribeckSolver<T>::ImplicitStribeckSolver(int nv) :
    nv_(nv),
    fixed_size_workspace_(nv),
    // Provide an initial (arbitrarily large enough for most applications)
    // workspace size so that we avoid re-allocations afterwards as much as we
    // can.
    variable_size_workspace_(128) {
  DRAKE_THROW_UNLESS(nv > 0);
}

template <typename T>
void ImplicitStribeckSolver<T>::SetProblemData(
    EigenPtr<const MatrixX<T>> M, EigenPtr<const MatrixX<T>> Jt,
    EigenPtr<const VectorX<T>> p_star,
    EigenPtr<const VectorX<T>> fn, EigenPtr<const VectorX<T>> mu) {
  nc_ = fn->size();
  DRAKE_THROW_UNLESS(p_star->size() == nv_);
  DRAKE_THROW_UNLESS(M->rows() == nv_ && M->cols() == nv_);
  DRAKE_THROW_UNLESS(Jt->rows() == 2 * nc_ && Jt->cols() == nv_);
  DRAKE_THROW_UNLESS(mu->size() == nc_);
  // Keep references to the problem data.
  problem_data_aliases_.Set(M, Jt, p_star, fn, mu);
  variable_size_workspace_.ResizeIfNeeded(nc_);
}

template <typename T>
ComputationInfo ImplicitStribeckSolver<T>::SolveWithGuess(
    double dt, const VectorX<T>& v_guess) {
  DRAKE_THROW_UNLESS(v_guess.size() == nv_);

  using std::abs;
  using std::max;
  using std::min;
  using std::sqrt;

  // Clear statistics so that we can update them with new ones for this call to
  // SolveWithGuess().
  statistics_.Reset();

  // If there are no contact points return a zero generalized friction forces
  // vector, i.e. tau_f = 0.
  if (nc_ == 0) {
    fixed_size_workspace_.mutable_tau_f().setZero();
    const auto& M = *problem_data_aliases_.M_ptr;
    const auto& p_star = *problem_data_aliases_.p_star_ptr;
    auto& v = fixed_size_workspace_.mutable_v();
    // With no friction forces Eq. (3) in the documentation reduces to
    // M⋅vⁿ⁺¹ = p*.
    v = M.ldlt().solve(p_star);
    // "One iteration" with exactly "zero" vt_error.
    statistics_.Update(0.0);
    return ComputationInfo::Success;
  }

  // Solver parameters.
  const int max_iterations = parameters_.max_iterations;
  // Tolerance used to monitor the convergence of the tangential velocities.
  const double vt_tolerance =
      parameters_.relative_tolerance * parameters_.stiction_tolerance;

  // Problem sizes.
  const int nv = nv_;  // Number of generalized velocities.
  const int nc = nc_;  // Number of contact points.
  // Size of the friction forces vector ft and tangential velocities vector vt.
  const int nf = 2 * nc;

  // Convenient aliases to problem data.
  const auto& M = *problem_data_aliases_.M_ptr;
  const auto& Jt = *problem_data_aliases_.Jt_ptr;
  const auto& p_star = *problem_data_aliases_.p_star_ptr;
  const auto& fn = *problem_data_aliases_.fn_ptr;
  const auto& mu = *problem_data_aliases_.mu_ptr;

  // Convenient aliases to fixed size workspace variables.
  auto& v = fixed_size_workspace_.mutable_v();
  auto& Delta_v = fixed_size_workspace_.mutable_Delta_v();
  auto& residual = fixed_size_workspace_.mutable_residual();
  auto& J = fixed_size_workspace_.mutable_J();
  auto& J_ldlt = fixed_size_workspace_.mutable_J_ldlt();
  auto& tau_f = fixed_size_workspace_.mutable_tau_f();

  // Convenient aliases to variable size workspace variables.
  // Note: auto resolve to Eigen::Block (no copies).
  auto vt = variable_size_workspace_.mutable_vt();
  auto ft = variable_size_workspace_.mutable_ft();
  auto Delta_vt = variable_size_workspace_.mutable_Delta_vt();
  auto Gt = variable_size_workspace_.mutable_dft_dv();
  auto mus = variable_size_workspace_.mutable_mu();
  auto t_hat = variable_size_workspace_.mutable_t_hat();
  auto v_slip = variable_size_workspace_.mutable_v_slip();

  // Initialize vt_error to a value larger than tolerance so that the solver at
  // least performs one iteration.
  T vt_error = 2 * vt_tolerance;

  // Initialize iteration with the provided guess.
  v = v_guess;
  vt = Jt * v;

  // The stiction tolerance.
  // TODO(amcastro-tri): rename v_stribeck to v_stiction, since our
  // "Stribeck function" is not a Stribeck model really.
  const double v_stribeck = parameters_.stiction_tolerance;

  // We use the stiction tolerance as a reference scale to estimate a small
  // velocity epsilon_v. With v_epsilon we define a "soft norm" which we
  // use to compute "soft" tangent vectors to avoid a division by zero
  // singularity when tangential velocities are zero.
  const double epsilon_v = v_stribeck * parameters_.relative_tolerance;
  const double epsilon_v2 = epsilon_v * epsilon_v;

  // TODO(amcastro-tri): Refactor this section of code into smaller chunks.
  for (int iter = 0; iter < max_iterations; ++iter) {
    // Compute 2D tangent vectors.
    // To avoid the singularity at v_slip = ‖vt‖ = 0 we use a "soft norm". The
    // idea is to replace the norm in the definition of slip velocity by a
    // "soft norm":
    //    ‖v‖ₛ ≜ sqrt(vᵀv + εᵥ²)
    // (in code εᵥ is named epsilon_v and εᵥ² is named epislon_v2). We use
    // this to redefine the slip velocity:
    //   v_slip = sqrt(vtᵀvt + v_epsilon)
    // and a "soft" tangent vector:
    //   t_hat = vₜ/‖vₜ‖ₛ
    // which now is not only well defined but it has well defined derivatives.
    // We use these softened quantities all throughout our derivations for
    // consistency.
    // Notes on the effect of the "soft norm":
    // Consider a 1D case for which vₜ = v, to avoid geometric complications,
    // but without loss of generality. If using a soft norm:
    //   fₜ(v) = μ(‖v‖ₛ)v/‖v‖ₛ
    // Now, consider the case εᵥ << v << vₛ (or equivalently, 0 < v << vₛ
    // in the limit to εᵥ --> 0). Approximating fₜ(v) in this limit leads to:
    //   fₜ(v) ≈ 2μ₀sgn(v)|v|
    // where sgn(v) is the sign function, μ₀ the (constant) friction
    // coefficient, and we have used the fact that in the limit
    // v --> 0, μ(|v|) ≈ 2μ₀|v|.
    // In this case (recall this is equivalent to the solution in the
    // limit εᵥ --> 0) fₜ(v) is linear in v.
    // Now, very close to the origin, in the limit |v| << εᵥ, where the
    // "softness" of the norm is important, the limit on fₜ(v) is:
    //   fₜ(v) ≈ 2μ₀v²
    // i.e. fₜ(v) is quadratic in v.
    // This explains why we have "weak" gradients in the limit of vₜ
    // approaching zero.
    // These observations easily extend to the general 2D case.
    for (int ic = 0; ic < nc; ++ic) {  // Index ic scans contact points.
      const int ik = 2 * ic;  // Index ik scans contact vector quantities.
      const auto vt_ic = vt.template segment<2>(ik);
      // "soft norm":
      v_slip(ic) = sqrt(vt_ic.squaredNorm() + epsilon_v2);
      // "soft" tangent vector:
      const Vector2<T> that_ic = vt_ic / v_slip(ic);
      t_hat.template segment<2>(ik) = that_ic;
      mus(ic) = ModifiedStribeck(v_slip(ic) / v_stribeck, mu(ic));
      // Friction force.
      ft.template segment<2>(ik) = -mus(ic) * that_ic * fn(ic);
    }

    // After the previous iteration, we allow updating ft above to have its
    // latest value before leaving.
    if (vt_error < vt_tolerance) {
      // Update generalized forces vector and return.
      tau_f = Jt.transpose() * ft;
      return ComputationInfo::Success;
    }

    // Newton-Raphson residual.
    residual = M * v - p_star - dt * Jt.transpose() * ft;

    // Compute dft/dvt, a 2x2 matrix with the derivative of the friction
    // force (in ℝ²) with respect to the tangent velocity (also in ℝ²).
    for (int ic = 0; ic < nc; ++ic) {
      const int ik = 2 * ic;

      // Compute dmu/dv = (1/v_stribeck) * dmu/dx
      // where x = v_slip / v_stribeck is the dimensionless slip velocity.
      const T x = v_slip(ic) / v_stribeck;
      const T dmudv = ModifiedStribeckDerivative(x, mu(ic)) / v_stribeck;

      const auto t_hat_ic = t_hat.template segment<2>(ik);

      // Projection matrix. It projects in the direction of t_hat.
      // Notice it is a symmetric 2x2 matrix.
      const Matrix2<T> P_ic = t_hat_ic * t_hat_ic.transpose();

      // Removes the component of a vector that lies in the direction of t_hat.
      // This is also a symmetric 2x2 matrix.
      const Matrix2<T> Pperp_ic = Matrix2<T>::Identity() - P_ic;

      // Some notes about projection matrices P:
      //  - They are symmetric, positive semi-definite.
      //  - All their eigenvalues are either one or zero.
      //  - Their rank equals the number of non-zero eigenvalues.
      //  - From the previous item we have rank(P) = trace(P).
      //  - If P is a projection matrix, so is (I - P).
      // From the above we then know that P and Pperp are both projection
      // matrices of rank one (i.e. rank deficient) and are symmetric
      // semi-positive definite. This has very important consequences for the
      // Jacobian of the vt_error.

      // We now compute the gradient with respect to the tangential velocity
      // ∇ᵥₜfₜ(vₜ) as (recall that fₜ(vₜ) = vₜ/‖vₜ‖ₛμ(‖vₜ‖ₛ),
      // with ‖v‖ₛ the soft norm ‖v‖ₛ ≜ sqrt(vᵀv + εᵥ²)):
      //   ∇ᵥₜfₜ = -Gt = -fn * (
      //     mu_stribeck(‖vₜ‖ₛ) / ‖vₜ‖ₛ * Pperp(t_hat) +
      //     dmu_stribeck/dx * P(t_hat) / v_stribeck )
      // where x = ‖vₜ‖ₛ / vₛ is the dimensionless slip velocity and we
      // have defined Gt = -∇ᵥₜfₜ.
      // Therefore Gt (in ℝ²ˣ²) is a linear combination of PSD matrices
      // (P and Pperp) where the coefficients of the combination are positive
      // scalars. Therefore,
      // IMPORTANT NOTE: Gt also PSD.
      // IMPORTANT NOTE 2: The derivation for Gt leads to exactly the
      // same result when using the "softened" definitions for v_slip and
      // t_hat where each occurrence of these quantities is replaced by its
      // softened counterpart.

      // Compute Gt:
      // Changes of vt in the direction perpendicular to t_hat (see the full
      // expression for Gt above).
      Gt[ic] = Pperp_ic * mus(ic) / v_slip(ic);

      // Changes in the magnitude of vt (which in turn makes mu_stribeck
      // change), in the direction of t_hat.
      Gt[ic] += P_ic * dmudv;

      // Note: Gt is a symmetric 2x2 matrix. This will imply the positive
      // definiteness of the system's Jacobian below.
      Gt[ic] *= fn(ic);
    }

    // Newton-Raphson Jacobian, i.e. the derivative of the residual with
    // respect to the independent variables which, in this case, are the
    // generalized velocities of the mechanical system.
    //  J = M + dt Jtᵀdiag(Gt)Jt:
    // J is an (nv x nv) symmetric positive definite matrix.
    // diag(Gt) is the (2nc x 2nc) block diagonal matrix with Gt in each 2x2
    // diagonal entry.

    // Start by multiplying diag(Gt)Jt and use the fact that diag(Gt) is
    // block diagonal.
    MatrixX<T> diag_Gt_times_Jt(nf, nv);
    // TODO(amcastro-tri): Only build half of the matrix since it is
    // symmetric.
    for (int ic = 0; ic < nc; ++ic) {  // Index ic scans contact points.
      const int ik = 2 * ic;  // Index ik scans contact vector quantities.
      diag_Gt_times_Jt.block(ik, 0, 2, nv) =
          Gt[ic] * Jt.block(ik, 0, 2, nv);
    }
    // Form J = M + dt Jtᵀdiag(Gt)Jt:
    // TODO(amcastro-tri): Unit test Jacobian idependently.
    J = M + dt * Jt.transpose() * diag_Gt_times_Jt;

    // TODO(amcastro-tri): Consider using a cheap iterative solver like CG.
    // Since we are in a non-linear iteration, an approximate cheap solution
    // is probably best.
    // TODO(amcastro-tri): Consider using a matrix-free iterative method to
    // avoid computing M and J. CG and the Krylov family can be matrix-free.
    J_ldlt.compute(J);  // Update factorization.
    if (J_ldlt.info() != Eigen::Success) {
      return ComputationInfo::LinearSolverFailed;
    }
    Delta_v = J_ldlt.solve(-residual);

    // Since we keep Jt constant we have that:
    // vₜᵏ⁺¹ = Jt⋅vᵏ⁺¹ = Jt⋅(vᵏ + α Δvᵏ)
    //                = vₜᵏ + α Jt⋅Δvᵏ
    //                = vₜᵏ + α Δvₜᵏ
    // where we defined Δvₜᵏ = Jt⋅Δvᵏ and 0 < α < 1 is a constant that we'll
    // determine by limiting the maximum angle change between vₜᵏ and vₜᵏ⁺¹.
    // For multiple contact points, we choose the minimum α among all contact
    // points.
    Delta_vt = Jt * Delta_v;

    // Convergence is monitored in the tangential velocties.
    vt_error = Delta_vt.norm();

    // TODO(amcastro-tri): Limit the angle change between vₜᵏ⁺¹ and vₜᵏ for
    // all contact points. The angle change θ is defined by the dot product
    // between vₜᵏ⁺¹ and vₜᵏ as: cos(θ) = vₜᵏ⁺¹⋅vₜᵏ/(‖vₜᵏ⁺¹‖‖vₜᵏ‖).
    // We'll do so by computing a coefficient 0 < α < 1 so that if the
    // generalized velocities are updated as vᵏ⁺¹ = vᵏ + α Δvᵏ then θ < θₘₐₓ
    // for all contact points.
    T alpha = 1.0;  // We set α = 1 for now.
    v = v + alpha * Delta_v;
    vt = Jt * v;

    // Save iteration statistics.
    statistics_.Update(ExtractDoubleOrThrow(vt_error));
  }

  // If we are here is because we reached the maximum number of iterations
  // without converging to the specified tolerance.
  return ComputationInfo::MaxIterationsReached;
}

template <typename T>
T ImplicitStribeckSolver<T>::ModifiedStribeck(const T& x, const T& mu) {
  DRAKE_ASSERT(x >= 0);
  if (x >= 1) {
    return mu;
  } else {
    return mu * x * (2.0 - x);
  }
}

template <typename T>
T ImplicitStribeckSolver<T>::ModifiedStribeckDerivative(
    const T& x, const T& mu) {
  DRAKE_ASSERT(x >= 0);
  if (x >= 1) {
    return 0;
  } else {
    return mu * (2 * (1 - x));
  }
}

}  // namespace implicit_stribeck
}  // namespace multibody
}  // namespace drake

// Explicitly instantiates on the most common scalar types.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    struct
    ::drake::multibody::implicit_stribeck::internal::DirectionChangeLimiter)

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::implicit_stribeck::ImplicitStribeckSolver)
