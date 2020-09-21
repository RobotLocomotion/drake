#include "drake/multibody/plant/tamsi_solver.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/extract_double.h"

namespace drake {
namespace multibody {
namespace internal {
template <typename T>
T TalsLimiter<T>::CalcAlpha(
    const Eigen::Ref<const Vector2<T>>& v,
    const Eigen::Ref<const Vector2<T>>& dv,
    double cos_theta_max, double v_stiction, double relative_tolerance) {
  DRAKE_ASSERT(v_stiction > 0);
  DRAKE_ASSERT(relative_tolerance > 0);
  DRAKE_ASSERT(dv.size() == v.size());

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
  // it to land within the circle of radius v_stiction, at v_stiction/2 in the
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
    // x1 < 1: we go from within the stiction region back into it. Since this
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
    // intersects the boundary of the stiction region. For this case we
    // compute alpha so that the update corresponds to the velocity closest
    // to the origin.
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
    //  - line connecting v with v1 never goes through the stiction region.
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
bool TalsLimiter<T>::CrossesTheStictionRegion(
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
        // v_alpha falls within the stiction region but its magnitude is
        // larger than epsilon_v.
        return true;  // Crosses the stiction region.
      }
    }
  }
  return false;
}

template <typename T>
T TalsLimiter<T>::SolveQuadraticForTheSmallestPositiveRoot(
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
TamsiSolver<T>::TamsiSolver(int nv) :
    nv_(nv),
    fixed_size_workspace_(nv),
    // Provide an initial (arbitrarily large enough for most applications)
    // workspace size so that we avoid re-allocations afterwards as much as we
    // can.
    variable_size_workspace_(128, nv) {
  // We allow empty worlds, with a trivial solution.
  DRAKE_THROW_UNLESS(nv >= 0);
}

template <typename T>
void TamsiSolver<T>::SetOneWayCoupledProblemData(
    EigenPtr<const MatrixX<T>> M,
    EigenPtr<const MatrixX<T>> Jn, EigenPtr<const MatrixX<T>> Jt,
    EigenPtr<const VectorX<T>> p_star,
    EigenPtr<const VectorX<T>> fn, EigenPtr<const VectorX<T>> mu) {
  DRAKE_DEMAND(M && Jn && Jt && p_star && fn && mu);
  nc_ = fn->size();
  DRAKE_THROW_UNLESS(p_star->size() == nv_);
  DRAKE_THROW_UNLESS(M->rows() == nv_ && M->cols() == nv_);
  DRAKE_THROW_UNLESS(Jn->rows() == nc_ && Jn->cols() == nv_);
  DRAKE_THROW_UNLESS(Jt->rows() == 2 * nc_ && Jt->cols() == nv_);
  DRAKE_THROW_UNLESS(mu->size() == nc_);
  // Keep references to the problem data.
  problem_data_aliases_.SetOneWayCoupledData(M, Jn, Jt, p_star, fn, mu);
  variable_size_workspace_.ResizeIfNeeded(nc_, nv_);
}

template <typename T>
void TamsiSolver<T>::SetTwoWayCoupledProblemData(
    EigenPtr<const MatrixX<T>> M, EigenPtr<const MatrixX<T>> Jn,
    EigenPtr<const MatrixX<T>> Jt, EigenPtr<const VectorX<T>> p_star,
    EigenPtr<const VectorX<T>> fn0, EigenPtr<const VectorX<T>> stiffness,
    EigenPtr<const VectorX<T>> dissipation, EigenPtr<const VectorX<T>> mu) {
  DRAKE_DEMAND(M && Jn && Jt && p_star && fn0 && stiffness && dissipation
                   && mu);
  nc_ = fn0->size();
  DRAKE_THROW_UNLESS(p_star->size() == nv_);
  DRAKE_THROW_UNLESS(M->rows() == nv_ && M->cols() == nv_);
  DRAKE_THROW_UNLESS(Jn->rows() == nc_ && Jn->cols() == nv_);
  DRAKE_THROW_UNLESS(Jt->rows() == 2 * nc_ && Jt->cols() == nv_);
  DRAKE_THROW_UNLESS(mu->size() == nc_);
  DRAKE_THROW_UNLESS(stiffness->size() == nc_);
  DRAKE_THROW_UNLESS(dissipation->size() == nc_);
  // Keep references to the problem data.
  problem_data_aliases_.SetTwoWayCoupledData(M, Jn, Jt, p_star, fn0, stiffness,
                                             dissipation, mu);
  variable_size_workspace_.ResizeIfNeeded(nc_, nv_);
}

template <typename T>
void TamsiSolver<T>::CalcFrictionForces(
    const Eigen::Ref<const VectorX<T>>& vt,
    const Eigen::Ref<const VectorX<T>>& fn,
    EigenPtr<VectorX<T>> v_slip_ptr,
    EigenPtr<VectorX<T>> t_hat_ptr,
    EigenPtr<VectorX<T>> mu_regularized_ptr,
    EigenPtr<VectorX<T>> ft) const {
  using std::sqrt;

  const int nc = nc_;  // Number of contact points.

  // Aliases to vector of friction coefficients.
  const auto& mu = problem_data_aliases_.mu();

  // Convenient aliases.
  auto mu_regularized = *mu_regularized_ptr;
  auto v_slip = *v_slip_ptr;
  auto t_hat = *t_hat_ptr;

  // The stiction tolerance.
  const double v_stiction = parameters_.stiction_tolerance;

  // We use the stiction tolerance as a reference scale to estimate a small
  // velocity epsilon_v. With v_epsilon we define a "soft norm" which we
  // use to compute "soft" tangent vectors to avoid a division by zero
  // singularity when tangential velocities are zero.
  const double epsilon_v = v_stiction * parameters_.relative_tolerance;
  const double epsilon_v2 = epsilon_v * epsilon_v;

  // Compute 2D tangent vectors.
  // To avoid the singularity at v_slip = ‖vt‖ = 0 we use a "soft norm". The
  // idea is to replace the norm in the definition of slip velocity by a
  // "soft norm":
  //    ‖v‖ₛ ≜ sqrt(vᵀv + εᵥ²)
  // (in code εᵥ is named epsilon_v and εᵥ² is named epsilon_v2). We use
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
    mu_regularized(ic) = RegularizedFriction(v_slip(ic) / v_stiction, mu(ic));
    // Friction force.
    ft->template segment<2>(ik) = -mu_regularized(ic) * that_ic * fn(ic);
  }
}

template <typename T>
void TamsiSolver<T>::CalcFrictionForcesGradient(
    const Eigen::Ref<const VectorX<T>>& fn,
    const Eigen::Ref<const VectorX<T>>& mu_vt,
    const Eigen::Ref<const VectorX<T>>& t_hat,
    const Eigen::Ref<const VectorX<T>>& v_slip,
    std::vector<Matrix2<T>>* dft_dvt_ptr) const {

  const int nc = nc_;  // Number of contact points.

  // Problem data.
  const auto& mu = problem_data_aliases_.mu();

  // Mutable reference to ∇ᵥₜfₜ(vₜ).
  std::vector<Matrix2<T>>& dft_dvt = *dft_dvt_ptr;

  // The stiction tolerance.
  const double v_stiction = parameters_.stiction_tolerance;

  // Compute dft/dvt, a 2x2 matrix with the derivative of the friction
  // force (in ℝ²) with respect to the tangent velocity (also in ℝ²).
  for (int ic = 0; ic < nc; ++ic) {
    const int ik = 2 * ic;

    // Compute dmu/dv = (1/v_stiction) * dmu/dx
    // where x = v_slip / v_stiction is the dimensionless slip velocity.
    const T x = v_slip(ic) / v_stiction;
    const T dmudv = RegularizedFrictionDerivative(x, mu(ic)) / v_stiction;

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
    //   ∇ᵥₜfₜ = −dft_dvt = −fn * (
    //     mu_regularized(‖vₜ‖ₛ) / ‖vₜ‖ₛ * Pperp(t̂) +
    //     dmu_regularized/dx * P(t̂) / v_stiction )
    // where x = ‖vₜ‖ₛ / vₛ is the dimensionless slip velocity and we
    // have defined dft_dvt = −∇ᵥₜfₜ.
    // Therefore dft_dvt (in ℝ²ˣ²) is a linear combination of PSD matrices
    // (P and Pperp) where the coefficients of the combination are positive
    // scalars. Therefore,
    // IMPORTANT NOTE: dft_dvt is also PSD.
    // IMPORTANT NOTE 2: The derivation for dft_dvt leads to exactly the
    // same result when using the "softened" definitions for v_slip and
    // t_hat where each occurrence of these quantities is replaced by its
    // softened counterpart.

    // Compute dft_dvt:
    // Changes of vt in the direction perpendicular to t_hat (see the full
    // expression for dft_dvt above).
    dft_dvt[ic] = Pperp_ic * mu_vt(ic) / v_slip(ic);

    // Changes in the magnitude of vt (which in turn makes mu_regularized
    // change), in the direction of t_hat.
    dft_dvt[ic] += P_ic * dmudv;

    // Note: dft_dvt is a symmetric 2x2 matrix. This will imply the positive
    // definiteness of the system's Jacobian below.
    dft_dvt[ic] *= fn(ic);
  }
}

template <typename T>
void TamsiSolver<T>::CalcNormalForces(
    const Eigen::Ref<const VectorX<T>>& vn,
    const Eigen::Ref<const MatrixX<T>>& Jn,
    double dt,
    // We change from fn/Gn in the header to fn_ptr, Gn_ptr here to avoid name
    // clashes with local variables.
    EigenPtr<VectorX<T>> fn_ptr,
    EigenPtr<MatrixX<T>> Gn_ptr) const {
  using std::max;
  const int nc = nc_;  // Number of contact points.

  if (!has_two_way_coupling()) {
    // Copy the input normal force (i.e. it is fixed).
    *fn_ptr = problem_data_aliases_.fn();
    return;
  }

  // Convenient aliases to problem data.
  const auto& fn0 = problem_data_aliases_.fn0();
  const auto& stiffness = problem_data_aliases_.stiffness();
  const auto& dissipation = problem_data_aliases_.dissipation();

  // Below we use the following notation:
  //   (x)₊ =  max(x, 0),
  //   H(x) = d(x)₊/dx, the Heaviside function.
  // We define the undamped normal force as undamped_fn = (fₙ₀ − h k vₙ)₊.
  VectorX<T> undamped_fn(nc);
  // We define damping_factor = (1 − d vₙ)₊.
  VectorX<T> damping_factor(nc);
  VectorX<T> H_damping_factor(nc);  // = H(1 − d vₙ)
  VectorX<T> H_undamped_fn(nc);     // = H(fₙ₀ − h k vₙ)

  auto& fn = *fn_ptr;
  for (int ic = 0; ic < nc; ++ic) {
    const T signed_damping_factor = (1.0 - dissipation(ic) * vn(ic));
    damping_factor(ic) = max(0.0, signed_damping_factor);

    // fₙ = (1 − d vₙ)₊ (fₙ₀ − h k vₙ)₊
    const T signed_undamped_fn = fn0(ic) - dt * stiffness(ic) * vn(ic);
    undamped_fn(ic) = max(0.0, signed_undamped_fn);
    fn(ic) = damping_factor(ic) * undamped_fn(ic);

    // Factors in the derivatives of damping factor (1 − d vₙ)₊ and the
    // undamped normal force defined as (fₙ₀ − h k vₙ)₊.
    H_damping_factor(ic) = signed_damping_factor >= 0 ? 1.0 : 0.0;
    H_undamped_fn(ic) = signed_undamped_fn >= 0 ? 1.0 : 0.0;
  }

  // We simply use the product rule to compute the derivative of fₙ as a
  // function of vₙ (on a per contact basis, that's why the use of .array()
  // operations below). dfₙ/dvₙ = -d⋅H(1 − d vₙ)⋅(fₙ₀ − h k vₙ)₊  - h⋅k⋅(1 − d
  // vₙ)₊⋅H(fₙ₀ − h k vₙ) Note that dfₙ/dvₙ < 0 always.
  const VectorX<T> dfn_dvn = -(
      dissipation.array() * H_damping_factor.array() * undamped_fn.array() +
      dt * stiffness.array() * damping_factor.array() * H_undamped_fn.array());

  // We use the chain rule to compute Gn = ∇ᵥfₙ. Since ∇ᵥvₙ = Jn, we have:
  auto& Gn = *Gn_ptr;
  Gn = dfn_dvn.asDiagonal() * Jn;
}

template <typename T>
void TamsiSolver<T>::CalcJacobian(
    const Eigen::Ref<const MatrixX<T>>& M,
    const Eigen::Ref<const MatrixX<T>>& Jn,
    const Eigen::Ref<const MatrixX<T>>& Jt,
    const Eigen::Ref<const MatrixX<T>>& Gn,
    const std::vector<Matrix2<T>>& dft_dvt,
    const Eigen::Ref<const VectorX<T>>& t_hat,
    const Eigen::Ref<const VectorX<T>>& mu_vt, double dt,
    EigenPtr<MatrixX<T>> J) const {
  // Problem sizes.
  const int nv = nv_;  // Number of generalized velocities.
  const int nc = nc_;  // Number of contact points.
  // Size of the friction forces vector ft and tangential velocities vector vt.
  const int nf = 2 * nc;

  // Newton-Raphson Jacobian, i.e. the derivative of the residual with
  // respect to the independent variable which, in this case, is the vector
  // of generalized velocities vˢ⁺¹ at the next time step. We just use v for
  // brevity here.
  // Analytical differentiation of the residual with respect to v leads to:
  //   J = ∇ᵥR = M − δt Jₙᵀ Gn − δt Jₜᵀ Gt
  // where Gn = Gn = ∇ᵥfₙ(∇ᵥfₙ(x(v), vₙ(v))) (of size nc x nv) and
  // Gt = ∇ᵥfₜ(vₜ(v)) (of size 2nc x nv) are the gradients with respect to v
  // of the normal and friction forces, respectively. The gradient of the normal
  // forces is an input to this method while the gradient of the tangential
  // forces can be computed in terms of dft_dvt and Gn as:
  //   Gt = ∇ᵥfₜ = −diag(dft_dvt) Jₜ - Gfn(ft) Jₙ
  // recall that dft_dvt = −∇ᵥₜfₜ so that dft_dvt is defined PSD.
  // For each contact point dft_dvt is a 2x2 PSD matrix. diag(dft_dvt) is the
  // (2nc x 2nc) block diagonal matrix with dft_dvt in each 2x2 diagonal entry.
  // Gfn(ft) is the gradient of the friction forces with respect to the normal
  // forces. Thus, Gfn(ft) Jₙ is nothing but the chain rule of differentiation
  // to compute the contribution to the gradient ∇ᵥfₜ with respect to v, due to
  // the functional dependence of the normal forces with v.
  // Notice that Gfn(ft) is zero for the one-way coupled scheme.

  // Compute Gt = −∇ᵥfₜ (gradient of the friction forces with respect to the
  // generalized velocities) as Gt = −diag(dft_dvt) Jt and use the fact that
  // diag(dft_dvt) is block diagonal.
  MatrixX<T> Gt(nf, nv);  // −∇ᵥfₜ
  for (int ic = 0; ic < nc; ++ic) {  // Index ic scans contact points.
    const int ik = 2 * ic;  // Index ik scans contact vector quantities.
    Gt.block(ik, 0, 2, nv) =
        -dft_dvt[ic] * Jt.block(ik, 0, 2, nv);

    // Add Contribution from Gn = ∇ᵥfₙ(xˢ⁺¹, vₙˢ⁺¹). Only for the two-way
    // coupled scheme.
    if (has_two_way_coupling()) {
      auto& t_hat_ic = t_hat.template segment<2>(ik);
      Gt.block(ik    , 0, 1, nv) -=
          mu_vt(ic) * t_hat_ic(0) * Gn.block(ic, 0, 1, nv);
      Gt.block(ik + 1, 0, 1, nv) -=
          mu_vt(ic) * t_hat_ic(1) * Gn.block(ic, 0, 1, nv);
    }
  }

  // Form J = M − Jnᵀ Gn − dt Jtᵀ Gt:
  *J = M - dt * Jt.transpose() * Gt;
  if (has_two_way_coupling()) {
    *J -= dt * Jn.transpose() * Gn;
  }
}

template <typename T>
T TamsiSolver<T>::CalcAlpha(
    const Eigen::Ref<const VectorX<T>>& vt,
    const Eigen::Ref<const VectorX<T>>& Delta_vt) const {
  using std::min;
  T alpha = 1.0;
  double v_stiction = parameters_.stiction_tolerance;
  for (int ic = 0; ic < nc_; ++ic) {  // Index ic scans contact points.
    const int ik = 2 * ic;  // Index ik scans contact vector quantities.
    auto vt_ic = vt.template segment<2>(ik);
    const auto dvt_ic = Delta_vt.template segment<2>(ik);
    alpha = min(
        alpha,
        internal::TalsLimiter<T>::CalcAlpha(
            vt_ic, dvt_ic,
            cos_theta_max_, v_stiction, parameters_.relative_tolerance));
  }
  DRAKE_DEMAND(0 < alpha && alpha <= 1.0);
  return alpha;
}

template <typename T>
TamsiSolverResult TamsiSolver<T>::SolveWithGuess(
    double dt, const VectorX<T>& v_guess) const {
  DRAKE_THROW_UNLESS(v_guess.size() == nv_);

  // Clear statistics so that we can update them with new ones for this call to
  // SolveWithGuess().
  statistics_.Reset();

  // If there are no contact points return a zero generalized friction force
  // vector, i.e. tau_f = 0.
  if (nc_ == 0) {
    fixed_size_workspace_.mutable_tau_f().setZero();
    fixed_size_workspace_.mutable_tau().setZero();
    const auto M = problem_data_aliases_.M();
    const auto p_star = problem_data_aliases_.p_star();
    auto& v = fixed_size_workspace_.mutable_v();
    // With no friction forces Eq. (3) in the documentation reduces to
    // M vˢ⁺¹ = p*.
    v = M.ldlt().solve(p_star);
    // "One iteration" with exactly "zero" vt_error.
    statistics_.Update(0.0);
    return TamsiSolverResult::kSuccess;
  }

  // Solver parameters.
  const int max_iterations = parameters_.max_iterations;
  // Tolerance used to monitor the convergence of the contact velocities in both
  // normal and tangential directions.
  // TODO(amcastro-tri): consider the monitoring of generalized velocities
  // directly. However in that case a proper scaling must be devised.
  const double v_contact_tolerance =
      parameters_.relative_tolerance * parameters_.stiction_tolerance;

  // Convenient aliases to problem data.
  const auto M = problem_data_aliases_.M();
  const auto Jn = problem_data_aliases_.Jn();
  const auto Jt = problem_data_aliases_.Jt();
  const auto p_star = problem_data_aliases_.p_star();

  // Convenient aliases to fixed size workspace variables.
  auto& v = fixed_size_workspace_.mutable_v();
  auto& Delta_v = fixed_size_workspace_.mutable_Delta_v();
  auto& residual = fixed_size_workspace_.mutable_residual();
  auto& J = fixed_size_workspace_.mutable_J();
  auto& tau_f = fixed_size_workspace_.mutable_tau_f();
  auto& tau = fixed_size_workspace_.mutable_tau();

  // Convenient aliases to variable size workspace variables.
  // Note: auto resolve to Eigen::Block (no copies).
  auto vn = variable_size_workspace_.mutable_vn();
  auto vt = variable_size_workspace_.mutable_vt();
  auto ft = variable_size_workspace_.mutable_ft();
  auto Delta_vn = variable_size_workspace_.mutable_Delta_vn();
  auto Delta_vt = variable_size_workspace_.mutable_Delta_vt();
  auto& dft_dvt = variable_size_workspace_.mutable_dft_dvt();
  auto Gn = variable_size_workspace_.mutable_Gn();
  auto mu_vt = variable_size_workspace_.mutable_mu();
  auto t_hat = variable_size_workspace_.mutable_t_hat();
  auto fn = variable_size_workspace_.mutable_fn();
  auto v_slip = variable_size_workspace_.mutable_v_slip();

  // Initialize vt_error to an arbitrary value larger than tolerance so that the
  // solver at least performs one iteration.
  double vt_error = 2 * v_contact_tolerance;
  double vn_error = 2 * v_contact_tolerance;

  // Initialize iteration with the guess provided.
  v = v_guess;

  for (int iter = 0; iter < max_iterations; ++iter) {
    // Update normal and tangential velocities.
    vn = Jn * v;
    vt = Jt * v;

    CalcNormalForces(vn, Jn, dt, &fn, &Gn);

    // Update v_slip, t_hat, mus and ft as a function of vt and fn.
    CalcFrictionForces(vt, fn, &v_slip, &t_hat, &mu_vt, &ft);

    // After the previous iteration, we allow updating ft above to have its
    // latest value before leaving.
    // Convergence is monitored in both tangential and normal directions.
    if (std::max(vt_error, vn_error) < v_contact_tolerance) {
      // Update generalized forces and return.
      tau_f = Jt.transpose() * ft;
      tau = tau_f + Jn.transpose() * fn;
      return TamsiSolverResult::kSuccess;
    }

    // Newton-Raphson residual.
    residual =
        M * v - p_star - dt * Jn.transpose() * fn - dt * Jt.transpose() * ft;

    // Compute gradient dft_dvt = ∇ᵥₜfₜ(vₜ) as a function of fn, mus,
    // t_hat and v_slip.
    CalcFrictionForcesGradient(fn, mu_vt, t_hat, v_slip, &dft_dvt);

    // Newton-Raphson Jacobian, J = ∇ᵥR, as a function of M, dft_dvt, Jt, dt.
    CalcJacobian(M, Jn, Jt, Gn, dft_dvt, t_hat, mu_vt, dt, &J);

    // TODO(amcastro-tri): Consider using a cheap iterative solver like CG.
    // Since we are in a non-linear iteration, an approximate cheap solution
    // is probably best.
    // TODO(amcastro-tri): Consider using a matrix-free iterative method to
    // avoid computing M and J. CG and the Krylov family can be matrix-free.
    if (has_two_way_coupling()) {
      auto& J_lu = fixed_size_workspace_.mutable_J_lu();
      J_lu.compute(J);  // Update factorization.
      Delta_v = J_lu.solve(-residual);
    } else {
      auto& J_ldlt = fixed_size_workspace_.mutable_J_ldlt();
      J_ldlt.compute(J);  // Update factorization.
      if (J_ldlt.info() != Eigen::Success) {
        return TamsiSolverResult::kLinearSolverFailed;
      }
      Delta_v = J_ldlt.solve(-residual);
    }

    // Since we keep Jt constant we have that:
    // vₜᵏ⁺¹ = Jt vᵏ⁺¹ = Jt (vᵏ + α Δvᵏ)
    //                 = vₜᵏ + α Jt Δvᵏ
    //                 = vₜᵏ + α Δvₜᵏ
    // where we defined Δvₜᵏ = Jt Δvᵏ and 0 < α < 1 is a constant that we'll
    // determine by limiting the maximum angle change between vₜᵏ and vₜᵏ⁺¹.
    // For multiple contact points, we choose the minimum α among all contact
    // points.
    Delta_vt = Jt * Delta_v;

    // Similarly to Δvₜᵏ above, we define the update in the normal velocities
    // as Δvₙᵏ = Jₙ Δvᵏ.
    Delta_vn = Jn * Delta_v;

    // We monitor convergence in both normal and tangential velocities.
    vn_error = ExtractDoubleOrThrow(Delta_vn.norm());
    vt_error = ExtractDoubleOrThrow(Delta_vt.norm());

    // Limit the angle change between vₜᵏ⁺¹ and vₜᵏ for all contact points.
    T alpha = CalcAlpha(vt, Delta_vt);

    // Update generalized velocity vector.
    v = v + alpha * Delta_v;

    // Save iteration statistics.
    statistics_.Update(vt_error);
  }

  // If we are here is because we reached the maximum number of iterations
  // without converging to the specified tolerance.
  return TamsiSolverResult::kMaxIterationsReached;
}

template <typename T>
T TamsiSolver<T>::RegularizedFriction(const T& s, const T& mu) {
  DRAKE_ASSERT(s >= 0);
  if (s >= 1) {
    return mu;
  } else {
    return mu * s * (2.0 - s);
  }
}

template <typename T>
T TamsiSolver<T>::RegularizedFrictionDerivative(
    const T& s, const T& mu) {
  DRAKE_ASSERT(s >= 0);
  if (s >= 1) {
    return 0;
  } else {
    return mu * (2 * (1 - s));
  }
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    struct ::drake::multibody::internal::TalsLimiter)

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::TamsiSolver)
