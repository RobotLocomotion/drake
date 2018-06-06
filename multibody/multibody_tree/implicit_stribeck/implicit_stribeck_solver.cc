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
T LimitDirectionChange<T>::run(
    const Eigen::Ref<const Vector2<T>>& v,
    const Eigen::Ref<const Vector2<T>>& dv,
    double cos_min, double v_stribeck, double tolerance) {
  DRAKE_DEMAND(dv.size() == v.size());

  using std::abs;
  using std::sqrt;

  const double epsilon_v = v_stribeck * tolerance;
  const double epsilon_v2 = epsilon_v * epsilon_v;

  const Vector2<T> v1 = v + dv;  // v_alpha = v + dv, when alpha = 1.

  // Case I: Quick exit for small changes in v.
  const T dv_norm2 = dv.squaredNorm();
  if (dv_norm2 < epsilon_v2) {
    return 1.0;
  }

  const T v_norm = v.norm();
  const T v1_norm = v1.norm();
  const T x = v_norm / v_stribeck;    // Dimensionless slip v and v1.
  const T x1 = v1_norm / v_stribeck;
  // From Case I, we know dv_norm > epsilon_v.
  const T dv_norm = sqrt(dv_norm2);

  // Case II: limit transition from stiction to sliding when x << 1.0 and
  // gradients might be close to zero (due to the "soft norms").
  if (x < tolerance && x1 > 1.0) {
    // we know v1 != 0  since x1 > 1.0.
    // We make |v_alpha| = v_stribeck / 2.
    // For this case dv ≈ v1 (v ≈ 0). Therefore:
    // alpha ≈ v_stribeck / dv_norm / 2.
    return v_stribeck / dv_norm / 2.0;
  }

  // Another quick exit.
  if (x < 1.0) {
    // Since we went through Case II, we know that either:
    //   1. x1 < 1.0. Then we just make alpha = 1.0
    //   2. x > parameters_.tolerance i.e. we are in the zone of
    //      "strong gradients" where it is safe to take alpha = 1.0.
    return 1.0;
  } else {  // x > 1.0
    // We want to detect transition from sliding (x > 1) to stiction when the
    // velocity change passes through the circle of radius v_stribeck.

    if (x1 < 1.0) {
      // Transition happens with alpha = 1.0.
      return 1.0;
    }

    // Since v_stribeck is so small, this very seldom happens. However, it is a
    // very common case for 1D-like problems for which tangential velocities
    // change in sign and typically if we don't do anything we miss the zero
    // crossing.
    // Notice that since we passed the check in Case I, we know that:
    //  - x > 1.0
    //  - x1 > 1.0
    //  - dv_norm > epsilon_v (i.e. non-zero)
    const T v_dot_dv = v.dot(dv);
    if (v_dot_dv < 0.0) {  // Moving towards the origin.
      T alpha = -v_dot_dv / dv_norm2;  // alpha > 0
      if (alpha < 1.0) {  // we might have missed a cross by zero. Check.
        const Vector2<T> v_alpha = v + alpha * dv;  // v1.dot(v) = 0.
        const T v_alpha_norm = v_alpha.norm();
        if (v_alpha_norm < v_stribeck) {
          return alpha;
        }
      }
    }

    // If we are here we know:
    //  - x > 1.0
    //  - x1 > 1.0
    //  - dv_norm > epsilon_v
    //  - line connecting v with v1 never goes through circle of radius
    //    v_stribeck.
    //
    // Case III:
    // Therefore we know changes happen entirely outside the circle of radius
    // v_stribeck. To avoid large jumps in the direction of v (typically during
    // strong impacts), we limit the maximum angle change between v to v1.
    // To this end, we find a scalar 0 < alpha < 1 such that
    // cos(theta_max) = v⋅v1/(‖v‖‖v1‖).

    // First we compute the angle change when alpha = 1, between v1 and v.
    const T cos1 = v.dot(v1) / v_norm / v1_norm;

    // We allow angle changes theta < theta_max, and we take alpha = 1.0.
    // In particular, when v1 is exactly aligned wiht v (but we know it does not
    // cross through zero, i.e. cos(theta) > 0).
    if (cos1 > cos_min) {
      return 1.0;
    } else {  // we limit the angle change to theta_max.
      // All term are made non-dimensional using v_stribeck as the reference
      // sale.
      const T x_dot_dx = v_dot_dv / (v_stribeck * v_stribeck);
      const T dx = dv.norm() / v_stribeck;
      const T dx2 = x * x;
      const T dx4 = dx2 * dx2;
      const T cmin2 = cos_min * cos_min;

      // Form the terms of the quadratic equation aα² + bα + c = 0.
      const T a = dx2 * dx * dx * cmin2 - x_dot_dx * x_dot_dx;
      const T b = 2 * dx2 * x_dot_dx * (cmin2 - 1.0);
      const T c = dx4 * (cmin2 - 1.0);

      // Solve quadratic equation. We know, from the geometry of the problem,
      // that the roots to this problem are real. Thus, the discriminant of the
      // quadratic equation (Δ = b² - 4ac) is positive.
      // Also, unless there is a single root (a = 0), they have different signs.
      // From Vieta's formulas we know:
      //   - α₁ + α₂ = -b / a
      //   - α₁ * α₂ = c / a < 0
      // Since we know that c < 0 (strictly), then we must have a ≥ 0.
      // Also since c < 0 strictly, α = 0 (zero) cannot be a root.
      // We use this knowledge in the solution below.

      T alpha;
      // First determine if a = 0 (to machine epsilon).
      if (abs(a) < std::numeric_limits<double>::epsilon()) {
        // There is only a single root to the, now linear, equation bα + c = 0.
        alpha = -c / b;
        // Note: a = 0, α > 0 => x_dot_dx = x * dx * cmin ≠ 0 => b ≠ 0
      } else {
        // The determinant of the quadratic equation.
        const T Delta = b * b - 4 * a * c;
        // Geometry tell us that a real solution does exist i.e. Delta > 0.
        DRAKE_ASSERT(Delta > 0);
        const T sqrt_delta = sqrt(Delta);
        alpha = (-b + sqrt_delta) / a / 2.0;  // we know a > 0
      }

      // The geometry of the problem tells us that α ≤ 1.0
      DRAKE_ASSERT(alpha <= 1.0);
      return alpha;
    }
  }

  // We should never reach this point.
  throw std::logic_error("Bug detected. An angle change case was missed.");
}
}

template <typename T>
ImplicitStribeckSolver<T>::ImplicitStribeckSolver(int nv) :
    nv_(nv),
    fixed_size_workspace_(nv),
    // Provide an initial workspace size so that we avoid re-allocations
    // afterwards as much as we can.
    variable_size_workspace_(128) {
  DRAKE_THROW_UNLESS(nv > 0);
  using std::cos;
  // Precompute cos(theta_max).
  cos_min_ = cos(parameters_.theta_max);
}

template <typename T>
void ImplicitStribeckSolver<T>::SetProblemData(
    EigenPtr<const MatrixX<T>> M, EigenPtr<const MatrixX<T>> D,
    EigenPtr<const VectorX<T>> p_star,
    EigenPtr<const VectorX<T>> fn, EigenPtr<const VectorX<T>> mu) {
  nc_ = fn->size();
  DRAKE_THROW_UNLESS(p_star->size() == nv_);
  DRAKE_THROW_UNLESS(M->rows() == nv_ && M->cols() == nv_);
  DRAKE_THROW_UNLESS(D->rows() == 2 * nc_ && D->cols() == nv_);
  DRAKE_THROW_UNLESS(mu->size() == nc_);
  // Keep references to the problem data.
  problem_data_aliases_.Set(M, D, p_star, fn, mu);
  variable_size_workspace_.ResizeIfNeeded(nc_);
}

template <typename T>
T ImplicitStribeckSolver<T>::LimitDirectionChange(
    const Eigen::Ref<const Vector2<T>>& v,
    const Eigen::Ref<const Vector2<T>>& dv) const {
  return internal::LimitDirectionChange<T>::run(
      v, dv, cos_min_, parameters_.stiction_tolerance, parameters_.tolerance);
}

template <typename T>
ComputationInfo ImplicitStribeckSolver<T>::SolveWithGuess(
    double dt, const VectorX<T>& v_guess) {
  DRAKE_THROW_UNLESS(v_guess.size() == nv_);

  using std::abs;
  using std::max;
  using std::min;
  using std::sqrt;

  // If there are no contact points return a zero generalized friction forces
  // vector, i.e. tau_f = 0.
  if (nc_ == 0) {
    fixed_size_workspace_.tau_f.setZero();
    return ComputationInfo::Success;
  }

  // Solver parameters.
  const int max_iterations = parameters_.max_iterations;
  // Tolerance used to monitor the convergence of the tangential velocities.
  const double vt_tolerance =
      parameters_.tolerance * parameters_.stiction_tolerance;

  // Problem sizes.
  const int nv = nv_;  // Number of generalized velocities.
  const int nc = nc_;  // Number of contact points.
  // Size of the friction forces vector ft and tangential velocities vector vt.
  const int nf = 2 * nc;

  // Convenient aliases to problem data.
  const auto& M = *problem_data_aliases_.M_ptr;
  const auto& D = *problem_data_aliases_.D_ptr;
  const auto& p_star = *problem_data_aliases_.p_star_ptr;
  const auto& fn = *problem_data_aliases_.fn_ptr;
  const auto& mu = *problem_data_aliases_.mu_ptr;

  // Convenient aliases to fixed size workspace variables.
  auto& v = fixed_size_workspace_.v;
  auto& Delta_v = fixed_size_workspace_.Delta_v;
  auto& R = fixed_size_workspace_.R;
  auto& J = fixed_size_workspace_.J;
  auto& J_ldlt = *fixed_size_workspace_.J_ldlt;
  auto& tau_f = fixed_size_workspace_.tau_f;

  // Convenient aliases to variable size workspace variables.
  auto vt = variable_size_workspace_.mutable_vt();
  auto ft = variable_size_workspace_.mutable_ft();
  auto Delta_vt = variable_size_workspace_.mutable_delta_vt();
  auto dft_dv = variable_size_workspace_.mutable_dft_dv();
  auto mus = variable_size_workspace_.mutable_mu();
  auto that = variable_size_workspace_.mutable_that();
  auto v_slip = variable_size_workspace_.mutable_v_slip();

  // Initialize residual to a value larger than tolerance so that the solver at
  // least performs one iteration
  T residual = 2 * vt_tolerance;

  // Initialize iteration with the provided guess.
  v = v_guess;
  vt = D * v;

  // The stiction tolerance.
  const double v_stribeck = parameters_.stiction_tolerance;

  // We use the stiction tolerance as a reference scale to estimate a small
  // velocity v_epsilon. With v_epsilon we define a "soft norm" which we
  // use to compute "soft" tangent vectors to avoid a division by zero
  // singularity when tangential velocities are zerl.
  const double epsilon_v = v_stribeck * 1.0e-4;
  const double epsilon_v2 = epsilon_v * epsilon_v;

  // Clear statistics so that we can update them with new ones for this call to
  // SolveWithGuess().
  statistics_.Reset();

  for (int iter = 0; iter < max_iterations; ++iter) {
    // Compute 2D tangent vectors.
    // To avoid the singularity at v_slip = ‖vt‖ = 0 we use a "soft norm". The
    // idea is to replace the norm in the definition of slip velocity by a
    // "soft norm":
    //    ‖v‖ ≜ sqrt(vᵀv + εᵥ²)
    // We use this to redefine the slip velocity:
    //   v_slip = sqrt(vtᵀvt + v_epsilon)
    // and a "soft" tangent vector:
    //   t̂ = vₜ / sqrt(vₜᵀvₜ + εᵥ²)
    // which now is not only well defined but it has well defined derivatives.
    // We use these softened quantities all throuout our derivations for
    // consistency.
    for (int ic = 0; ic < nc; ++ic) {
      const int ik = 2 * ic;
      const auto vt_ic = vt.template segment<2>(ik);
      // "soft norm":
      v_slip(ic) = sqrt(vt_ic.squaredNorm() + epsilon_v2);
      // "soft" tangent vector:
      const Vector2<T> that_ic = vt_ic / v_slip(ic);
      that.template segment<2>(ik) = that_ic;
      mus(ic) = ModifiedStribeck(v_slip(ic) / v_stribeck, mu(ic));
      // Friction force.
      // Note: minus sign not included in this definition.
      ft.template segment<2>(ik) = mus(ic) * that_ic * fn(ic);
    }

    // After the previous iteration, we allow updating ft above to have its
    // latest value before leaving.
    if (residual < vt_tolerance) {
      // Update generalized forces vector and return.
      tau_f = D.transpose() * ft;
      return ComputationInfo::Success;
    }

    // Newton-Raphson residual
    R = M * v - p_star + dt * D.transpose() * ft;

    // Compute dft/dvt, a 2x2 matrix with the derivative of the friction
    // force (in ℝ²) with respect to the tangent velocity (also in ℝ²).
    for (int ic = 0; ic < nc; ++ic) {
      const int ik = 2 * ic;

      // Compute dmu/dv = (1/v_stribeck) * dmu/dx
      // where x = v_slip / v_stribeck is the dimensionless slip velocity.
      const T dmudv = ModifiedStribeckPrime(
          v_slip(ic) / v_stribeck, mu(ic)) / v_stribeck;

      const auto that_ic = that.template segment<2>(ik);

      // Projection matrix. It projects in the direction of that.
      // Notice it is a symmetric 2x2 matrix.
      const Matrix2<T> P_ic = that_ic * that_ic.transpose();

      // Removes the projected direction along that.
      // This is also a symmetric 2x2 matrix.
      const Matrix2<T> Pperp_ic = Matrix2<T>::Identity() - P_ic;

      // Some notes about projection matrices P:
      //  - They are symmetric, positive semi-definite.
      //  - All their eigenvalues are either one or zero.
      //  - Their rank equals the number of non-zero eigenvales.
      //  - From the previous item we have rank(P) = trace(P).
      //  - If P is a projection matrix, so is (I - P).
      // From the above we then know that P and Pperp are both projection
      // matrices of rank one (i.e. rank deficient) and are symmetric
      // semi-positive definite. This has very important consequences for the
      // Jacobian of the residual.

      // We now compute dft/dvt as:
      //   dft/dvt = fn * (
      //     mu_stribeck(‖vₜ‖) / ‖vₜ‖ * Pperp(t̂) +
      //     dmu_stribeck/dx * P(t̂) / v_stribeck )
      // where x = v_slip / v_stribeck is the dimensionless slip velocity.
      // Therefore dft/dvt (in ℝ²ˣ²) is a linear combination of PSD matrices
      // (P and Pperp) where the coefficients of the combination are positive
      // scalars. Therefore,
      // IMPORTANT NOTE: dft/dvt also PSD.
      // IMPORTANT NOTE 2: The derivation for dft/dvt leads to exactly the
      // same result when using the "softened" definitions for v_slip and
      // that where each occurrence of these quantities is replaced by their
      // softened counterpart.

      // Compute dft/dv:
      // Changes of vt in the direction perpendicular to that.
      dft_dv[ic] = Pperp_ic * mus(ic) / v_slip(ic);

      // Changes in the magnitude of vt (which in turns makes mu_stribeck
      // change), in the direction of that.
      dft_dv[ic] += P_ic * dmudv;

      // Note: dft_dv is a symmetric 2x2 matrix.
      dft_dv[ic] *= fn(ic);
    }

    // Newton-Raphson Jacobian:
    //  J = I + dt M⁻¹Dᵀdiag(dfₜ/dvₜ)D
    // J is an (nv x nv) symmetric positive definite matrix.
    // diag(dfₜ/dvₜ) is the (2nc x 2nc) block diagonal matrix with dfₜ/dvₜ
    // in each 2x2 diagonal entry.

    // Start by multiplying diag(dfₜ/dvₜ)D and use the fact that
    // diag(dfₜ/dvₜ) is block diagonal.
    MatrixX<T> diag_dftdv_times_D(nf, nv);
    // TODO(amcastro-tri): Only build half of the matrix since it is
    // symmetric.
    for (int ic = 0; ic < nc; ++ic) {
      const int ik = 2 * ic;
      diag_dftdv_times_D.block(ik, 0, 2, nv) =
          dft_dv[ic] * D.block(ik, 0, 2, nv);
    }
    // Form J = M + dt Dᵀdiag(dfₜ/dvₜ)D:
    J = M + dt * D.transpose() * diag_dftdv_times_D;

    // TODO(amcastro-tri): Consider using a cheap iterative solver like CG.
    // Since we are in a non-linear iteration, an approximate cheap solution
    // is probably best.
    // TODO(amcastro-tri): Consider using a matrix-free iterative method to
    // avoid computing M and J. CG and the Krylov family can be matrix-free.
    J_ldlt.compute(J);  // Update factorization.
    Delta_v = J_ldlt.solve(-R);
    if (J_ldlt.info() != Eigen::Success) {
      return ComputationInfo::LinearSolverFailed;
    }

    // Since we keep D constant we have that:
    // vₜᵏ⁺¹ = D⋅vᵏ⁺¹ = D⋅(vᵏ + α Δvᵏ)
    //                = vₜᵏ + α D⋅Δvᵏ
    //                = vₜᵏ + α Δvₜᵏ
    // where we defined Δvₜᵏ = D⋅Δvᵏ and 0 < α < 1 is a constant that we'll
    // determine by limiting the maximum angle change between vₜᵏ and vₜᵏ⁺¹.
    // For multiple contact points, we choose the minimum α amongs all contact
    // points.
    Delta_vt = D * Delta_v;

    residual = Delta_vt.norm();

    // Limit the angle change between vₜᵏ⁺¹ and vₜᵏ for all contact points.
    // The angle change θ is defined by the dot product between vₜᵏ⁺¹ and vₜᵏ
    // as: cos(θ) = vₜᵏ⁺¹⋅vₜᵏ/(‖vₜᵏ⁺¹‖‖vₜᵏ‖).
    // We'll do so by computing a coefficient 0 < α < 1 so that if the
    // generalized velocities are updated as vᵏ⁺¹ = vᵏ + α Δvᵏ then θ < θₘₐₓ
    // for all contact points.
    T alpha = 1.0;
    for (int ic = 0; ic < nc; ++ic) {
      const int ik = 2 * ic;
      auto vt_ic = vt.template segment<2>(ik);
      const auto dvt_ic = Delta_vt.template segment<2>(ik);
      alpha = min(alpha, LimitDirectionChange(vt_ic, dvt_ic));
    }

    // Limit v update:
    v = v + alpha * Delta_v;
    vt = D * v;

    // Save iteration statistics.
    statistics_.Update(
        ExtractDoubleOrThrow(residual), ExtractDoubleOrThrow(alpha));
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
T ImplicitStribeckSolver<T>::ModifiedStribeckPrime(const T& x, const T& mu) {
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
    class ::drake::multibody::implicit_stribeck::ImplicitStribeckSolver)
