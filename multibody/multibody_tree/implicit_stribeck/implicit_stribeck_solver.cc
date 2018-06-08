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

template <typename T>
ImplicitStribeckSolver<T>::ImplicitStribeckSolver(int nv) :
    nv_(nv),
    fixed_size_workspace_(nv),
    // Provide an initial workspace size so that we avoid re-allocations
    // afterwards as much as we can.
    variable_size_workspace_(128) {
  DRAKE_THROW_UNLESS(nv > 0);
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
    fixed_size_workspace_.tau_f.setZero();
    const auto& M = *problem_data_aliases_.M_ptr;
    const auto& p_star = *problem_data_aliases_.p_star_ptr;
    auto& v = fixed_size_workspace_.v;
    // With no friction forces Eq. (3) in the documentation reduces to
    // M⋅vⁿ⁺¹ = p*.
    v = M.ldlt().solve(p_star);
    // "One iteration" with exactly "zero" residual.
    statistics_.Update(0.0);
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
  auto that = variable_size_workspace_.mutable_t_hat();
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
  // singularity when tangential velocities are zero.
  const double epsilon_v = v_stribeck * 1.0e-4;
  const double epsilon_v2 = epsilon_v * epsilon_v;

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
      const T dmudv = ModifiedStribeckDerivative(
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
    if (J_ldlt.info() != Eigen::Success) {
      return ComputationInfo::LinearSolverFailed;
    }
    Delta_v = J_ldlt.solve(-R);

    // Since we keep D constant we have that:
    // vₜᵏ⁺¹ = D⋅vᵏ⁺¹ = D⋅(vᵏ + α Δvᵏ)
    //                = vₜᵏ + α D⋅Δvᵏ
    //                = vₜᵏ + α Δvₜᵏ
    // where we defined Δvₜᵏ = D⋅Δvᵏ and 0 < α < 1 is a constant that we'll
    // determine by limiting the maximum angle change between vₜᵏ and vₜᵏ⁺¹.
    // For multiple contact points, we choose the minimum α amongs all contact
    // points.
    Delta_vt = D * Delta_v;

    // Convergence is monitored in the tangential velocties.
    residual = Delta_vt.norm();

    // TODO(amcastro-tri): Limit the angle change between vₜᵏ⁺¹ and vₜᵏ for
    // all contact points. The angle change θ is defined by the dot product
    // between vₜᵏ⁺¹ and vₜᵏ as: cos(θ) = vₜᵏ⁺¹⋅vₜᵏ/(‖vₜᵏ⁺¹‖‖vₜᵏ‖).
    // We'll do so by computing a coefficient 0 < α < 1 so that if the
    // generalized velocities are updated as vᵏ⁺¹ = vᵏ + α Δvᵏ then θ < θₘₐₓ
    // for all contact points.
    T alpha = 1.0;  // We set α = 1 for now.
    v = v + alpha * Delta_v;
    vt = D * v;

    // Save iteration statistics.
    statistics_.Update(ExtractDoubleOrThrow(residual));
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
    class ::drake::multibody::implicit_stribeck::ImplicitStribeckSolver)
