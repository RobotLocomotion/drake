#include "drake/multibody/contact_solvers/implicit_stribeck_solver.h"

#include <memory>
#include <utility>
#include <vector>

#include<Eigen/IterativeLinearSolvers>

#include "drake/common/default_scalars.h"
#include "drake/common/extract_double.h"

#include <fstream>
#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;
//#define PRINT_VAR(a) (void)a;
//#define PRINT_VARn(a) (void)a;

namespace drake {
namespace multibody {

template <typename T>
ImplicitStribeckSolver<T>::ImplicitStribeckSolver(
    int nv, double stiction_tolerance) :
    nv_(nv), stiction_tolerance_(stiction_tolerance) {
  DRAKE_DEMAND(nv > 0);
  // Allocate once and for all workspace with size only dependent on nv.
  vk.resize(nv);
  Rk.resize(nv);
  Delta_vk.resize(nv);
  Jk.resize(nv, nv);
}

template <typename T>
void ImplicitStribeckSolver<T>::SetProblemData(
    const MatrixX<T>* M, const MatrixX<T>* D, const VectorX<T>* p_star,
    const VectorX<T>* fn, const VectorX<T>* mu) {
  nv_ = p_star->size();
  nc_ = fn->size();
  DRAKE_THROW_UNLESS(M->rows() == nv_ && M_->cols() == nv_);
  DRAKE_THROW_UNLESS(D->rows() == 2 * nc_ && D_->cols() == nv_);
  DRAKE_THROW_UNLESS(mu->size() == nc_);
  M_ = M;
  D_ = D;
  p_star_ = p_star;
  fn_ = fn;
  mu_ = mu;
  ResizeSolverWorkspaceAsNeeded(nc_);
}

template <typename T>
void ImplicitStribeckSolver<T>::ResizeSolverWorkspaceAsNeeded(int nc) {
  const int nf = 2 * nc;

  // Only reallocate if sizes from previous allocations are not sufficient.
  if (vtk.size() < nf) vtk.resize(nf);
  if (ftk.size() < nf) ftk.resize(nf);
  if (Delta_vtk.size() < nf) Delta_vtk.resize(nf);
  if (vtk.size() < nf) vtk.resize(nf);
  if (that.size() < nf) that.resize(nf);
  if (v_slip.size() < nc) v_slip.resize(nc);
  if (mus.size() < nc) mus.resize(nc);
  if (dmudv.size() < nc) dmudv.resize(nc);
  // There is no reallocation if std::vector::capacity() >= nc.
  dft_dv.resize(nc);
}

template <typename T>
VectorX<T> ImplicitStribeckSolver<T>::SolveWithGuess(
    double dt, const VectorX<T>& v_guess) {
  DRAKE_THROW_UNLESS(v_guess.size() == nv_);

  using std::abs;
  using std::max;
  using std::min;
  using std::sqrt;

  // If there are no contact points return a zero generalized friction forces
  // vector, i.e. tau_f = 0.
  if (nc_ == 0) return VectorX<T>::Zero(nv_);

  // Solver parameters.
  const int max_iterations = parameters_.max_iterations;
  const double tolerance = parameters_.v_tolerance;

  // Problem sizes.
  const int nv = nv_;  // Number of generalized velocities.
  const int nc = nc_;  // Number of contact points.
  // Size of the friction forces vector ft and tangential velocities vector vt.
  const int nf = 2 * nc;

  // Convenient aliases.
  const auto& M = *M_;
  const auto& D = *D_;
  const auto& p_star = *p_star_;
  const auto& fn = *fn_;
  const auto& mu = *mu_;

  // Initialize residual to a value larger than tolerance so that the solver at
  // least performs one iteration
  T residual = 2 * tolerance;

  // Initialize iteration with the provided guess.
  vk = v_guess;
  vtk = D * vk;

  // The stiction tolerance.
  const double v_stribeck = stiction_tolerance_;

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
      const auto vt_ic = vtk.template segment<2>(ik);
      // "soft norm":
      v_slip(ic) = sqrt(vt_ic.squaredNorm() + epsilon_v2);
      // "soft" tangent vector:
      const Vector2<T> that_ic =  vt_ic / v_slip(ic);
      that.template segment<2>(ik) = that_ic;
      mus(ic) = ModifiedStribeck(v_slip(ic) / v_stribeck, mu(ic));
      // Friction force.
      // Note: minus sign not included in this definition.
      ftk.template segment<2>(ik) = mus(ic) * that_ic * fn(ic);
    }

    // After the previous iteration, we allow updating ftk above to have its
    // latest value before leaving.
    if (residual < tolerance) {
      break;
    }

    // Newton-Raphson residual
    Rk = M * vk - p_star + dt * D.transpose() * ftk;

    // Compute dft/dvt, a 2x2 matrix with the derivative of the friction
    // force (in ℝ²) with respect to the tangent velocity (also in ℝ²).
    for (int ic = 0; ic < nc; ++ic) {
      const int ik = 2 * ic;

      // Compute dmu/dv = (1/v_stribeck) * dmu/dx
      // where x = v_slip / v_stribeck is the dimensionless slip velocity.
      dmudv(ic) = ModifiedStribeckPrime(
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
      dft_dv[ic] += P_ic * dmudv(ic);

      // Note: dft_dv is a symmetric 2x2 matrix.
      dft_dv[ic] *= fn(ic);

      //PRINT_VARn(dft_dv[ic]);
    }

    // Newton-Raphson Jacobian:
    //  J = I + dt M⁻¹Dᵀdiag(dfₜ/dvₜ)D
    // J is an (nv x nv) symmetric positive definite matrix.
    // diag(dfₜ/dvₜ) is the (2nc x 2nc) block diagonal matrix with dfₜ/dvₜ in
    // each 2x2 diagonal entry.

    // Start by multiplying diag(dfₜ/dvₜ)D and use the fact that diag(dfₜ/dvₜ)
    // is block diagonal.
    MatrixX<T> diag_dftdv_times_D(nf, nv);
    // TODO(amcastro-tri): Only build half of the matrix since it is
    // symmetric.
    for (int ic = 0; ic < nc; ++ic) {
      const int ik = 2 * ic;
      diag_dftdv_times_D.block(ik, 0, 2, nv) =
          dft_dv[ic] * D.block(ik, 0, 2, nv);
    }
    // Form J = M + dt Dᵀdiag(dfₜ/dvₜ)D:
    Jk = M + dt * D.transpose() * diag_dftdv_times_D;

    //PRINT_VARn(Jk);
    //PRINT_VARn(Rk.transpose());
    //PRINT_VARn(Minv_times_Dtrans);
    //PRINT_VARn((Minv_times_Dtrans * D).eval());
    //PRINT_VARn((D.transpose() * D).eval());

    // TODO(amcastro-tri): Consider using a cheap iterative solver like CG.
    // Since we are in a non-linear iteration, an approximate cheap solution
    // is probably best.
    // TODO(amcastro-tri): Consider using a matrix-free iterative method to
    // avoid computing M and J. CG and the Krylov family can be matrix-free.
    //Delta_vk = Jk.llt().solve(-Rk);
    // TODO(amcastro-tri): Make cg a solver's member and figure out how to avoid
    // allocation.
    Eigen::ConjugateGradient<MatrixX<T>, Eigen::Lower|Eigen::Upper> cg;
    cg.compute(Jk);
    cg.setTolerance(100 * tolerance);
    cg.setMaxIterations(nv);
    Delta_vk = cg.solve(-Rk);

    if (cg.info() != Eigen::Success) {
      throw std::logic_error("Iterative linear solver did not converge to the "
                                 "specified tolerance.");
    }

    // cg.iterations()
    // cg.error()

    // Since we keep D constant we have that:
    // vₜᵏ⁺¹ = D⋅vᵏ⁺¹ = D⋅(vᵏ + α Δvᵏ)
    //                = vₜᵏ + α D⋅Δvᵏ
    //                = vₜᵏ + α Δvₜᵏ
    // where we defined Δvₜᵏ = D⋅Δvᵏ and 0 < α < 1 is a constant that we'll
    // determine by limiting the maximum angle change between vₜᵏ and vₜᵏ⁺¹.
    // For multiple contact points, we choose the minimum α amongs all contact
    // points.
    Delta_vtk = D * Delta_vk;

    residual = Delta_vk.norm();

    // Limit the angle change
    const  T theta_max = 0.25;  // about 15 degs
    const T cmin = cos(theta_max);
    T alpha_min = 1.0;
    for (int ic = 0; ic < nc; ++ic) {
      const int ik = 2 * ic;
      auto v = vtk.template segment<2>(ik);
      const auto dv = Delta_vtk.template segment<2>(ik);

      T A = v.norm();
      T B = v.dot(dv);
      T DD = dv.norm();

      //if (A < 1e-14 && D < 1e-14) continue;

      Vector2<T> v1 = v+dv;  // for alpha = 1
      const T v1_norm = v1.norm();
      const T v_norm = v.norm();
      const T cos_init = v1.dot(v) / (v1_norm+1e-10) / (v_norm+1e-10);

      Vector2<T> valpha;
      T alpha;

      const T x = v_norm / v_stribeck;
      const T x1 = v1_norm / v_stribeck;

      // 180 degrees direction change.
      if ( abs(1.0+cos_init) < 1.0e-10 ) {
        // Clip to near the origin since we know for sure we crossed it.
        valpha = v / (v.norm()+1e-14) * v_stribeck / 2.0;
        alpha = dv.dot(valpha - v) / dv.squaredNorm();
      } else if (cos_init > cmin || (v1_norm*v_norm) < 1.0e-14 || x < 1.0 || x1 < 1.0) {  // the angle change is small enough
        alpha = 1.0;
        valpha = v1;
      } else { // Limit the angle change
        T A2 = A * A;
        T A4 = A2 * A2;
        T cmin2 = cmin * cmin;

        T a = A2 * DD * DD * cmin2 - B * B;
        T b = 2 * A2 * B * (cmin2 - 1.0);
        T c = A4 * (cmin2 - 1.0);

        T delta = b * b - 4 * a * c;

        T sqrt_delta = sqrt(max(delta, 0.0));

        // There should be a positive and a negative root.
        alpha = (-b + sqrt_delta) / a / 2.0;
        //double alpha2 = (-b - sqrt_delta)/a/2.0;
        if (alpha <= 0) {
          PRINT_VAR(alpha);
          PRINT_VAR(iter);
          PRINT_VAR(delta);
          PRINT_VAR(A);
          PRINT_VAR(B);
          PRINT_VAR(cmin);
          PRINT_VAR(DD);
          PRINT_VAR(cos_init);
          PRINT_VAR(abs(1.0+cos_init));
          PRINT_VAR(a);
          PRINT_VAR(b);
          PRINT_VAR(c);
          PRINT_VAR(v.transpose());
          PRINT_VAR(v1.transpose());
          PRINT_VAR(dv.transpose());
        }

        DRAKE_DEMAND(alpha > 0);

        valpha = v + alpha * dv;
      }

      // clip v
      v = valpha;
      alpha_min = min(alpha_min, alpha);
    }

    // Limit vk update:
    vk = vk + alpha_min * Delta_vk;
    vtk = D * vk;

    // Save iteration statistics.
    statistics_.iteration_residuals.push_back(ExtractDoubleOrThrow(residual));
    statistics_.iteration_alpha.push_back(ExtractDoubleOrThrow(alpha_min));
    statistics_.linear_iterations.push_back(cg.iterations());
    statistics_.linear_residuals.push_back(ExtractDoubleOrThrow(cg.error()));
  }

  // Returns vector of generalized friction forces.
  return D.transpose() * ftk;
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


}  // namespace multibody
}  // namespace drake

// Explicitly instantiates on the most common scalar types.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::ImplicitStribeckSolver)
