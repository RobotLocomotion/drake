#pragma once

// This file implements a method to compute the  Newton-Raphson Jacobian
// J = ∇ᵥR of the residual for the TamsiSolver using automatic
// differentiation. This separate implementation is used to verify the
// analytical (and faster) Jacobian computed by the internal implementation of
// the solver.

#include <memory>

#include "drake/math/autodiff_gradient.h"

namespace drake {
namespace multibody {
namespace test {

// Computes the normal force as a function of the signed penetration depth x
// and separation velocity vn using a compliant law of the form:
//   fₙ(x, vₙ) = k(vₙ)₊ x₊
//       k(vₙ) = k (1 − d vₙ)₊
// where `x₊ = max(x, 0)` and k and d are the stiffness and
// dissipation coefficients for a given contact point, respectively.
template <typename U>
VectorX<U> CalcNormalForces(
    const VectorX<U>& x,
    const VectorX<double>& stiffness, const VectorX<double>& dissipation,
    double dt, const VectorX<U>& vn) {
  int nc = x.size();

  // Compute normal force at t^{n+1}
  const VectorX<U> k_vn =
      stiffness.array() *
      (VectorX<U>::Ones(nc).array() - dissipation.array() * vn.array());
  const VectorX<U> k_vn_clamped = k_vn.template cwiseMax(VectorX<U>::Zero(nc));
  const VectorX<U> x_clamped = x.cwiseMax(VectorX<U>::Zero(nc));
  const VectorX<U> fn = k_vn_clamped.asDiagonal() * x_clamped;
  return fn;
}

// Returns the friction forces ft as a function of the tangential velocity vt,
// for given friction coefficient mu and normal force fn.
template <typename U>
VectorX<U> CalcFrictionForces(
    double v_stiction, double epsilon_v,
    const VectorX<double> mu,
    const VectorX<U>& vt, const VectorX<U>& fn) {
  using std::sqrt;

  const int nc = mu.size();  // Number of contact points.

  // We use the stiction tolerance as a reference scale to estimate a small
  // velocity v_epsilon. With v_epsilon we define a "soft norm" which we
  // use to compute "soft" tangent vectors to avoid a division by zero
  // singularity when tangential velocities are zero.
  const double epsilon_v2 = epsilon_v * epsilon_v;

  VectorX<U> ft(2 * nc);
  VectorX<U> mu_vt(nc);  // = μ(‖vₜ‖)
  VectorX<U> t_hat(2 * nc);
  VectorX<U> v_slip(nc);

  auto RegularizedFriction = [](U x, double friction_coefficient) {
    if (x >= 1) {
      return U(friction_coefficient);
    } else {
      return friction_coefficient * x * (2.0 - x);
    }
  };

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
  // We use these softened quantities all throughout our derivations for
  // consistency.
  for (int ic = 0; ic < nc; ++ic) {  // Index ic scans contact points.
    const int ik = 2 * ic;  // Index ik scans contact vector quantities.
    const auto vt_ic = vt.template segment<2>(ik);
    // "soft norm":
    v_slip(ic) = sqrt(vt_ic.squaredNorm() + epsilon_v2);
    // "soft" tangent vector:
    const Vector2<U> that_ic = vt_ic / v_slip(ic);
    t_hat.template segment<2>(ik) = that_ic;
    mu_vt(ic) = RegularizedFriction(v_slip(ic) / v_stiction, mu(ic));
    // Friction force.
    ft.template segment<2>(ik) = -mu_vt(ic) * that_ic * fn(ic);
  }
  return ft;
}

// Computes and returns the Newton-Raphson residual for the TAMSI solver. This
// templated method is used to automatically differentiate the residual and
// compute its Jacobian J = ∇ᵥR.
// This same method is used to evaluate the residual for both the one-way (
// normal forces are fixed) and two-way coupled schemes. two_way_coupling = true
// indicates to compute the residual for the two-way coupled scheme. Call with
// two_way_coupling = false to compute the residual for the one-way coupled
// scheme.
// When two_way_coupling = true fn_data is not used.
// When two_way_coupling = false x0, stiffness and dissipation are not used.
template <typename U>
VectorX<U> CalcResidual(
    const MatrixX<double>& M,
    const MatrixX<double>& Jn,
    const MatrixX<double>& Jt,
    const VectorX<double>& p_star,
    const VectorX<double>& x0,
    const VectorX<double>& mu,
    const VectorX<double>& fn_data,
    const VectorX<double>& stiffness,
    const VectorX<double>& dissipation,
    double dt, double v_stiction, double epsilon_v,
    bool two_way_coupling,
    const VectorX<U>& v) {
  // Separation velocities vₙˢ⁺¹ ( = vn in code below).
  VectorX<U> vn = Jn * v;

  VectorX<U> fn;
  if (two_way_coupling) {
    // Compute penetration distance at O(dt).
    // xˢ⁺¹ = xˢ − δt vₙˢ⁺¹. The minus sign is needed because vn's are
    // **separation** velocities, i.e. when negative, x (penetration distance)
    // increases. That is, ẋ = -vₙ.
    VectorX<U> x = x0 - dt * vn;

    // Normal force, as a function of xˢ⁺¹ and vₙˢ⁺¹.
    fn = CalcNormalForces(x, stiffness, dissipation, dt, vn);
  } else {
    fn = fn_data;  // Fixed to input.
  }

  // Tangential velocity.
  VectorX<U> vt = Jt * v;

  // Friction forces.
  VectorX<U> ft = CalcFrictionForces(v_stiction, epsilon_v, mu, vt, fn);

  // Newton-Raphson residual
  VectorX<U> residual =
      M * v - p_star - dt * Jt.transpose() * ft - dt * Jn.transpose() * fn;

  return residual;
}

// Computes the Jacobian J = ∇ᵥR of the residual for the two-way coupled scheme
// of TamsiSolver using automatic differentiation.
MatrixX<double> CalcTwoWayCoupledJacobianWithAutoDiff(
    const MatrixX<double>& M,
    const MatrixX<double>& Jn,
    const MatrixX<double>& Jt,
    const VectorX<double>& p_star,
    const VectorX<double>& x0,
    const VectorX<double>& mu,
    const VectorX<double>& stiffness,
    const VectorX<double>& dissipation,
    double dt, double v_stiction, double epsilon_v,
    const VectorX<double>& v) {
  VectorX<AutoDiffXd> v_autodiff(v.size());
  math::InitializeAutoDiff(v, &v_autodiff);
  // Empty vector for data not used by the two-way coupled scheme.
  const VectorX<double> not_used;
  VectorX<AutoDiffXd> residual = CalcResidual(
      M, Jn, Jt, p_star, x0, mu, not_used, stiffness, dissipation,
      dt, v_stiction, epsilon_v, true,
      v_autodiff);
  return math::ExtractGradient(residual);
}

// Computes the Jacobian J = ∇ᵥR of the residual for the one-way coupled scheme
// of TamsiSolver using automatic differentiation.
MatrixX<double> CalcOneWayCoupledJacobianWithAutoDiff(
    const MatrixX<double>& M,
    const MatrixX<double>& Jn,
    const MatrixX<double>& Jt,
    const VectorX<double>& p_star,
    const VectorX<double>& mu,
    const VectorX<double>& fn,
    double dt, double v_stiction, double epsilon_v,
    const VectorX<double>& v) {
  VectorX<AutoDiffXd> v_autodiff(v.size());
  math::InitializeAutoDiff(v, &v_autodiff);
  // Empty vector for data not used by the one-way coupled scheme.
  const VectorX<double> not_used;
  VectorX<AutoDiffXd> residual = CalcResidual(
      M, Jn, Jt, p_star, not_used, mu, fn, not_used, not_used,
      dt, v_stiction, epsilon_v, false,
      v_autodiff);
  return math::ExtractGradient(residual);
}

}  // namespace test
}  // namespace multibody
}  // namespace drake

