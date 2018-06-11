#pragma once

#include <memory>

#include <gtest/gtest.h>

#include "drake/math/autodiff_gradient.h"

namespace drake {
namespace multibody {
namespace implicit_stribeck {
namespace {

template <typename U>
VectorX<U> CalcNormalForces(
    const VectorX<U>& phi,
    const VectorX<double>& stiffness, const VectorX<double>& damping,
    double dt, const VectorX<U>& vn) {
  int nc = phi.size();

  // Compute normal force at t^{n+1}
  const VectorX<U> k_vn = stiffness * (VectorX<U>::Ones(nc) - damping.asDiagonal() * vn);
  const VectorX<U> k_vn_clamped = k_vn.template cwiseMax(VectorX<U>::Zero(nc));
  const VectorX<U> phi_clamped = phi.cwiseMax(VectorX<U>::Zero(nc));
  const VectorX<U> fn = k_vn_clamped.asDiagonal() * phi_clamped;
  return fn;
}

template <typename U>
VectorX<U> CalcFrictionForces(
    double v_stribeck, double epsilon_v,
    const VectorX<double> mu,
    const VectorX<U>& vt, const VectorX<U>& fn) {
  const int nc = mu.size();  // Number of contact points.

  // We use the stiction tolerance as a reference scale to estimate a small
  // velocity v_epsilon. With v_epsilon we define a "soft norm" which we
  // use to compute "soft" tangent vectors to avoid a division by zero
  // singularity when tangential velocities are zero.
  const double epsilon_v2 = epsilon_v * epsilon_v;

  VectorX<U> ft(2 * nc);
  VectorX<U> mus(nc);
  VectorX<U> that(2 * nc);
  VectorX<U> v_slip(nc);

  auto ModifiedStribeck = [](U x, double mu) {
    if (x >= 1) {
      return U(mu);
    } else {
      return mu * x * (2.0 - x);
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
  // We use these softened quantities all throuout our derivations for
  // consistency.
  for (int ic = 0; ic < nc; ++ic) {
    const int ik = 2 * ic;
    const auto vt_ic = vt.template segment<2>(ik);
    // "soft norm":
    v_slip(ic) = sqrt(vt_ic.squaredNorm() + epsilon_v2);
    // "soft" tangent vector:
    const Vector2<U> that_ic = vt_ic / v_slip(ic);
    that.template segment<2>(ik) = that_ic;
    mus(ic) = ModifiedStribeck(v_slip(ic) / v_stribeck, mu(ic));
    // Friction force.
    ft.template segment<2>(ik) = -mus(ic) * that_ic * fn(ic);
  }
  return ft;
}


template <typename U>
VectorX<U> CalcResidualOnU(
    const MatrixX<double>& M,
    const MatrixX<double>& Jn,
    const MatrixX<double>& Jt,
    const VectorX<double>& p_star,
    const VectorX<double>& phi0,
    const VectorX<double>& mu,
    const VectorX<double>& fn_data,
    const VectorX<double>& stiffness,
    const VectorX<double>& damping,
    double dt, double v_stribeck, double epsilon_v,
    bool two_way_coupling,
    const VectorX<U>& v) {
  // Separation velocities vₙⁿ⁺¹.
  VectorX<U> vn = Jn * v;

  VectorX<U> fn;
  if (two_way_coupling) {
    // Compute separation distance at O(dt).
    // φⁿ⁺¹ = φⁿ - δt⋅vₙⁿ⁺¹. The minus sign is needed because vn's are
    // **separation** velocities, i.e. when negative, phi (penetration distance)
    // increases. That is, φ̇ = -vₙ.
    VectorX<U> phi = phi0 - dt * vn;

    // Normal force, as a function of φⁿ⁺¹ and vₙⁿ⁺¹.
    fn = CalcNormalForces(phi, stiffness, damping, dt, vn);
  } else {
    fn = fn_data;  // Fixed to input.
  }

  // Tangential velocity.
  VectorX<U> vt = Jt * v;

  // Friction forces.
  VectorX<U> ft = CalcFrictionForces(v_stribeck, epsilon_v, mu, vt, fn);

  // Newton-Raphson residual
  VectorX<U> residual =
      M * v - p_star - dt * Jt.transpose() * ft - dt * Jn.transpose() * fn;

  return residual;
}

MatrixX<double> CalcJacobianWithAutoDiff(
    const MatrixX<double>& M,
    const MatrixX<double>& Jn,
    const MatrixX<double>& Jt,
    const VectorX<double>& p_star,
    const VectorX<double>& phi0,
    const VectorX<double>& mu,
    const VectorX<double>& fn,
    const VectorX<double>& stiffness,
    const VectorX<double>& damping,
    double dt, double v_stribeck, double epsilon_v,
    bool two_way_coupling,
    const VectorX<double>& v) {
  VectorX<AutoDiffXd> v_autodiff(v.size());
  math::initializeAutoDiff(v, v_autodiff);
  VectorX<AutoDiffXd> residual = CalcResidualOnU(
      M, Jn, Jt, p_star, phi0, mu, fn, stiffness, damping,
      dt, v_stribeck, epsilon_v, two_way_coupling,
      v_autodiff);
  return math::autoDiffToGradientMatrix(residual);
}

}  // namespace
}  // namespace implicit_stribeck
}  // namespace multibody
}  // namespace drake

