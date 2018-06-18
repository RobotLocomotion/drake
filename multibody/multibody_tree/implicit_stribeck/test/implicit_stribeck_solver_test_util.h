#pragma once

// This file implements a method to compute the  Newton-Raphson Jacobian
// J = ∇ᵥR of the residual for the ImplicitStribeckSolver using automatic
// differentiation. This separate implementation is used to verify the
// analytical (and faster) Jacobian computed by the internal implementation of
// the solver.

#include <memory>

#include <gtest/gtest.h>

#include "drake/math/autodiff_gradient.h"

namespace drake {
namespace multibody {
namespace implicit_stribeck {
namespace test {

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

  auto ModifiedStribeck = [](U x, double friction_coefficient) {
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
    mu_vt(ic) = ModifiedStribeck(v_slip(ic) / v_stiction, mu(ic));
    // Friction force.
    ft.template segment<2>(ik) = -mu_vt(ic) * that_ic * fn(ic);
  }
  return ft;
}

// Computes and returns the Newton-Raphson residual for the implicit Stribeck
// solver. This templated method is used to automatically differentiate the
// residual and compute its Jacobian J = ∇ᵥR.
template <typename U>
VectorX<U> CalcResidual(
    const MatrixX<double>& M,
    const MatrixX<double>& Jt,
    const VectorX<double>& p_star,
    const VectorX<double>& mu,
    const VectorX<double>& fn_data,
    double dt, double v_stiction, double epsilon_v,
    const VectorX<U>& v) {
  // Tangential velocity.
  VectorX<U> vt = Jt * v;

  // Normal forces.
  // TODO(amcastro-tri): Modify this (now constant) term to implicitly include
  // the dependence on generalized velocities so that we can test the two-way
  // coupling scheme (which also computes normal forces implicitly).
  const VectorX<U> fn = fn_data;

  // Friction forces.
  VectorX<U> ft = CalcFrictionForces(v_stiction, epsilon_v, mu, vt, fn);

  // Newton-Raphson residual
  VectorX<U> residual = M * v - p_star - dt * Jt.transpose() * ft;

  return residual;
}

// Computes the Jacobian J = ∇ᵥR of the residual for the ImplicitStribeckSolver
// using automatic differentiation.
MatrixX<double> CalcJacobianWithAutoDiff(
    const MatrixX<double>& M,
    const MatrixX<double>& Jt,
    const VectorX<double>& p_star,
    const VectorX<double>& mu,
    const VectorX<double>& fn,
    double dt, double v_stiction, double epsilon_v,
    const VectorX<double>& v) {
  VectorX<AutoDiffXd> v_autodiff(v.size());
  math::initializeAutoDiff(v, v_autodiff);
  VectorX<AutoDiffXd> residual = CalcResidual(
      M, Jt, p_star, mu, fn, dt, v_stiction, epsilon_v, v_autodiff);
  return math::autoDiffToGradientMatrix(residual);
}

}  // namespace test
}  // namespace implicit_stribeck
}  // namespace multibody
}  // namespace drake

