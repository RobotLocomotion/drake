#pragma once

// This file implements a method to compute the  Newton-Raphson Jacobian
// J = ∇ᵥR of the residual for the ImplicitStribecSolver using automatic
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

// Computes and returns the Newton-Raphson residual for the implicit Stribeck
// solver. This templated method is used to automatically differentiate the
// residual and compute its Jacobian J = ∇ᵥR.
template <typename U>
VectorX<U> CalcResidualOnU(
    const MatrixX<double>& M,
    const MatrixX<double>& Jt,
    const VectorX<double>& p_star,
    const VectorX<double>& mu,
    const VectorX<double>& fn_data,
    double dt, double v_stribeck, double epsilon_v,
    const VectorX<U>& v) {
  // Tangential velocity.
  VectorX<U> vt = Jt * v;

  // Normal forces.
  // TODO(amcastro-tri): Modify this (now constant) term to implicitly include
  // the dependence on generalized velocities so that we can test the two-way
  // coupling schem (which also computes normal forces implicitly).
  const VectorX<U>& fn = fn_data;

  // Friction forces.
  VectorX<U> ft = CalcFrictionForces(v_stribeck, epsilon_v, mu, vt, fn);

  // Newton-Raphson residual
  VectorX<U> residual = M * v - p_star - dt * Jt.transpose() * ft;

  return residual;
}

// Computes the Jacobian J = ∇ᵥR of the residual for the ImplicitStribecSolver
// using automatic differentiation.
MatrixX<double> CalcJacobianWithAutoDiff(
    const MatrixX<double>& M,
    const MatrixX<double>& Jt,
    const VectorX<double>& p_star,
    const VectorX<double>& mu,
    const VectorX<double>& fn,
    double dt, double v_stribeck, double epsilon_v,
    const VectorX<double>& v) {
  VectorX<AutoDiffXd> v_autodiff(v.size());
  math::initializeAutoDiff(v, v_autodiff);
  VectorX<AutoDiffXd> residual = CalcResidualOnU(
      M, Jt, p_star, mu, fn, dt, v_stribeck, epsilon_v, v_autodiff);
  return math::autoDiffToGradientMatrix(residual);
}

}  // namespace test
}  // namespace implicit_stribeck
}  // namespace multibody
}  // namespace drake

