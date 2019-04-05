#pragma once

/// @file
/// Template method implementations for radau3_integrator.h.
/// Most users should only include that file, not this one.
/// For background, see https://drake.mit.edu/cxx_inl.html.

/* clang-format off to disable clang-format-includes */
#include "drake/systems/analysis/implicit_integrator.h"
/* clang-format on */

#include <algorithm>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

#include "drake/common/text_logging.h"
#include "drake/math/autodiff.h"

namespace drake {
namespace systems {

template <class T>
void ImplicitIntegrator<T>::DoResetStatistics() {
  num_jacobian_function_evaluations_ = 0;
  num_jacobian_evaluations_ = 0;
}

// Computes the Jacobian of the ordinary differential equations taken with
// respect to the continuous state (at a point specified by @p state) using
// automatic differentiation.
template <>
MatrixX<AutoDiffXd> ImplicitIntegrator<AutoDiffXd>::
    ComputeAutoDiffJacobian(const System<AutoDiffXd>&,
      const AutoDiffXd&, const VectorX<AutoDiffXd>&,
      const Context<AutoDiffXd>&) {
        throw std::runtime_error("AutoDiff'd Jacobian not supported from "
                                     "AutoDiff'd ImplicitIntegrator");
}

// Computes the Jacobian of the ordinary differential equations taken with
// respect to the continuous state (at a point specified by @p state) using
// automatic differentiation.
// @param system The dynamical system.
// @param context The context at which to compute the time derivatives.
// @param state The continuous state at which to compute the time derivatives.
//              The function can modify this continuous state during the
//              Jacobian computation.
// @post The continuous state will be indeterminate on return.
template <class T>
MatrixX<T> ImplicitIntegrator<T>::ComputeAutoDiffJacobian(
    const System<T>& system, const T& t, const VectorX<T>& xc,
    const Context<T>& context) {
  SPDLOG_DEBUG(drake::log(), "  ImplicitIntegrator Compute Autodiff Jacobian "
               "t={}",t);
  // Create AutoDiff versions of the state vector.
  VectorX<AutoDiffXd> a_xc = xc;

  // Set the size of the derivatives and prepare for Jacobian calculation.
  const int n_state_dim = a_xc.size();
  for (int i = 0; i < n_state_dim; ++i)
    a_xc[i].derivatives() = VectorX<T>::Unit(n_state_dim, i);

  // Get the system and the context in AutoDiffable format. Inputs must also
  // be copied to the context used by the AutoDiff'd system (which is
  // accomplished using FixInputPortsFrom()).
  // TODO(edrumwri): Investigate means for moving as many of the operations
  //                 below offline (or with lower frequency than once-per-
  //                 Jacobian calculation) as is possible. These operations
  //                 are likely to be expensive.
  const auto adiff_system = system.ToAutoDiffXd();
  std::unique_ptr<Context<AutoDiffXd>> adiff_context = adiff_system->
      AllocateContext();
  adiff_context->SetTimeStateAndParametersFrom(context);
  adiff_context->SetTime(t);
  adiff_system->FixInputPortsFrom(system, context, adiff_context.get());

  // Set the continuous state in the context.
  adiff_context->get_mutable_continuous_state().get_mutable_vector().
      SetFromVector(a_xc);

  // Evaluate the derivatives at that state.
  const VectorX<AutoDiffXd> result =
      this->EvalTimeDerivatives(*adiff_system, *adiff_context).CopyToVector();

  return math::autoDiffToGradientMatrix(result);
}

// Evaluates the ordinary differential equations at the time and state in
// the system's context (stored by the integrator).
template <class T>
VectorX<T> ImplicitIntegrator<T>::EvalTimeDerivativesUsingContext() {
    return this->EvalTimeDerivatives(this->get_context()).CopyToVector();
}

// Computes the Jacobian of the ordinary differential equations taken with
// respect to the continuous state (at a point specified by @p state) using
// a first-order forward difference (i.e., numerical differentiation).
// @param system The dynamical system.
// @param context Mutable context for in-place computation of time derivatives.
// @param state The continuous state at which to compute the time derivatives.
//              The function can modify this continuous state during the
//              Jacobian computation.
// @post The continuous state will be indeterminate on return.
template <class T>
MatrixX<T> ImplicitIntegrator<T>::ComputeForwardDiffJacobian(
    const System<T>&, const T& t, const VectorX<T>& xc, Context<T>* context) {
  using std::abs;

  // Set epsilon to the square root of machine precision.
  const double eps = std::sqrt(std::numeric_limits<double>::epsilon());

  // Get the number of continuous state variables xc.
  const int n = context->num_continuous_states();

  SPDLOG_DEBUG(drake::log(), "  ImplicitIntegrator Compute Forwarddiff "
               "{}-Jacobian t={}", n, t);
  SPDLOG_DEBUG(drake::log(), "  computing from state {}", xc.transpose());

  // Initialize the Jacobian.
  MatrixX<T> J(n, n);

  // Evaluate f(t,xc).
  context->SetTimeAndContinuousState(t, xc);
  const VectorX<T> f = EvalTimeDerivativesUsingContext();

  // Compute the Jacobian.
  VectorX<T> xc_prime = xc;
  for (int i = 0; i < n; ++i) {
    // Compute a good increment to the dimension using approximately 1/eps
    // digits of precision. Note that if |xc| is large, the increment will
    // be large as well. If |xc| is small, the increment will be no smaller
    // than eps.
    const T abs_xi = abs(xc(i));
    T dxi(abs_xi);
    if (dxi <= 1) {
      // When |xc[i]| is small, increment will be eps.
      dxi = eps;
    } else {
      // |xc[i]| not small; make increment a fraction of |xc[i]|.
      dxi = eps * abs_xi;
    }

    // Update xc', minimizing the effect of roundoff error by ensuring that
    // x and dx differ by an exactly representable number. See p. 192 of
    // Press, W., Teukolsky, S., Vetterling, W., and Flannery, P. Numerical
    //   Recipes in C++, 2nd Ed., Cambridge University Press, 2002.
    xc_prime(i) = xc(i) + dxi;
    dxi = xc_prime(i) - xc(i);

    // TODO(sherm1) This is invalidating q, v, and z but we only changed one.
    //              Switch to a method that invalides just the relevant
    //              partition, and ideally modify only the one changed element.
    // Compute f' and set the relevant column of the Jacobian matrix.
    context->SetTimeAndContinuousState(t, xc_prime);
    J.col(i) = (EvalTimeDerivativesUsingContext() - f) / dxi;

    // Reset xc' to xc.
    xc_prime(i) = xc(i);
  }

  return J;
}

// Computes the Jacobian of the ordinary differential equations taken with
// respect to the continuous state (at a point specified by @p state) using
// a second-order central difference (i.e., numerical differentiation).
// @param system The dynamical system.
// @param context Mutable context for in-place computation of time derivatives.
// @post The continuous state will be indeterminate on return.
template <class T>
MatrixX<T> ImplicitIntegrator<T>::ComputeCentralDiffJacobian(
    const System<T>&, const T& t, const VectorX<T>& xc, Context<T>* context) {
  using std::abs;

  // Cube root of machine precision (indicated by theory) seems a bit coarse.
  // Pick power of eps halfway between 6/12 (i.e., 1/2) and 4/12 (i.e., 1/3).
  const double eps = std::pow(std::numeric_limits<double>::epsilon(), 5.0/12);

  // Get the number of continuous state variables xc.
  const int n = context->num_continuous_states();

  SPDLOG_DEBUG(drake::log(), "  ImplicitIntegrator Compute ",
               "Centraldiff {}-Jacobian t={}", n, t);


  // Initialize the Jacobian.
  MatrixX<T> J(n, n);

  // Evaluate f(t,xc).
  context->SetTimeAndContinuousState(t, xc);
  const VectorX<T> f = EvalTimeDerivativesUsingContext();

  // Compute the Jacobian.
  VectorX<T> xc_prime = xc;
  for (int i = 0; i < n; ++i) {
    // Compute a good increment to the dimension using approximately 1/eps
    // digits of precision. Note that if |xc| is large, the increment will
    // be large as well. If |xc| is small, the increment will be no smaller
    // than eps.
    const T abs_xi = abs(xc(i));
    T dxi(abs_xi);
    if (dxi <= 1) {
      // When |xc[i]| is small, increment will be eps.
      dxi = eps;
    } else {
      // |xc[i]| not small; make increment a fraction of |xc[i]|.
      dxi = eps * abs_xi;
    }

    // Update xc', minimizing the effect of roundoff error, by ensuring that
    // x and dx differ by an exactly representable number. See p. 192 of
    // Press, W., Teukolsky, S., Vetterling, W., and Flannery, P. Numerical
    //   Recipes in C++, 2nd Ed., Cambridge University Press, 2002.
    xc_prime(i) = xc(i) + dxi;
    const T dxi_plus = xc_prime(i) - xc(i);

    // TODO(sherm1) This is invalidating q, v, and z but we only changed one.
    //              Switch to a method that invalides just the relevant
    //              partition, and ideally modify only the one changed element.
    // Compute f(x+dx).
    context->SetContinuousState(xc_prime);
    VectorX<T> fprime_plus = EvalTimeDerivativesUsingContext();

    // Update xc' again, minimizing the effect of roundoff error.
    xc_prime(i) = xc(i) - dxi;
    const T dxi_minus = xc(i) - xc_prime(i);

    // Compute f(x-dx).
    context->SetContinuousState(xc_prime);
    VectorX<T> fprime_minus = EvalTimeDerivativesUsingContext();

    // Set the Jacobian column.
    J.col(i) = (fprime_plus - fprime_minus) / (dxi_plus + dxi_minus);

    // Reset xc' to xc.
    xc_prime(i) = xc(i);
  }

  return J;
}

// Factors a dense matrix (the negated iteration matrix) using LU factorization,
// which should be faster than the QR factorization used in the specialized
// template method immediately below.
template <class T>
void ImplicitIntegrator<T>::IterationMatrix::SetAndFactorIterationMatrix(
    const MatrixX<T>& iteration_matrix) {
  LU_.compute(iteration_matrix);
  matrix_factored_ = true;
}

// Factors a dense matrix (the negated iteration matrix). This
// AutoDiff-specialized method is necessary because Eigen's LU factorization,
// which should be faster than the QR factorization used here, is not currently
// AutoDiff-able (while the QR factorization *is* AutoDiff-able).
// TODO(edrumwri): Record the factorization.
template <>
void ImplicitIntegrator<AutoDiffXd>::IterationMatrix::
    SetAndFactorIterationMatrix(const MatrixX<AutoDiffXd>& iteration_matrix) {
  QR_.compute(iteration_matrix);
  matrix_factored_ = true;
}

// Solves a linear system Ax = b for x using a negated iteration matrix (A)
// factored using LU decomposition.
// @sa Factor()
template <class T>
VectorX<T> ImplicitIntegrator<T>::IterationMatrix::Solve(
    const VectorX<T>& b) const {
  return LU_.solve(b);
}

// Solves the linear system Ax = b for x using a negated iteration matrix (A)
// factored using QR decomposition.
// @sa Factor()
template <>
VectorX<AutoDiffXd> ImplicitIntegrator<AutoDiffXd>::IterationMatrix::Solve(
    const VectorX<AutoDiffXd>& b) const {
  return QR_.solve(b);
}

// Checks to see whether a Jacobian matrix has "become bad" and needs to be
// refactorized.
template <class T>
bool ImplicitIntegrator<T>::IsBadJacobian(const MatrixX<T>& J) const {
  return !J.allFinite();
}

// Compute the partial derivative of the ordinary differential equations with
// respect to the state variables for a given x(t).
// @post the context's time and continuous state will be temporarily set during
//       this call (and then reset to their original values) on return.
template <class T>
MatrixX<T> ImplicitIntegrator<T>::CalcJacobian(const T& t,
                                               const VectorX<T>& x) {
  // We change the context but will change it back.
  Context<T>* context = this->get_mutable_context();

  // Get the current time and state.
  const T t_current = context->get_time();
  const VectorX<T> x_current = context->get_continuous_state_vector().
      CopyToVector();

  // Update the time and state.
  context->SetTimeAndContinuousState(t, x);
  num_jacobian_evaluations_++;

  // Get the current number of ODE evaluations.
  int64_t current_ODE_evals = this->get_num_derivative_evaluations();

  // Get a the system.
  const System<T>& system = this->get_system();

  // TODO(edrumwri): Give the caller the option to provide their own Jacobian.
  const MatrixX<T> J = [this, context, &system, &t, &x]() {
    switch (jacobian_scheme_) {
      case JacobianComputationScheme::kForwardDifference:
        return ComputeForwardDiffJacobian(system, t, x, &*context);

      case JacobianComputationScheme::kCentralDifference:
        return ComputeCentralDiffJacobian(system, t, x, &*context);

      case JacobianComputationScheme::kAutomatic:
        return ComputeAutoDiffJacobian(system, t, x, *context);
    }
    DRAKE_UNREACHABLE();
  }();

  // Use the new number of ODE evaluations to determine the number of Jacobian
  // evaluations.
  num_jacobian_function_evaluations_ += this->get_num_derivative_evaluations()
      - current_ODE_evals;

  // Reset the time and state.
  context->SetTimeAndContinuousState(t_current, x_current);

  return J;
}

}  // namespace systems
}  // namespace drake
