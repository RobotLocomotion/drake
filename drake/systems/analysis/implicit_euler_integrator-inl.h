#pragma once

/// @file
/// Template method implementations for runge_kutta_3_integrator.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include <limits>
#include <utility>

#include "drake/systems/analysis/line_search.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"

namespace drake {
namespace systems {

template <class T>
void ImplicitEulerIntegrator<T>::DoResetStatistics() {
  num_objective_function_increases_ = 0;
  num_misdirected_descents_ = 0;
  num_overly_small_updates_ = 0;
}

// Computes the Jacobian of the residual "error" with respect to the state
// variables using automatic differentiation.
template <class T>
MatrixX<T> ImplicitEulerIntegrator<T>::ComputeADiffJacobian(
    const VectorX<T>& xt, const VectorX<T>& xtplus, double h) {

/*
// Get the system and the context
using Scalar = typename std::remove_reference<decltype(xt)>::type::Scalar;
const auto& system = this->get_system();
Context<T>* context = this->get_mutable_context();

  // TODO(edrumwri): Fix this.

  // Must make copies (?).
  VectorX<T> xt_copy = xt;
  VectorX<T> xtplus_copy = xtplus; 
*/
  // Set g
/*
  MatrixX<T> M(5,3);
  VectorX<T> q(3);
  auto g = [&](const auto& x) {
    using Scalar = typename std::remove_reference<decltype(x)>::type::Scalar;
//    return (M.template cast<Scalar>().eval() * x).eval();
    return x;
  };
  VectorX<T> v(3);
  math::jacobian(g, v);
  return MatrixX<T>();
*/
/*
  auto g = [&](const auto& x) {
//    return this->EvaluateNonlinearEquations(xt_copy, x, h).eval();
//    using Scalar = typename std::remove_reference<decltype(x)>::type::Scalar;
//    return (xt_copy - x).eval();
    context->get_mutable_continuous_state()->get_mutable_vector()->
      SetFromVector(x.template cast<T>());
    system.CalcTimeDerivatives(*context, derivs_.get());

  return (x - xt_copy - h*derivs_->CopyToVector()).eval();

  };
  auto result = math::jacobian(g, xtplus_copy);
  return math::autoDiffToGradientMatrix(result); 
//  return math::jacobian(g, xtplus_copy).template cast<Scalar>();
*/
  return MatrixX<T>();
}

// Computes the Jacobian of the residual "error" with respect to the state
// variables using numerical differentiation.
template <class T>
MatrixX<T> ImplicitEulerIntegrator<T>::ComputeNDiffJacobian(
    const VectorX<T>& xt, const VectorX<T>& xtplus, double h) {
  // Set epsilon to the square root of machine precision.
  double eps = std::sqrt(std::numeric_limits<double>::epsilon());

  Context<T>* context = this->get_mutable_context();
  const int n = context->get_continuous_state()->size();

  // Initialize the Jacobian.
  MatrixX<T> J(n, n);

  // Evaluate the equation xtplus - xt - h*f(t+h,xtplus) for the current xtplus.
  VectorX<T> g = EvaluateNonlinearEquations(xt, xtplus, h);

  // Compute the Jacobian.
  VectorX<T> xtplus_prime = xtplus;
  for (int i = 0; i < n; ++i) {
    // Compute a good increment to the dimension.
    const double abs_xi = std::abs(xtplus(i));
    double dxi = (abs_xi > 0) ? eps * abs_xi : eps;

    // Update xtplus', minimizing the effect of roundoff error.
    xtplus_prime(i) = xtplus(i) + dxi;
    dxi = xtplus_prime(i) - xtplus(i);

    // Compute g'.
    VectorX<T> gprime = EvaluateNonlinearEquations(xt, xtplus_prime, h);

    // Set the Jacobian column.
    J.col(i) = (gprime - g)/dxi;

    // Reset xtplus' to xtplus.
    xtplus_prime(i) = xtplus(i);
  }

  return J;
}

// Computes the Jacobian of the residual "error" with respect to the state
// variables using a second-order numerical differentiation.
// TODO(edrumwri): Write proper documentation for this.
template <class T>
MatrixX<T> ImplicitEulerIntegrator<T>::ComputeN2DiffJacobian(
    const VectorX<T>& xt, const VectorX<T>& xtplus, double h) {
  // Set epsilon to the square root of machine precision.
  double eps = std::sqrt(std::numeric_limits<double>::epsilon());

  Context<T>* context = this->get_mutable_context();
  const int n = context->get_continuous_state()->size();

  // Initialize the Jacobian.
  MatrixX<T> J(n, n);

  // Compute the Jacobian.
  VectorX<T> xtplus_prime = xtplus;
  for (int i = 0; i < n; ++i) {
    // Compute a good increment to the dimension.
    const double abs_xi = std::abs(xtplus(i));
    double dxi = (abs_xi > 0) ? eps * abs_xi : eps;

    // Update xtplus', minimizing the effect of roundoff error.
    xtplus_prime(i) = xtplus(i) + dxi;
    const double dxi_pos = xtplus_prime(i) - xtplus(i);

    // Compute g'.
    VectorX<T> gprime_pos = EvaluateNonlinearEquations(xt, xtplus_prime, h);

    // Update xtplus' again, minimizing the effect of roundoff error.
    xtplus_prime(i) = xtplus(i) - dxi;
    const double dxi_minus = xtplus(i) - xtplus_prime(i);

    // Compute g' again.
    VectorX<T> gprime_minus = EvaluateNonlinearEquations(xt, xtplus_prime, h);

    // Set the Jacobian column.
    J.col(i) = (gprime_pos - gprime_minus) / (dxi_pos + dxi_minus);

    // Reset xtplus' to xtplus.
    xtplus_prime(i) = xtplus(i);
  }

  return J;
}

// Evaluates x(t+h) - x(t) - h f(t+h, x(t+h))
template <class T>
VectorX<T> ImplicitEulerIntegrator<T>::EvaluateNonlinearEquations(
    const VectorX<T>& xt,
    const VectorX<T>& xtplus,
    double h) {

  // Get the system and the context.
  const auto& system = this->get_system();
  Context<T>* context = this->get_mutable_context();

  // Set the continuous state in the context.
  context->get_mutable_continuous_state()->get_mutable_vector()->
      SetFromVector(xtplus);

  // Evaluate the derivatives at that state.
  system.CalcTimeDerivatives(*context, derivs_.get());

  return xtplus - xt - h*derivs_->CopyToVector();
}

// Restores the integrator time and state to that at the start of the
// integration interval. This restoration is necessary when the integrator
// attempts (and fails) a trial step.
template <class T>
void ImplicitEulerIntegrator<T>::RestoreTimeAndState(const T& t,
                                                     const VectorX<T>& x) {
  Context<T>* context = this->get_mutable_context();
  context->set_time(t);
  context->get_mutable_continuous_state()->SetFromVector(x);
}

// Attempts a trial step of time length @p dt.
// @returns 'true' if successful, 'false' otherwise (typically occurs because
//          @p dt is too large to allow the nonlinear equation solver to
//          converge).
template <class T>
bool ImplicitEulerIntegrator<T>::DoTrialStep(const T& dt) {
  using std::abs;
  // TODO(edrumwri): Make this user-settable.
  const double dx_tol = 1e-8 * dt;

  // Presumptuously advance the context time; this means that all derivatives
  // will be computed at t+dt.
  Context<T>* context = this->get_mutable_context();
  double last_time = context->get_time();
  context->set_time(last_time + dt);

  // Get the current continuous state.
  const VectorX<T>& xt = context->get_continuous_state_vector().CopyToVector();

  // Use the current state as the candidate value for the next state.
  VectorX<T> xtplus = xt;

  // Setup g
  std::function<VectorX<T>(const VectorX<T>&)> g =
      [&xt, dt, this](const VectorX<T>& x) {
         return this->EvaluateNonlinearEquations(xt, x, dt);
  };

  // Evaluate g(x(t+h)) = x(t+h) - x(t) - h f(t+h,x(t+h)), where h = dt;
  VectorX<T> goutput = g(xtplus);

  // Set the initial value of the objective function f = g^Tg
  double f_last = 0.5*goutput.norm();

  // TODO(edrumwri): Routine assumes that typical values for x and g are on the
  // order of unity.

  // Evaluate the objective function.
  while (f_last > convergence_tol_) {
    // Form the Jacobian matrix ∂g/∂xtplus.
    MatrixX<T> J = ComputeN2DiffJacobian(xt, xtplus, dt);

    // The Jacobian matrix yields the relationship J dxtplus/dt = dg/dt.
    // Converting this from a derivative into a differential yields
    // J dxtplus = dg. Setting dg \equiv -g, we now solve this linear
    // equation for dxtplus, which gives a descent direction toward
    // reducing ||g||. J will be n x n, where n is the number of state
    // variables. It will generally possess no "nice" properties (e.g.,
    // symmetry) or even be guaranteed to be invertible.
    // TODO(edrumwri): Check for singular matrix?
//    VectorX<T> dx = J.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(-goutput);
    LU_.compute(J);
    VectorX<T> dx = LU_.solve(-goutput);

    // Compute 1/2 the gradient of g'*g and reject the integration step
    // size if the dot product of this gradient and dx is positive. Note that
    // there is no need to scale the gradient by the 1/2 factor, as we look
    // only for a positive dot product.
    const VectorX<T> grad = J * goutput;
/*
    if (grad.dot(dx) > 0) {
      num_misdirected_descents_++;
      RestoreTimeAndState(last_time, xt); 
      return false;
    }
*/

    // Search the ray \alpha \in [0, \infty] for a "good" value that minimizes
    // || x(t+h)_i + dx \alpha - x(t) - h f(t+h, x(t+h)_i + dx \alpha) ||. The
    // ray search is relativelly expensive, as it requres evaluating f().
    // Therefore, a good ray search will optimize the computational tradeoff
    // between finding the value of \alpha that minimizes the norm above and
    // forming the next Jacobian and solving the next linear system.
    typename LineSearch<T>::LineSearchOutput lout = 
        LineSearch<T>::search_line(xtplus, dx, f_last, grad, g);
    // Check to see whether the update is too close to zero. This test
    // effectively also covers the case where the gradient is zero.
    if ((xtplus - lout.x_new).template lpNorm<1>() < dx_tol) {
      RestoreTimeAndState(last_time, xt);
      num_overly_small_updates_++;
      return false;
    }

    // Update x, g_output, and f 
    goutput = lout.g_output;
    xtplus = lout.x_new;
    f_last = lout.f_new;
/*
    // Evaluate new xtplus and guard against increase in error.
    xtplus = xt + dx;
    goutput = g(xtplus);
    double f_new = 0.5*goutput.squaredNorm();
    if (f_new > f_last) {
      RestoreTimeAndState(last_time, xt);
      num_objective_function_increases_++;
      return false;
    }

    // Update f_last
    f_last = f_new;
*/
  }

  // Converged - last evaluation of g() updated the context to xt+.
  return true;
}

template <class T>
std::pair<bool, T> ImplicitEulerIntegrator<T>::DoStepOnceAtMost(
    const T& max_dt) {
  // Attempt taking the trial step until it succeeds (by halving steps). This
  // must succeed: at the limit as dt = 0, the nonlinear system is trivially
  // "solved".
  double dt = max_dt;
  if (DoTrialStep(dt)) {
    this->UpdateStatistics(dt);
    return std::make_pair(true, max_dt);
  }

  // Trial step didn't work- need to loop.
  // TODO(edrumwri): Account for minimum directed step size.
  do {
    dt /= 2;
  } while (!DoTrialStep(dt));
  this->UpdateStatistics(dt);
  return std::make_pair(false, dt);
}

/// Attempts to take a given step of the requested size.
/// @throws std::runtime_error if the nonlinear system solver fails for the
///         given step size.
template <class T>
void ImplicitEulerIntegrator<T>::DoStepOnceFixedSize(const T &dt) {
  if (!DoTrialStep(dt))
    throw std::runtime_error("Fixed step failed.");

  this->UpdateStatistics(dt);
}

}  // namespace systems
}  // namespace drake
