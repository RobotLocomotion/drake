#pragma once

/// @file
/// Template method implementations for runge_kutta_3_integrator.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include <utility>

#include "drake/systems/analysis/implicit_euler_integrator.h"

namespace drake {
namespace systems {

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
  system.CalcTimeDerivatives(context, derivs_.get());

  return xtplus - xt - h*derivs_->CopyToVector();
}

// Attempts a trial step of time length @p dt.
// @returns 'true' if successful, 'false' otherwise (typically occurs because
//          @p dt is too large to allow the nonlinear equation solver to
//          converge).
template <class T>
bool ImplicitEulerIntegrator<T>::DoTrialStep(const T& dt) {
  using std::abs;

  // Presumptuously advance the context time.
  Context<T>* context = get_mutable_context();
  double last_time = context->get_time();
  context->set_time(last_time + dt);

  // TODO(edrumwri): On failure, reset the context time.

  // Get the current continuous state.
  const VectorX<T>& xt = context->get_continuous_state_vector().CopyToVector();

  // Use the current state as the candidate value for the next state.
  // TODO(edrumwri): Experiment with using an explicit Euler step as the
  // candidate value for the next state.
  VectorX<T> xtplus = xt;

  // Evaluate the formula \phi = x(t+h) - x(t) - h f(t+h,x(t+h))
  VectorX<T> phi = EvaluateNonlinearEquations(xt, xtplus, dt);

  // Evaluate the objective function.
  while (phi.squaredNorm()/2 > convergence_tol_) {
    // Form the Jacobian matrix.

    // The Jacobian matrix yields the relationship
    // J dx/dt = dphi/dt. Converting this from a derivative into a differential
    // yields J dx = dphi. Setting dphi \equiv -phi, we now solve this linear
    // equation for dx.

    // See whether the gradient is effectively zero, meaning that the process
    // has converged prematurely.

    // Search the ray \alpha \in [0, \infty] for a "good" value that minimizes
    // || x(t+h)_i + dx \alpha - x(t) - h f(t+h, x(t+h)_i + dx \alpha) ||. The
    // ray search is relativelly expensive, as it requres evaluating f().
    // Therefore, a good ray search will optimize the computational tradeoff
    // between finding the value of \alpha that minimizes the norm above and
    // forming the next Jacobian and solving the next linear system.
    // TODO(edrumwri): phi.norm() should have already been calculated here!
  }

  // Converged.
  return true;
}

template <class T>
std::pair<bool, T> ImplicitEulerIntegrator<T>::DoStepOnceAtMost(
    const T& max_dt) {
  // TODO(edrumwri): Account for minimum directed step size.
  // Attempt taking the trial step until it succeeds (by halving steps). This
  // must succeed: at the limit as dt = 0, the nonlinear system is trivially
  // "solved".
  double dt = max_dt;
  while (!DoTrialStep(dt))
    dt /= 2;
}

/// Attempts to take a given step of the requested size.
/// @throws std::runtime_error if the nonlinear system solver fails for the
///         given step size.
template <class T>
void ImplicitEulerIntegrator<T>::DoStepOnceFixedSize(const T &dt) {
  if (!DoTrialStep(dt))
    throw std::runtime_error("Fixed step failed.");
}

}  // namespace systems
}  // namespace drake
