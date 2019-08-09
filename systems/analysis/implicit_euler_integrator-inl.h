#pragma once

/// @file
/// Template method implementations for implicit_euler_integrator.h.
/// Most users should only include that file, not this one.
/// For background, see https://drake.mit.edu/cxx_inl.html.

/* clang-format off to disable clang-format-includes */
#include "drake/systems/analysis/implicit_euler_integrator.h"
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
void ImplicitEulerIntegrator<T>::DoResetImplicitIntegratorStatistics() {
  num_nr_iterations_ = 0;
  num_err_est_nr_iterations_ = 0;
  num_err_est_function_evaluations_ = 0;
  num_err_est_jacobian_function_evaluations_ = 0;
  num_err_est_jacobian_reforms_ = 0;
  num_err_est_iter_factorizations_ = 0;
}

template <class T>
void ImplicitEulerIntegrator<T>::DoInitialize() {
  using std::isnan;

  // Allocate storage for changes to state variables during Newton-Raphson.
  dx_state_ = this->get_system().AllocateTimeDerivatives();

  const double kDefaultAccuracy = 1e-1;  // Good for this particular integrator.
  const double kLoosestAccuracy = 5e-1;  // Loosest accuracy is quite loose.

  // Set an artificial step size target, if not set already.
  if (isnan(this->get_initial_step_size_target())) {
    // Verify that maximum step size has been set.
    if (isnan(this->get_maximum_step_size()))
      throw std::logic_error("Neither initial step size target nor maximum "
                                 "step size has been set!");

    this->request_initial_step_size_target(
        this->get_maximum_step_size());
  }

  // Sets the working accuracy to a good value.
  double working_accuracy = this->get_target_accuracy();

  // If the user asks for accuracy that is looser than the loosest this
  // integrator can provide, use the integrator's loosest accuracy setting
  // instead.
  if (isnan(working_accuracy))
    working_accuracy = kDefaultAccuracy;
  else if (working_accuracy > kLoosestAccuracy)
    working_accuracy = kLoosestAccuracy;
  this->set_accuracy_in_use(working_accuracy);

  // Reset the Jacobian matrix (so that recomputation is forced).
  this->get_mutable_jacobian().resize(0, 0);

  // Initialize the embedded second order Runge-Kutta integrator. The maximum
  // step size will be set to infinity because we will explicitly request the
  // step sizes to be taken.
  rk2_ = std::make_unique<RungeKutta2Integrator<T>>(
      this->get_system(),
      std::numeric_limits<double>::infinity() /* maximum step size */,
      this->get_mutable_context());
}

template <class T>
void ImplicitEulerIntegrator<T>::ComputeAndFactorImplicitEulerIterationMatrix(
    const MatrixX<T>& J, const T& h,
    typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix) {
  const int n = J.rows();
  // TODO(edrumwri) Investigate using a move-type operation below.
  // We form the iteration matrix in this particular way to avoid an O(n^2)
  // subtraction as would be the case with:
  // MatrixX<T>::Identity(n, n) - J * h.
  iteration_matrix->SetAndFactorIterationMatrix(
      J * -h + MatrixX<T>::Identity(n, n));
}

template <class T>
void ImplicitEulerIntegrator<T>::
ComputeAndFactorImplicitTrapezoidIterationMatrix(
    const MatrixX<T>& J, const T& h,
    typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix) {
  const int n = J.rows();
  // TODO(edrumwri) Investigate using a move-type operation below.
  // We form the iteration matrix in this particular way to avoid an O(n^2)
  // subtraction as would be the case with:
  // MatrixX<T>::Identity(n, n) - J * h / 2.
  iteration_matrix->SetAndFactorIterationMatrix(
      J * (-h / 2.0) + MatrixX<T>::Identity(n, n));
}

// Performs the bulk of the stepping computation for both implicit Euler and
// implicit trapezoid method; all those methods need to do is provide a
// residual function (`g`) and an iteration matrix computation and factorization
// function (`compute_and_factor_iteration_matrix`) specific to the
// particular integrator scheme and this method does the rest.
// @param t0 the time at the left end of the integration interval.
// @param h the integration step size (> 0) to attempt.
// @param xt0 the continuous state at t0.
// @param g the particular implicit function to compute the root of.
// @param [in, out] the starting guess for x(t0+h); the value for x(t0+h) on
//        return.
// @param trial the attempt for this approach (1-4). StepAbstract() uses more
//        computationally expensive methods as the trial numbers increase.
// @returns `true` if the method was successfully able to take an integration
//           step of size h (or `false` otherwise).
// @note The time and continuous state in the context are indeterminate upon
//       exit.
// TODO(edrumwri) Explicitly test this method's fallback logic (i.e., how it
//                calls MaybeFreshenMatrices()) in a unit test).
template <class T>
bool ImplicitEulerIntegrator<T>::StepAbstract(const T& t0, const T& h,
    const VectorX<T>& xt0, const std::function<VectorX<T>()>& g,
    const std::function<void(const MatrixX<T>&, const T&,
        typename ImplicitIntegrator<T>::IterationMatrix*)>&
        compute_and_factor_iteration_matrix,
    VectorX<T>* xtplus, int trial) {
  using std::max;
  using std::min;

  // Verify the trial number is valid.
  DRAKE_ASSERT(trial >= 1 && trial <= 4);

  // Verify xtplus
  DRAKE_ASSERT(xtplus && xtplus->size() == xt0.size());

  SPDLOG_DEBUG(drake::log(), "StepAbstract() entered for t={}, h={}, trial={}",
      t0, h, trial);

  // Advance the context time and state to compute derivatives at t0 + h.
  const T tf = t0 + h;
  Context<T>* context = this->get_mutable_context();
  context->SetTimeAndContinuousState(tf, *xtplus);

  // Evaluate the residual error using:
  // g(x(t0+h)) = x(t0+h) - x(t0) - h f(t0+h,x(t0+h)).
  VectorX<T> goutput = g();

  // Initialize the "last" state update norm; this will be used to detect
  // convergence.
  T last_dx_norm = std::numeric_limits<double>::infinity();

  // Calculate Jacobian and iteration matrices (and factorizations), as needed,
  // around (tf, xtplus).
  if (!this->MaybeFreshenMatrices(tf, *xtplus, h, trial,
      compute_and_factor_iteration_matrix, &iteration_matrix_)) {
    return false;
  }

  // Do the Newton-Raphson iterations.
  for (int i = 0; i < this->max_newton_raphson_iterations(); ++i) {
    // Update the number of Newton-Raphson iterations.
    num_nr_iterations_++;

    // Compute the state update using the equation A*x = -g(), where A is the
    // iteration matrix.
    // TODO(edrumwri): Allow caller to provide their own solver.
    VectorX<T> dx = iteration_matrix_.Solve(-goutput);

    // Get the infinity norm of the weighted update vector.
    dx_state_->get_mutable_vector().SetFromVector(dx);
    T dx_norm = this->CalcStateChangeNorm(*dx_state_);

    // The check below looks for convergence by identifying cases where the
    // update to the state results in no change. We do this check only after
    // at least one Newton-Raphson update has been applied to ensure that there
    // is at least some change to the state, no matter how small, on a
    // non-stationary system.
    if (i > 0 && this->IsUpdateZero(*xtplus, dx))
      return true;

    // Update the state vector.
    *xtplus += dx;
    context->SetTimeAndContinuousState(tf, *xtplus);

    // Compute the convergence rate and check convergence.
    // [Hairer, 1996] notes that this convergence strategy should only be
    // applied after *at least* two iterations (p. 121).
    if (i >= 1) {
      const T theta = dx_norm / last_dx_norm;
      const T eta = theta / (1 - theta);
      SPDLOG_DEBUG(drake::log(), "Newton-Raphson loop {} theta: {}, eta: {}",
                   i, theta, eta);

      // Look for divergence.
      if (theta > 1) {
        SPDLOG_DEBUG(drake::log(), "Newton-Raphson divergence detected for "
            "h={}", h);
        break;
      }

      // Look for convergence using Equation 8.10 from [Hairer, 1996].
      // [Hairer, 1996] determined values of kappa in [0.01, 0.1] work most
      // efficiently on a number of test problems with Radau5 (a fifth order
      // implicit integrator), p. 121. We select a value halfway in-between.
      const double kappa = 0.05;
      const double k_dot_tol = kappa * this->get_accuracy_in_use();
      if (eta * dx_norm < k_dot_tol) {
        SPDLOG_DEBUG(drake::log(), "Newton-Raphson converged; η = {}, h = {}",
                     eta, h);
        return true;
      }
    }

    // Update the norm of the state update.
    last_dx_norm = dx_norm;

    // Update the state in the context and compute g(xⁱ⁺¹).
    goutput = g();
  }

  SPDLOG_DEBUG(drake::log(), "StepAbstract() convergence failed");

  // If Jacobian and iteration matrix factorizations are not reused, there
  // is nothing else we can try.
  if (!this->get_reuse())
    return false;

  // Try StepAbstract again, freshening Jacobians and iteration matrix
  // factorizations as helpful.
  return StepAbstract(
      t0, h, xt0, g, compute_and_factor_iteration_matrix, xtplus, trial+1);
}

// Steps the system forward by a single step of at most h using the implicit
// Euler method.
// @param t0 the time at the left end of the integration interval.
// @param h the maximum time increment to step forward.
// @param xt0 the continuous state at t0.
// @param [in,out] xtplus a guess for the continuous state at the right end of
//                 the integration interval (i.e., `x(t0+h)`), on entry, and
//                 the computed value for `x(t0+h)` on successful return.
// @returns `true` if the step of size `h` was successful, `false` otherwise.
// @note The time and continuous state in the context are indeterminate upon
//       exit.
template <class T>
bool ImplicitEulerIntegrator<T>::StepImplicitEuler(const T& t0, const T& h,
    const VectorX<T>& xt0, VectorX<T>* xtplus) {
  using std::abs;

  SPDLOG_DEBUG(drake::log(), "StepImplicitEuler(h={}) t={}", h, t0);

  // Set g for the implicit Euler method.
  Context<T>* context = this->get_mutable_context();
  std::function<VectorX<T>()> g =
      [&xt0, h, context, this]() {
        return (context->get_continuous_state().CopyToVector() - xt0 -
            h * this->EvalTimeDerivatives(*context).CopyToVector()).eval();
      };

  // Use the current state as the candidate value for the next state.
  // [Hairer 1996] validates this choice (p. 120).
  *xtplus = xt0;

  // Attempt the step.
  return StepAbstract(
      t0, h, xt0, g, ComputeAndFactorImplicitEulerIterationMatrix, &*xtplus);
}

// Steps forward by a single step of `h` using the implicit trapezoid
// method, if possible.
// @param t0 the time at the left end of the integration interval.
// @param h the maximum time increment to step forward.
// @param dx0 the time derivatives computed at time and state (t0, xt0).
// @param [in,out] xtplus the continuous state at the right end of the
//                 integration (i.e., x(t0+h)) computed by implicit Euler
//                 on entry; x(t0+h) computed by the implicit trapezoid method
//                 on successful return.
// @returns `true` if the step was successful and `false` otherwise.
// @note The time and continuous state in the context are indeterminate upon
//       exit.
template <class T>
bool ImplicitEulerIntegrator<T>::StepImplicitTrapezoid(const T& t0, const T& h,
    const VectorX<T>& xt0, const VectorX<T>& dx0, VectorX<T>* xtplus) {
  using std::abs;

  SPDLOG_DEBUG(drake::log(), "StepImplicitTrapezoid(h={}) t={}", h, t0);

  // Set g for the implicit trapezoid method.
  // Define g(x(t+h)) ≡ x(t0+h) - x(t0) - h/2 (f(t0,x(t0)) + f(t0+h,x(t0+h)) and
  // evaluate it at the current x(t+h).
  Context<T>* context = this->get_mutable_context();
  std::function<VectorX<T>()> g =
      [&xt0, h, &dx0, context, this]() {
        return (context->get_continuous_state().CopyToVector() - xt0 - h/2 *
            (dx0 + this->EvalTimeDerivatives(
                *context).CopyToVector().eval())).eval();
      };

  // Store statistics before calling StepAbstract(). The difference between
  // the modified statistics and the stored statistics will be used to compute
  // the trapezoid method-specific statistics.
  int stored_num_jacobian_evaluations = this->get_num_jacobian_evaluations();
  int stored_num_iter_factorizations =
      this->get_num_iteration_matrix_factorizations();
  int64_t stored_num_function_evaluations =
      this->get_num_derivative_evaluations();
  int64_t stored_num_jacobian_function_evaluations =
      this->get_num_derivative_evaluations_for_jacobian();
  int stored_num_nr_iterations = this->get_num_newton_raphson_iterations();

  // Attempt to step.
  bool success = StepAbstract(
      t0, h, xt0, g, ComputeAndFactorImplicitTrapezoidIterationMatrix, xtplus);

  // Move statistics to implicit trapezoid-specific.
  num_err_est_jacobian_reforms_ +=
      this->get_num_jacobian_evaluations() - stored_num_jacobian_evaluations;
  num_err_est_iter_factorizations_ +=
      this->get_num_iteration_matrix_factorizations() -
      stored_num_iter_factorizations;
  num_err_est_function_evaluations_ +=
      this->get_num_derivative_evaluations() - stored_num_function_evaluations;
  num_err_est_jacobian_function_evaluations_ +=
      this->get_num_derivative_evaluations_for_jacobian() -
      stored_num_jacobian_function_evaluations;
  num_err_est_nr_iterations_ += this->get_num_newton_raphson_iterations() -
      stored_num_nr_iterations;

  return success;
}

// Steps both implicit Euler and implicit trapezoid forward by h, if possible.
// @param t0 the time at the left end of the integration interval.
// @param h the integration step size to attempt.
// @param [out] xtplus_ie contains the Euler integrator solution (i.e.,
//              `x(t0+h)`) on return.
// @param [out] xtplus_itr contains the implicit trapezoid solution (i.e.,
//              `x(t0+h)`) on return.
// @returns `true` if the step of size `h` was successful, `false` otherwise.
template <class T>
bool ImplicitEulerIntegrator<T>::AttemptStepPaired(const T& t0, const T& h,
    const VectorX<T>& xt0, VectorX<T>* xtplus_ie, VectorX<T>* xtplus_itr) {
  using std::abs;
  DRAKE_ASSERT(xtplus_ie);
  DRAKE_ASSERT(xtplus_itr);

  // Compute the derivative at time and state (t0, x(t0)). NOTE: the derivative
  // is calculated at this point (early on in the integration process) in order
  // to reuse the derivative evaluation, via the cache, from the last
  // integration step (if possible).
  const VectorX<T> dx0 = this->EvalTimeDerivatives(
      this->get_context()).CopyToVector();

  // Do the Euler step.
  if (!StepImplicitEuler(t0, h, xt0, xtplus_ie)) {
    SPDLOG_DEBUG(drake::log(), "Implicit Euler approach did not converge for "
        "step size {}", h);
    return false;
  }

  // The error estimation process uses the implicit trapezoid method, which
  // is defined as:
  // x(t0+h) = x(t0) + h/2 (f(t0, x(t0) + f(t0+h, x(t0+h))
  // x(t0+h) from the implicit Euler method is presumably a good starting point.

  // The error estimate is derived as follows (thanks to Michael Sherman):
  // x*(t0+h) = xᵢₑ(t0+h) + O(h²)      [implicit Euler]
  //          = xₜᵣ(t0+h) + O(h³)      [implicit trapezoid]
  // where x*(t0+h) is the true (generally unknown) answer that we seek.
  // This implies:
  // xᵢₑ(t0+h) + O(h²) = xₜᵣ(t0+h) + O(h³).
  // Given that the second order term subsumes the third order one, we have:
  // xᵢₑ(t0+h) - xₜᵣ(t0+h) = O(h²).

  // Attempt to compute the implicit trapezoid solution.
  *xtplus_itr = *xtplus_ie;
  if (StepImplicitTrapezoid(t0, h, xt0, dx0, xtplus_itr)) {
    // Reset the state to that computed by implicit Euler.
    // TODO(edrumwri): Explore using the implicit trapezoid method solution
    //                 instead as *the* solution, rather than the implicit
    //                 Euler. Refer to [Lambert, 1991], Ch 6.
    Context<T>* context = this->get_mutable_context();
    context->SetTimeAndContinuousState(t0 + h, *xtplus_ie);
    return true;
  } else {
    SPDLOG_DEBUG(drake::log(), "Implicit trapezoid approach FAILED with a step"
        "size that succeeded on implicit Euler.");
    return false;
  }
}

/// Takes a given step of the requested size, if possible.
/// @returns `true` if successful and `false` otherwise; on `true`, the time
///          and continuous state will be advanced in the context (e.g., from
///          t0 to t0 + h). On `false` return, the time and continuous state in
///          the context will be restored to its original value (at t0).
template <class T>
bool ImplicitEulerIntegrator<T>::DoImplicitIntegratorStep(const T& h) {
  // Save the current time and state.
  Context<T>* context = this->get_mutable_context();
  const T t0 = context->get_time();
  SPDLOG_DEBUG(drake::log(), "IE DoStep(h={}) t={}", h, t0);

  xt0_ = context->get_continuous_state().CopyToVector();
  xtplus_ie_.resize(xt0_.size());
  xtplus_tr_.resize(xt0_.size());

  // If the requested h is less than the minimum step size, we'll advance time
  // using an explicit Euler step.
  if (h < this->get_working_minimum_step_size()) {
    SPDLOG_DEBUG(drake::log(), "-- requested step too small, taking explicit "
        "step instead");

    // TODO(edrumwri): Investigate replacing this with an explicit trapezoid
    //                 step, which would be expected to give better accuracy.
    //                 The mitigating factor is that h is already small, so a
    //                 test of, e.g., a square wave function, should quantify
    //                 the improvement (if any).

    // The error estimation process for explicit Euler uses an explicit second
    // order Runge-Kutta method so that the order of the asymptotic term
    // matches that used for estimating the error of the implicit Euler
    // integrator.

    // Compute the Euler step.
    xdot_ = this->EvalTimeDerivatives(*context).CopyToVector();
    xtplus_ie_ = xt0_ + h * xdot_;

    // Compute the RK2 step.
    const int evals_before_rk2 = rk2_->get_num_derivative_evaluations();
    if (!rk2_->IntegrateWithSingleFixedStepToTime(t0 + h)) {
      throw std::runtime_error("Embedded RK2 integrator failed to take a single"
          "fixed step to the requested time.");
    }

    const int evals_after_rk2 = rk2_->get_num_derivative_evaluations();
    xtplus_tr_ = context->get_continuous_state().CopyToVector();

    // Update the error estimation ODE counts.
    num_err_est_function_evaluations_ += (evals_after_rk2 - evals_before_rk2);

    // Revert the state to that computed by explicit Euler.
    context->SetTimeAndContinuousState(t0 + h, xtplus_ie_);
  } else {
    // Try taking the requested step.
    bool success = AttemptStepPaired(t0, h, xt0_, &xtplus_ie_, &xtplus_tr_);

    // If the step was not successful, reset the time and state.
    if (!success) {
      context->SetTimeAndContinuousState(t0, xt0_);
      return false;
    }
  }

  // Compute and update the error estimate.
  err_est_vec_ = xtplus_ie_ - xtplus_tr_;

  // Update the caller-accessible error estimate.
  this->get_mutable_error_estimate()->get_mutable_vector().
      SetFromVector(err_est_vec_);

  return true;
}

}  // namespace systems
}  // namespace drake
