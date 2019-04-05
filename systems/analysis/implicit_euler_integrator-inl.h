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
  num_iter_factorizations_ = 0;
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
}

// Computes any necessary matrices (Jacobian, iteration matrix, or both)
// for the Newton-Raphson iteration in StepAbstract() around time/state `(t,xt)`
// and using the same `scale` parameter as in StepAbstract().
// @returns `false` if the calling StepAbstract method should indicate failure;
//          `true` otherwise.
template <class T>
bool ImplicitEulerIntegrator<T>::CalcMatrices(const T& t, const T& h,
    const VectorX<T>& xt, int scale, int trial) {
  // Compute the initial Jacobian and negated iteration matrices (see
  // rationale for the negation below) and factor them, if necessary.
  MatrixX<T>& J = this->get_mutable_jacobian();
  if (!this->get_reuse() || J.rows() == 0 || this->IsBadJacobian(J)) {
    // The Jacobian can become bad through a divergent Newton-Raphson
    // iteration, which causes the state to overflow, which then causes the
    // Jacobian to overflow. If the state overflows, recomputing the Jacobian
    // using this bad state will result in another bad Jacobian, eventually
    // causing DoStep() to return indicating failure (but not before resetting
    // the continuous state to its previous, good value). DoStep() will then
    // be called again with a smaller step size and the good state; the
    // bad Jacobian will then be corrected.
    J = this->CalcJacobian(t, xt);
    ComputeAndFactorIterationMatrix(J, h, scale);
    return true;
  }

  switch (trial) {
    case 1:
      // For the first trial, we do nothing special.
      return true;

    case 2: {
      // For the second trial, re-construct and factor the iteration matrix.
      ComputeAndFactorIterationMatrix(J, h, scale);
      return true;
    }

    case 3: {
      // If the last call to StepAbstract() ended in failure, we know that
      // the Jacobian matrix is fresh and the iteration matrix has been newly
      // formed and factored (on Trial #2), so there is nothing more to be
      // done.
      if (!this->last_call_succeeded()) {
        return false;
      } else {
        // Reform the Jacobian matrix and refactor the iteration matrix.
        J = this->CalcJacobian(t, xt);
        ComputeAndFactorIterationMatrix(J, h, scale);
      }
      return true;

      case 4: {
        // Trial #4 indicates failure.
        return false;
      }

      default:
        throw std::domain_error("Unexpected trial number.");
    }
  }
}

template <class T>
void ImplicitEulerIntegrator<T>::ComputeAndFactorIterationMatrix(
  const MatrixX<T>& J, const T& h, int scale) {
    ++num_iter_factorizations_;
    const int n = J.rows();
    // TODO(edrumwri) Investigate using a move-type operation below.
    // We form the iteration matrix in this particular way to avoid an O(n^2)
    // subtraction as would be the case with:
    // MatrixX<T>::Identity(n, n) - J * (h / scale).
    iteration_matrix_.SetAndFactorIterationMatrix(
        J * (-h / static_cast<double>(scale)) + MatrixX<T>::Identity(n, n));
}

// Performs the bulk of the stepping computation for both implicit Euler and
// implicit trapezoid method; all those methods need to do is provide a
// residual function (`g`) and a scale factor (`scale`) specific to the
// particular integrator scheme and this method does the rest.
// @param t0 the time at the left end of the integration interval.
// @param h the integration step size (> 0) to attempt.
// @param xt0 the continuous state at t0.
// @param g the particular implicit function to compute the root of.
// @param scale a scale factor- either 1 or 2- that allows this method to be
//        used by both implicit Euler and implicit trapezoid methods.
// @param [in, out] the starting guess for x(t0+h); the value for x(t0+h) on
//        return.
// @param trial the attempt for this approach (1-4). StepAbstract() uses more
//        computationally expensive methods as the trial numbers increase.
// @returns `true` if the method was successfully able to take an integration
//           step of size h (or `false` otherwise).
// @note The time and continuous state in the context are indeterminate upon
//       exit.
template <class T>
bool ImplicitEulerIntegrator<T>::StepAbstract(const T& t0, const T& h,
    const VectorX<T>& xt0, const std::function<VectorX<T>()>& g,
    int scale, VectorX<T>* xtplus, int trial) {
  using std::max;
  using std::min;

  // Verify the trial number is valid.
  DRAKE_ASSERT(trial >= 1 && trial <= 4);

  // Verify the scale factor is correct.
  DRAKE_ASSERT(scale == 1 || scale == 2);

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
  if (!CalcMatrices(tf, h, *xtplus, scale, trial)) {
    this->set_last_call_succeeded(false);
    return false;
  }

  // The maximum number of Newton-Raphson iterations to take before declaring
  // failure. [Hairer, 1996] states, "It is our experience that the code becomes
  // more efficient when we allow a relatively high number of iterations (e.g.,
  // [7 or 10])", p. 121.  The focus of that quote is a higher order integrator
  // with a quasi-Newton approach, so our mileage may vary.
  // TODO(edrumwri): Consider making this a settable parameter. Not putting it
  //                 toward staving off parameter overload.
  const int max_iterations = 10;

  // Do the Newton-Raphson iterations.
  for (int i = 0; i < max_iterations; ++i) {
    // Update the number of Newton-Raphson iterations.
    num_nr_iterations_++;

    // Compute the state update using the equation A*x = -g(), where A is the
    // iteration matrix.
    // TODO(edrumwri): Allow caller to provide their own solver.
    VectorX<T> dx = iteration_matrix_.Solve(-goutput);

    // Get the infinity norm of the weighted update vector.
    dx_state_->get_mutable_vector().SetFromVector(dx);
    T dx_norm = this->CalcStateChangeNorm(*dx_state_);

    // Update the state vector.
    *xtplus += dx;
    context->SetTimeAndContinuousState(tf, *xtplus);

    // The check below looks for convergence using machine epsilon. Without
    // this check, the convergence criteria can be applied when
    // |dx_norm| ~ 1e-22 (one example taken from practice), which does not
    // allow the norm to be reduced further. What happens: dx_norm will become
    // equivalent to last_dx_norm, making theta = 1, and eta = infinity. Thus,
    // convergence would never be identified.
    if (dx_norm < 10 * std::numeric_limits<double>::epsilon()) {
      this->set_last_call_succeeded(true);
      return true;
    }

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
        this->set_last_call_succeeded(true);
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
  if (!this->get_reuse()) {
    this->set_last_call_succeeded(false);
    return false;
  }

  // Try StepAbstract again, freshening Jacobians and iteration matrix
  // factorizations as helpful.
  return StepAbstract(t0, h, xt0, g, scale, xtplus, trial+1);
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
            h * this->EvalTimeDerivativesUsingContext()).eval();
      };

  // Use the current state as the candidate value for the next state.
  // [Hairer 1996] validates this choice (p. 120).
  *xtplus = xt0;

  // Attempt the step.
  return StepAbstract(t0, h, xt0, g, 1, &*xtplus);
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
            (dx0 + this->EvalTimeDerivativesUsingContext().eval())).eval();
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
  bool success = StepAbstract(t0, h, xt0, g, 2, xtplus);

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
  const VectorX<T> dx0 = this->EvalTimeDerivativesUsingContext();

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
  // xᵢₑ(t0+h) + O(h²) = xₜᵣ(t0+h) + O(h³)
  // Given that the second order term subsumes the third order one:
  // xᵢₑ(t0+h) - xₜᵣ(t0+h) = O(h²)
  // Therefore the difference between the implicit trapezoid solution and the
  // implicit Euler solution gives a second-order error estimate for the
  // implicit Euler result xᵢₑ.

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
bool ImplicitEulerIntegrator<T>::DoStep(const T& h) {
  // Save the current time and state.
  Context<T>* context = this->get_mutable_context();
  const T t0 = context->get_time();
  SPDLOG_DEBUG(drake::log(), "IE DoStep(h={}) t={}", h, t0);

  // TODO(sherm1) Heap allocation here; consider mutable temporaries instead.
  const VectorX<T> xt0 = context->get_continuous_state().CopyToVector();
  VectorX<T> xtplus_ie(xt0.size()), xtplus_itr(xt0.size());

  // If the requested h is less than the minimum step size, we'll advance time
  // using two explicit Euler steps of size h/2. For error estimation, we also
  // take a single step of size h. We can estimate the error in the larger
  // step that way, but note that we propagate the two half-steps on the
  // assumption the result will be better despite not having an error estimate
  // for them (that's called "local extrapolation").
  if (h < this->get_working_minimum_step_size()) {
    SPDLOG_DEBUG(drake::log(), "-- requested step too small, taking explicit "
        "step instead");

    // TODO(edrumwri): Investigate replacing this with an explicit trapezoid
    //                 step, which would be expected to give better accuracy.
    //                 The mitigating factor is that h is already small, so a
    //                 test of, e.g., a square wave function, should quantify
    //                 the improvement (if any).

    // The error estimation process for explicit Euler uses two half-steps
    // of explicit Euler (for a total of two derivative evaluations). The error
    // estimation process is derived as follows:
    // (1) x*(t0+h) = xₑ(t0+h) + h²/2 df/h + ...                [full step]
    // (2) x*(t0+h) = ̅xₑ(t0+h) + h²/8 (df/h + df₊/h) + ...      [two 1/2-steps]
    //
    // where x*(t0+h) is the true (generally unknown) answer that we seek,
    // f() is the ordinary differential equation evaluated at x(t0), and
    // f₊() is the derivative evaluated at x(t0 + h/2). Subtracting (1) from
    // (2), the above equations are rewritten as:
    // 0 = x̅ₑ(t0+h) - xₑ(t0+h) + h²/8 (-3df/h + df₊/h) + ...
    // The sum of all but the first two terms on the right hand side
    // of the above equation is less in magnitude than ch², for some
    // sufficiently large c. Or, written using Big-Oh notation:
    // x̅ₑ(t0+h) - xₑ(t0+h) = O(h²)
    // Thus, subtracting the two solutions yields a second order error estimate
    // (we compute norms on the error estimate, so the apparent sign error
    // in the algebra when arriving at the final equation is inconsequential).

    // Compute the Euler step.
    // TODO(sherm1) Heap allocation here; consider mutable temporary instead.
    VectorX<T> xdot = this->EvalTimeDerivatives(*context).CopyToVector();
    xtplus_ie = xt0 + h * xdot;

    // Do one half step.
    // TODO(sherm1) Heap allocation here; consider mutable temporary instead.
    const VectorX<T> xtpoint5 = xt0 + h / 2.0 * xdot;
    context->SetTimeAndContinuousState(t0 + h / 2.0, xtpoint5);

    // Do another half step, then set the "trapezoid" state to be the result of
    // taking two explicit Euler half steps. The code below the if/then/else
    // block simply subtracts the two results to obtain the error estimate.
    xdot = this->EvalTimeDerivatives(*context).CopyToVector();  // xdot(t + h/2)
    xtplus_itr = xtpoint5 + h / 2.0 * xdot;
    context->SetTimeAndContinuousState(t0 + h, xtplus_itr);

    // Update the error estimation ODE counts.
    num_err_est_function_evaluations_ += 2;
  } else {
    // Try taking the requested step.
    bool success = AttemptStepPaired(t0, h, xt0, &xtplus_ie, &xtplus_itr);

    // If the step was not successful, reset the time and state.
    if (!success) {
      context->SetTimeAndContinuousState(t0, xt0);
      return false;
    }
  }

  // Compute and update the error estimate.
  err_est_vec_ = xtplus_ie - xtplus_itr;

  // Update the caller-accessible error estimate.
  this->get_mutable_error_estimate()->get_mutable_vector().
      SetFromVector(err_est_vec_);

  return true;
}

}  // namespace systems
}  // namespace drake
