#include "drake/systems/analysis/implicit_euler_integrator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"

namespace drake {
namespace systems {

template <class T>
void ImplicitEulerIntegrator<T>::DoResetImplicitIntegratorStatistics() {
  num_nr_iterations_ = 0;
  hie_statistics_ = {};
  itr_statistics_ = {};
}

template <class T>
void ImplicitEulerIntegrator<T>::DoResetCachedJacobianRelatedMatrices() {
  ie_iteration_matrix_ = {};
  itr_iteration_matrix_ = {};
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

template <class T>
bool ImplicitEulerIntegrator<T>::StepAbstract(
    const T& t0, const T& h, const VectorX<T>& xt0,
    const std::function<VectorX<T>()>& g,
    const std::function<void(const MatrixX<T>&, const T&,
                             typename ImplicitIntegrator<T>::IterationMatrix*)>&
        compute_and_factor_iteration_matrix,
    const VectorX<T>& xtplus_guess,
    typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix,
    VectorX<T>* xtplus, int trial) {
  using std::max;
  using std::min;

  // Verify the trial number is valid.
  DRAKE_ASSERT(trial >= 1 && trial <= 4);

  // Verify xtplus
  DRAKE_ASSERT(xtplus && xtplus->size() == xt0.size());

  DRAKE_LOGGER_DEBUG("StepAbstract() entered for t={}, h={}, trial={}",
      t0, h, trial);

  // Start from the guess.
  *xtplus = xtplus_guess;
  DRAKE_LOGGER_DEBUG("Starting state: {}", xtplus->transpose());

  // Advance the context time and state to compute derivatives at t0 + h.
  const T tf = t0 + h;
  Context<T>* context = this->get_mutable_context();
  context->SetTimeAndContinuousState(tf, *xtplus);

  // Initialize the "last" state update norm; this will be used to detect
  // convergence.
  T last_dx_norm = std::numeric_limits<double>::infinity();

  // Calculate Jacobian and iteration matrices (and factorizations), as needed,
  // around (t0, xt0). We do not do this calculation if full Newton is in use;
  // the calculation will be performed at the beginning of the loop instead.
  // TODO(edrumwri) Consider computing the Jacobian matrix around tf and/or
  //                xtplus. This would give a better Jacobian, but would
  //                complicate the logic, since the Jacobian would no longer
  //                (necessarily) be fresh upon fallback to a smaller step size.
  if (!this->get_use_full_newton() &&
      !this->MaybeFreshenMatrices(t0, xt0, h, trial,
                                  compute_and_factor_iteration_matrix,
                                  iteration_matrix)) {
    return false;
  }

  // Do the Newton-Raphson iterations.
  for (int i = 0; i < this->max_newton_raphson_iterations(); ++i) {
    this->FreshenMatricesIfFullNewton(tf, *xtplus, h,
                                      compute_and_factor_iteration_matrix,
                                      iteration_matrix);

    // Evaluate the residual error using:
    // g(x(t0+h)) = x(t0+h) - x(t0) - h f(t0+h,x(t0+h)).
    VectorX<T> goutput = g();

    // Update the number of Newton-Raphson iterations.
    num_nr_iterations_++;

    // Compute the state update using the equation A*x = -g(), where A is the
    // iteration matrix.
    // TODO(edrumwri): Allow caller to provide their own solver.
    VectorX<T> dx = iteration_matrix->Solve(-goutput);

    // Get the infinity norm of the weighted update vector.
    dx_state_->get_mutable_vector().SetFromVector(dx);
    T dx_norm = this->CalcStateChangeNorm(*dx_state_);

    // Update the state vector.
    *xtplus += dx;
    context->SetTimeAndContinuousState(tf, *xtplus);

    // Check for Newton-Raphson convergence.
    typename ImplicitIntegrator<T>::ConvergenceStatus status =
        this->CheckNewtonConvergence(i, *xtplus, dx, dx_norm, last_dx_norm);
    // If it converged, we're done.
    if (status == ImplicitIntegrator<T>::ConvergenceStatus::kConverged)
      return true;
    // If it diverged, we have to abort and try again.
    if (status == ImplicitIntegrator<T>::ConvergenceStatus::kDiverged)
      break;
    // Otherwise, continue to the next Newton-Raphson iteration.
    DRAKE_DEMAND(status ==
                 ImplicitIntegrator<T>::ConvergenceStatus::kNotConverged);

    // Update the norm of the state update.
    last_dx_norm = dx_norm;
  }

  DRAKE_LOGGER_DEBUG("StepAbstract() convergence failed");

  // If Jacobian and iteration matrix factorizations are not reused, there
  // is nothing else we can try.  Note that get_reuse() returns false if
  // "full Newton-Raphson" mode is activated (see
  // ImplicitIntegrator::get_use_full_newton()).
  if (!this->get_reuse())
    return false;

  // Try StepAbstract again. That method will freshen Jacobians and iteration
  // matrix factorizations as necessary.
  return StepAbstract(t0, h, xt0, g, compute_and_factor_iteration_matrix,
                      xtplus_guess, iteration_matrix, xtplus, trial + 1);
}

template <class T>
bool ImplicitEulerIntegrator<T>::StepImplicitEuler(const T& t0, const T& h,
    const VectorX<T>& xt0, VectorX<T>* xtplus) {
  DRAKE_LOGGER_DEBUG("StepImplicitEuler(h={}) t={}", h, t0);

  // Use the current state as the candidate value for the next state.
  // [Hairer 1996] validates this choice (p. 120).
  const VectorX<T>& xtplus_guess = xt0;

  return this->StepImplicitEulerWithGuess(t0, h, xt0, xtplus_guess, xtplus);
}

template <class T>
bool ImplicitEulerIntegrator<T>::StepImplicitEulerWithGuess(
    const T& t0, const T& h, const VectorX<T>& xt0,
    const VectorX<T>& xtplus_guess, VectorX<T>* xtplus) {
  using std::abs;

  DRAKE_LOGGER_DEBUG("StepImplicitEulerWithGuess(h={}) t={}", h, t0);

  // Set g for the implicit Euler method.
  Context<T>* context = this->get_mutable_context();
  std::function<VectorX<T>()> g =
      [&xt0, h, context, this]() {
        return (context->get_continuous_state().CopyToVector() - xt0 -
            h * this->EvalTimeDerivatives(*context).CopyToVector()).eval();
      };

  // Attempt the step.
  return StepAbstract(t0, h, xt0, g,
                      ComputeAndFactorImplicitEulerIterationMatrix,
                      xtplus_guess, &ie_iteration_matrix_, &*xtplus);
}

template <class T>
bool ImplicitEulerIntegrator<T>::StepHalfSizedImplicitEulers(
    const T& t0, const T& h, const VectorX<T>& xt0,
    const VectorX<T>& xtplus_ie, VectorX<T>* xtplus) {
  using std::abs;

  DRAKE_LOGGER_DEBUG("StepHalfSizedImplicitEulers(h={}) t={}", h, t0);


  // Store statistics before calling StepAbstract(). The difference between
  // the modified statistics and the stored statistics will be used to compute
  // the half-sized-implicit-Euler-specific statistics.
  const int stored_num_jacobian_evaluations =
      this->get_num_jacobian_evaluations();
  const int stored_num_iter_factorizations =
      this->get_num_iteration_matrix_factorizations();
  const int64_t stored_num_function_evaluations =
      this->get_num_derivative_evaluations();
  const int64_t stored_num_jacobian_function_evaluations =
      this->get_num_derivative_evaluations_for_jacobian();
  const int stored_num_nr_iterations =
      this->get_num_newton_raphson_iterations();

  // We set our guess for the state after a half-step to the average of the
  // guess for the final state, xtplus_ie, and the initial state, xt0.
  VectorX<T> xtmp = 0.5 * (xt0 + xtplus_ie);
  // Attempt to step.
  bool success = StepImplicitEulerWithGuess(t0, 0.5 * h, xt0, xtmp, xtplus);
  if (!success) {
    DRAKE_LOGGER_DEBUG("First Half IE convergence failed.");
  } else {
    // Swap the current output, xtplus, into xthalf, which functions as the new
    // xⁿ.
    std::swap(xtmp, *xtplus);
    const VectorX<T>& xthalf = xtmp;

    // Since the first half-step succeeded, either we recomputed a Jacobian at
    // (t0, x0), or we reused an older Jacobian. Therefore, as far as the next
    // half-sized step is concerned, the Jacobian is not at state
    // (t+h/2, x(t+h/2)). Since jacobian_is_fresh_ means that the Jacobian is
    // computed at the (t0,x0) of the beginning of the step we want to take, we
    // mark it as not-fresh.
    this->set_jacobian_is_fresh(false);

    // TODO(antequ): One possible optimization is, if the Jacobian is fresh at
    // this point, we can set a flag to cache the Jacobian if it gets
    // recomputed, so that if the second substep fails, we simply restore the
    // cached Jacobian instead of marking it stale. Since the second substep
    // very rarely fails if the large step and the first substep succeeded,
    // our tests indicates that this optimization saves only about 2% of the
    // effort (0-5% in most cases), on a stiff 3-body pile of objects example.
    // Therefore we omitted this optimization for code simplicity. See
    // Revision 1 of PR 13224 for an implementation of this optimization.

    success = StepImplicitEulerWithGuess(t0 + 0.5 * h, 0.5 * h, xthalf,
        xtplus_ie, xtplus);
    if (!success) {
      DRAKE_LOGGER_DEBUG("Second Half IE convergence failed.");
      // After a failure, the Jacobians were updated, so we have to mark that
      // the current Jacobian is not fresh by setting
      // failed_jacobian_is_from_second_small_step_ to true, so that at the
      // beginning of the next step, we know to set jacobian_is_fresh_ to
      // false, in DoImplicitIntegratorStep(). (Note that here we were slightly
      // abusing the jacobian_is_fresh_ flag --- for the second half-sized step,
      // we called MaybeFreshenMatrices() at t = t0 + h/2, meaning that
      // jacobian_is_fresh_ now marks whether the Jacobian was last computed at
      // t = t0 + h/2 instead of its usual definition of t = t0. This is why
      // when the step fails, ImplicitIntegrator<T>::DoStep() will incorrectly
      // mark the Jacobian as fresh, and we will need to fix this in
      // DoImplicitIntegratorStep() for the next step.)
      failed_jacobian_is_from_second_small_step_ = true;
    }
  }
  // Move statistics to half-sized-implicit-Euler-specific statistics.
  // Notice that we log the statistics even if either step fails.
  hie_statistics_.num_jacobian_reforms +=
      this->get_num_jacobian_evaluations() - stored_num_jacobian_evaluations;
  hie_statistics_.num_iter_factorizations +=
      this->get_num_iteration_matrix_factorizations() -
      stored_num_iter_factorizations;
  hie_statistics_.num_function_evaluations +=
      this->get_num_derivative_evaluations() - stored_num_function_evaluations;
  hie_statistics_.num_jacobian_function_evaluations +=
      this->get_num_derivative_evaluations_for_jacobian() -
      stored_num_jacobian_function_evaluations;
  hie_statistics_.num_nr_iterations +=
      this->get_num_newton_raphson_iterations() - stored_num_nr_iterations;

  return success;
}

template <class T>
bool ImplicitEulerIntegrator<T>::StepImplicitTrapezoid(
    const T& t0, const T& h, const VectorX<T>& xt0, const VectorX<T>& dx0,
    const VectorX<T>& xtplus_ie, VectorX<T>* xtplus) {
  using std::abs;

  DRAKE_LOGGER_DEBUG("StepImplicitTrapezoid(h={}) t={}", h, t0);

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
  const int stored_num_jacobian_evaluations =
      this->get_num_jacobian_evaluations();
  const int stored_num_iter_factorizations =
      this->get_num_iteration_matrix_factorizations();
  const int64_t stored_num_function_evaluations =
      this->get_num_derivative_evaluations();
  const int64_t stored_num_jacobian_function_evaluations =
      this->get_num_derivative_evaluations_for_jacobian();
  const int stored_num_nr_iterations =
      this->get_num_newton_raphson_iterations();

  // Attempt to step.
  bool success = StepAbstract(t0, h, xt0, g,
                              ComputeAndFactorImplicitTrapezoidIterationMatrix,
                              xtplus_ie, &itr_iteration_matrix_, xtplus);

  // Move statistics to implicit trapezoid-specific.
  // Notice that we log the statistics even if the step fails.
  itr_statistics_.num_jacobian_reforms +=
      this->get_num_jacobian_evaluations() - stored_num_jacobian_evaluations;
  itr_statistics_.num_iter_factorizations +=
      this->get_num_iteration_matrix_factorizations() -
      stored_num_iter_factorizations;
  itr_statistics_.num_function_evaluations +=
      this->get_num_derivative_evaluations() - stored_num_function_evaluations;
  itr_statistics_.num_jacobian_function_evaluations +=
      this->get_num_derivative_evaluations_for_jacobian() -
      stored_num_jacobian_function_evaluations;
  itr_statistics_.num_nr_iterations +=
      this->get_num_newton_raphson_iterations() - stored_num_nr_iterations;

  return success;
}

template <class T>
bool ImplicitEulerIntegrator<T>::AttemptStepPaired(const T& t0, const T& h,
    const VectorX<T>& xt0, VectorX<T>* xtplus_ie, VectorX<T>* xtplus_hie) {
  using std::abs;
  DRAKE_ASSERT(xtplus_ie != nullptr);
  DRAKE_ASSERT(xtplus_hie != nullptr);

  // Compute the derivative at time and state (t0, x(t0)). NOTE: the derivative
  // is calculated at this point (early on in the integration process) in order
  // to reuse the derivative evaluation, via the cache, from the last
  // integration step (if possible).
  const VectorX<T> dx0 = this->EvalTimeDerivatives(
      this->get_context()).CopyToVector();

  // Do the Euler step.
  if (!StepImplicitEuler(t0, h, xt0, xtplus_ie)) {
    DRAKE_LOGGER_DEBUG("Implicit Euler approach did not converge for "
        "step size {}", h);
    return false;
  }

  if (!use_implicit_trapezoid_error_estimation_) {
    // In this case, step two half-sized implicit Euler steps along
    // with the full step for error estimation.
    if (StepHalfSizedImplicitEulers(t0, h, xt0, *xtplus_ie, xtplus_hie)) {
      Context<T>* context = this->get_mutable_context();
      context->SetTimeAndContinuousState(t0 + h, *xtplus_hie);
      return true;
    } else {
      DRAKE_LOGGER_DEBUG("Implicit Euler half-step approach FAILED with a step"
          "size that succeeded on full-sized implicit Euler.");
      return false;
    }
  } else {
    // In this case, use the implicit trapezoid method, which is defined as:
    // x(t0+h) = x(t0) + h/2 (f(t0, x(t0) + f(t0+h, x(t0+h))
    // x(t0+h) from the implicit Euler method is presumably a good starting
    // point.

    // The error estimate is derived as follows (thanks to Michael Sherman):
    // x*(t0+h) = xᵢₑ(t0+h) + O(h²)      [implicit Euler]
    //          = xₜᵣ(t0+h) + O(h³)      [implicit trapezoid]
    // where x*(t0+h) is the true (generally unknown) answer that we seek.
    // This implies:
    // xᵢₑ(t0+h) + O(h²) = xₜᵣ(t0+h) + O(h³).
    // Given that the second order term subsumes the third order one, we have:
    // xᵢₑ(t0+h) - xₜᵣ(t0+h) = O(h²).

    // Attempt to compute the implicit trapezoid solution.
    if (StepImplicitTrapezoid(t0, h, xt0, dx0, *xtplus_ie, xtplus_hie)) {
      // Reset the state to that computed by implicit Euler.
      // TODO(edrumwri): Explore using the implicit trapezoid method solution
      //                 instead as *the* solution, rather than the implicit
      //                 Euler. Refer to [Lambert, 1991], Ch 6.
      Context<T>* context = this->get_mutable_context();
      context->SetTimeAndContinuousState(t0 + h, *xtplus_ie);
      return true;
    } else {
      DRAKE_LOGGER_DEBUG("Implicit trapezoid approach FAILED with a step"
          "size that succeeded on implicit Euler.");
      return false;
    }
  }
}

template <class T>
bool ImplicitEulerIntegrator<T>::DoImplicitIntegratorStep(const T& h) {
  // Save the current time and state.
  Context<T>* context = this->get_mutable_context();
  const T t0 = context->get_time();
  DRAKE_LOGGER_DEBUG("IE DoStep(h={}) t={}", h, t0);

  xt0_ = context->get_continuous_state().CopyToVector();
  xtplus_ie_.resize(xt0_.size());
  xtplus_hie_.resize(xt0_.size());

  // If the most recent step failed only after the second small step, this
  // indicates that the Jacobian was computed from the second small step, and
  // not at (t0, xt0). ImplicitIntegrator<T>::DoStep() would have incorrectly
  // marked the Jacobian as fresh, and so we must correct it by marking
  // jacobian_is_fresh_ as false. (Note that this occurs because we were
  // slightly abusing the jacobian_is_fresh_ flag --- for the second half-sized
  // step, we called MaybeFreshenMatrices() at t = t0 + h/2, meaning that
  // jacobian_is_fresh_ marked whether the Jacobian was last computed at
  // t = t0 + h/2 instead of its usual definition of t = t0. This is why when
  // the step failed, ImplicitIntegrator<T>::DoStep() would have incorrectly
  // marked the Jacobian as fresh.)
  if (failed_jacobian_is_from_second_small_step_) {
    this->set_jacobian_is_fresh(false);

    // Now that we've correctly set the jacobian_is_fresh_ flag, make sure that
    // the failed_jacobian_is_from_second_small_step_ is reset back to false.
    failed_jacobian_is_from_second_small_step_ = false;
  }

  // If the requested h is less than the minimum step size, we'll advance time
  // using an explicit Euler step.
  if (h < this->get_working_minimum_step_size()) {
    DRAKE_LOGGER_DEBUG("-- requested step too small, taking explicit "
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

    // Compute the RK2 or two half-sized Euler steps.
    if (use_implicit_trapezoid_error_estimation_) {
      // Compute the RK2 step
      const int evals_before_rk2 = rk2_->get_num_derivative_evaluations();
      if (!rk2_->IntegrateWithSingleFixedStepToTime(t0 + h)) {
        throw std::runtime_error("Embedded RK2 integrator failed to take a"
            "single fixed step to the requested time.");
      }

      const int evals_after_rk2 = rk2_->get_num_derivative_evaluations();
      xtplus_hie_ = context->get_continuous_state().CopyToVector();

      // Update the implicit Trapezoid ODE counts.
      itr_statistics_.num_function_evaluations +=
          (evals_after_rk2 - evals_before_rk2);

      // Revert the state to that computed by explicit Euler.
      context->SetTimeAndContinuousState(t0 + h, xtplus_ie_);
    } else {
      // complete two half-sized explicit Euler steps.
      xtplus_hie_ = xt0_ + 0.5 * h * xdot_;
      context->SetTimeAndContinuousState(t0 + 0.5 * h, xtplus_hie_);
      xtplus_hie_ +=
          0.5 * h * this->EvalTimeDerivatives(*context).CopyToVector();

      // Update the half-sized step ODE counts.
      ++(hie_statistics_.num_function_evaluations);

      context->SetTimeAndContinuousState(t0 + h, xtplus_hie_);
    }

  } else {
    // Try taking the requested step.
    bool success = AttemptStepPaired(t0, h, xt0_, &xtplus_ie_, &xtplus_hie_);

    // If the step was not successful, reset the time and state.
    if (!success) {
      context->SetTimeAndContinuousState(t0, xt0_);
      return false;
    }
  }

  // Compute and update the error estimate.
  err_est_vec_ = xtplus_ie_ - xtplus_hie_;

  // Update the caller-accessible error estimate.
  this->get_mutable_error_estimate()->get_mutable_vector().
      SetFromVector(err_est_vec_);

  return true;
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::systems::ImplicitEulerIntegrator)
