#pragma once

/// @file
/// Template method implementations for
///    second_order_implicit_euler_integrator.h.
/// Most users should only include that file, not this one.
/// For background, see https://drake.mit.edu/cxx_inl.html.

/* clang-format off to disable clang-format-includes */
#include "drake/systems/analysis/second_order_implicit_euler_integrator.h"
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
void SecondOrderImplicitEulerIntegrator<T>::
         DoResetImplicitIntegratorStatistics() {
  num_nr_iterations_ = 0;
  num_err_est_nr_iterations_ = 0;
  num_err_est_function_evaluations_ = 0;
  num_err_est_jacobian_function_evaluations_ = 0;
  num_err_est_jacobian_reforms_ = 0;
  num_err_est_iter_factorizations_ = 0;
}

template <class T>
void SecondOrderImplicitEulerIntegrator<T>::DoInitialize() {
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
  this->get_mutable_velocity_jacobian().resize(0, 0);

  // Initialize the embedded second order Runge-Kutta integrator. The maximum
  // step size will be set to infinity because we will explicitly request the
  // step sizes to be taken.
  rk2_ = std::make_unique<RungeKutta2Integrator<T>>(
      this->get_system(),
      std::numeric_limits<double>::infinity() /* maximum step size */,
      this->get_mutable_context());
}

template <class T>
void SecondOrderImplicitEulerIntegrator<T>::
         ComputeAndFactorImplicitEulerIterationMatrix(
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
void SecondOrderImplicitEulerIntegrator<T>::
         ComputeAndFactorImplicitTrapezoidIterationMatrix(
    const MatrixX<T>& J, const T& h,
    typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix) {
      throw std::logic_error("Should not be factorizing IT Matrix!");
}


// COPIED FROM IMPLICIT INTEGRATOR H
// Computes the Jacobian of the ordinary differential equations around time
// and continuous state `(t, xt)` using a first-order forward difference (i.e.,
// numerical differentiation).
// @param system The dynamical system.
// @param t the time around which to compute the Jacobian matrix.
// @param xt the continuous state around which to compute the Jacobian matrix.
// @param context the Context of the system, at time and continuous state
//        unknown.
// @param [out] the Jacobian matrix around time and state `(t, xt)`.
// @note The continuous state will be indeterminate on return.
template <class T>
void SecondOrderImplicitEulerIntegrator<T>::ComputeForwardDiffVelocityJacobian(
    const T& t, const VectorX<T>& xt, Context<T>* context,
    MatrixX<T>* J) {
  using std::abs;

  // Set epsilon to the square root of machine precision.
  const double eps = std::sqrt(std::numeric_limits<double>::epsilon());

  // Get the number of continuous state variables xt.
  const int n = context->num_continuous_states();

  SPDLOG_DEBUG(drake::log(), "  ImplicitIntegrator Compute Forwarddiff "
               "{}-Jacobian t={}", n, t);
  SPDLOG_DEBUG(drake::log(), "  computing from state {}", xt.transpose());

  // Initialize the Jacobian.
  J->resize(n, n);

  // Evaluate f(t,xt).
  context->SetTimeAndContinuousState(t, xt);
  const VectorX<T> f = this->EvalTimeDerivatives(*context).CopyToVector();

  // Compute the Jacobian.
  VectorX<T> xt_prime = xt;
  for (int i = 0; i < n; ++i) {
    // Compute a good increment to the dimension using approximately 1/eps
    // digits of precision. Note that if |xt| is large, the increment will
    // be large as well. If |xt| is small, the increment will be no smaller
    // than eps.
    const T abs_xi = abs(xt(i));
    T dxi(abs_xi);
    if (dxi <= 1) {
      // When |xt[i]| is small, increment will be eps.
      dxi = eps;
    } else {
      // |xt[i]| not small; make increment a fraction of |xt[i]|.
      dxi = eps * abs_xi;
    }

    // Update xt', minimizing the effect of roundoff error by ensuring that
    // x and dx differ by an exactly representable number. See p. 192 of
    // Press, W., Teukolsky, S., Vetterling, W., and Flannery, P. Numerical
    //   Recipes in C++, 2nd Ed., Cambridge University Press, 2002.
    xt_prime(i) = xt(i) + dxi;
    dxi = xt_prime(i) - xt(i);

    // TODO(sherm1) This is invalidating q, v, and z but we only changed one.
    //              Switch to a method that invalides just the relevant
    //              partition, and ideally modify only the one changed element.
    // Compute f' and set the relevant column of the Jacobian matrix.
    context->SetTimeAndContinuousState(t, xt_prime);
    J->col(i) = (this->EvalTimeDerivatives(*context).CopyToVector() - f) / dxi;

    // Reset xt' to xt.
    xt_prime(i) = xt(i);
  }
}


// COPIED FROM IMPLICIT INTEGRATOR H
// Compute the partial derivative of the ordinary differential equations with
// respect to the state variables for a given x(t).
// @post the context's time and continuous state will be temporarily set during
//       this call (and then reset to their original values) on return.
template <class T>
const MatrixX<T>& SecondOrderImplicitEulerIntegrator<T>::CalcVelocityJacobian(
    const T& t, const VectorX<T>& x) {
  // We change the context but will change it back.
  Context<T>* context = this->get_mutable_context();

  // Get the current time and state.
  const T t_current = context->get_time();
  const ContinuousState<T>& cstate = context->get_continuous_state();
  const VectorX<T> x_current = cstate.CopyToVector();
  //const VectorX<T>& y_current = x_current.block(cstate.num_q(),0,cstate.num_v() + cstate.num_z(),1);

  // Update the time and state.
  context->SetTimeAndContinuousState(t, x);
  num_jacobian_evaluations_++;

  // Get the current number of ODE evaluations.
  int64_t current_ODE_evals = this->get_num_derivative_evaluations();

  // Get a the system.
  const System<T>& system = this->get_system();

  // TODO(edrumwri): Give the caller the option to provide their own Jacobian.
  [this, context, &system, &t, &x]() {
    switch (jacobian_scheme_) {
      case JacobianComputationScheme::kForwardDifference:
        ComputeForwardDiffVelocityJacobian(system, t, x, &*context, &Jv_);
        break;

      default:
        throw new std::logic_error("Non forward diffs are not supported yet!");
    }
  }();

  // Use the new number of ODE evaluations to determine the number of Jacobian
  // evaluations.
  num_jacobian_function_evaluations_ += this->get_num_derivative_evaluations()
      - current_ODE_evals;

  // Reset the time and state.
  context->SetTimeAndContinuousState(t_current, x_current);

  return Jv_;
}


// COPIED FROM IMPLICIT INTEGRATOR H
template <class T>
bool SecondOrderImplicitEulerIntegrator<T>::MaybeFreshenMatrices(
    const T& t, const VectorX<T>& xt, const T& h, int trial,
    const std::function<void(const MatrixX<T>&, const T&,
        typename ImplicitIntegrator<T>::IterationMatrix*)>&
        compute_and_factor_iteration_matrix,
    typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix) {
  // Compute the initial Jacobian and iteration matrices and factor them, if
  // necessary.
  MatrixX<T>& J = get_mutable_velocity_jacobian();
  if (!get_reuse() || J.rows() == 0 || IsBadJacobian(J)) {
    J = CalcVelocityJacobian(t, xt);
    ++num_iter_factorizations_;
    compute_and_factor_iteration_matrix(J, h, iteration_matrix);
    return true;  // Indicate success.
  }

  // Reuse is activated, Jacobian is fully sized, and Jacobian is not "bad".
  // If the iteration matrix has not been set and factored, do only that.
  if (!iteration_matrix->matrix_factored()) {
    ++num_iter_factorizations_;
    compute_and_factor_iteration_matrix(J, h, iteration_matrix);
    return true;  // Indicate success.
  }

  switch (trial) {
    case 1:
      // For the first trial, we do nothing: this will cause the Newton-Raphson
      // process to use the last computed (and already factored) iteration
      // matrix.
      return true;  // Indicate success.

    case 2: {
      // For the second trial, we perform the (likely) next least expensive
      // operation, re-constructing and factoring the iteration matrix.
      ++num_iter_factorizations_;
      compute_and_factor_iteration_matrix(J, h, iteration_matrix);
      return true;
    }

    case 3: {
      // For the third trial, the Jacobian matrix may already be "fresh",
      // meaning that there is nothing more that can be tried (Jacobian and
      // iteration matrix are both fresh) and we need to indicate failure.
      if (jacobian_is_fresh_)
        return false;

      // Reform the Jacobian matrix and refactor the iteration matrix.
      J = CalcVelocityJacobian(t, xt);
      ++num_iter_factorizations_;
      compute_and_factor_iteration_matrix(J, h, iteration_matrix);
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
bool SecondOrderImplicitEulerIntegrator<T>::StepAbstract(const T& t0,
    const T& h, const VectorX<T>& xt0, const std::function<VectorX<T>()>& g,
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

  // Try StepAbstract again, first resetting xtplus to xt0. That method will
  // freshen Jacobians and iteration matrix factorizations as necessary.
  *xtplus = xt0;
  return StepAbstract(
      t0, h, xt0, g, compute_and_factor_iteration_matrix, xtplus, trial+1);
}

// evaluates g(y) with y from the context. Context should be at time tf
template <class T>
void SecondOrderImplicitEulerIntegrator<T>::eval_g_with_y_from_context(
    const VectorX<T>& qt0, const T& h,
    const VectorX<T>& qk, VectorX<T>* result) {
  Context<T>* context = this->get_mutable_context();
  const systems::ContinuousState<T>& cstate = context->get_continuous_state();
  context->get_mutable_continous_state().get_mutable_generalized_position().SetFromVector(qk);
  BasicVector<T> qdot (qt0.rows());
  this->get_system().MapVelocityToQDot(*context, cstate.get_generalized_velocity(), &qdot);
  VectorX<T> q = qt0 + h * qdot.CopyToVector();
  context->get_mutable_continous_state().get_mutable_generalized_position().SetFromVector(q);
  const ContinuousState<T>& xc_deriv = this->EvalTimeDerivatives(*context);
  //VectorX<T> to_return (yt0.rows()); // using result instead of to_return
  xc_deriv.get_generalized_velocity().CopyToPreSizedVector(result->block(0,0,xc_deriv.num_v(),1));
  xc_deriv.get_misc_continuous_state().CopyToPreSizedVector(result->block(xc_deriv.num_v(),0,xc_deriv.num_z(),1));
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
bool SecondOrderImplicitEulerIntegrator<T>::StepImplicitEuler(const T& t0,
    const T& h, const VectorX<T>& xt0, VectorX<T>* xtplus) {
  using std::abs;

  SPDLOG_DEBUG(drake::log(), "StepImplicitEuler(h={}) t={}", h, t0);

  const System<T>& system = this->get_system();

  // Use the current state as the candidate value for the next state.
  // [Hairer 1996] validates this choice (p. 120).
  *xtplus = xt0;
  // set a residual evaluator
  Context<T>* context = this->get_mutable_context();
  const systems::ContinuousState<T>& cstate = context->get_continuous_state();
        const VectorX<T>& qt0 = xt0.block(0,0,cstate.num_q(),1);
        const VectorX<T>& yt0 = xt0.block(cstate.num_q(),0,cstate.num_v() + cstate.num_z(),1);
        const VectorX<T>& qk = xtplus->block(0,0,cstate.num_q(),1);
  std::function<VectorX<T>()> velocity_residual_R =
      [&qt0, &yt0, h, context, &qk, this]() {
        const systems::ContinuousState<T>& cstate = context->get_continuous_state();
        VectorX<T> y (cstate.num_v() + cstate.num_z());
        VectorX<T> g_of_y (cstate.num_v() + cstate.num_z());
        cstate.get_generalized_velocity().CopyToPreSizedVector(y.block(0,0,cstate.num_v(),1));
        cstate.get_misc_continuous_state().CopyToPreSizedVector(y.block(cstate.num_v(),0,cstate.num_z(),1));
        this->eval_g_with_y_from_context(qt0, h, qk, &g_of_y);
        return (y - yt0 - h * g_of_y);
      };


  // Verify xtplus
  DRAKE_ASSERT(xtplus && xtplus->size() == xt0.size());
 
  SPDLOG_DEBUG(drake::log(), "StepAbstract() entered for t={}, h={}, trial={}",
      t0, h, trial);
  VectorX<T>& qt = xtplus->block(0,0,cstate.num_q(),1);
  
  VectorX<T>& yt = xtplus->block(cstate.num_q(),0,cstate.num_v() + cstate.num_z(),1);
  BasicVector<T> qdot (qt0.rows());
  VectorX<T> last_qt ( qt );
  VectorX<T> dx (xtplus->rows());

  // Advance the context time and state to compute derivatives at t0 + h.
  const T tf = t0 + h;
  Context<T>* context = this->get_mutable_context();
  context->SetTimeAndContinuousState(tf, *xtplus);
  // Evaluate the residual error for equation (11)
  VectorX<T> residual = velocity_residual_R();

  // Initialize the "last" state update norm; this will be used to detect
  // convergence.
  T last_dx_norm = std::numeric_limits<double>::infinity();
  // Full Newton: Calculate Jacobian and iteration matrices (and factorizations), as needed,
    // around (tf, xtplus).
  if (!this->MaybeFreshenMatrices(tf, *xtplus, h, 3,
      ComputeAndFactorImplicitEulerIterationMatrix, &iteration_matrix_)) {
    return false;
  }


  // Do the Newton-Raphson iterations.
  for (int i = 0; i < this->max_newton_raphson_iterations(); ++i) {
    // Update the number of Newton-Raphson iterations.
    num_nr_iterations_++;


    // Compute the state update using the equation A*x = -g(), where A is the
    // iteration matrix.
    // TODO(edrumwri): Allow caller to provide their own solver.
    VectorX<T> dy = iteration_matrix_.Solve(-residual);
   if (i > 0 && this->IsUpdateZero(yt, dy))
      return true;
    yt += dy;

    // assume context uses qtk
    this->get_system().MapVelocityToQDot(*context, yt.block(0,0,cstate.num_v(),1), &qdot);
    qt = qt0 + h * qdot.CopyToVector();
    dx << dy, qt - last_qt;
    last_qt = qt;

    
    // Get the infinity norm of the weighted update vector.
    dx_state_->get_mutable_vector().SetFromVector(dx);
    T dx_norm = this->CalcStateChangeNorm(*dx_state_);
    // update the context
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
    last_dx_norm = dx_norm;
    residual = velocity_residual_R();


  }

  SPDLOG_DEBUG(drake::log(), "SOIE convergence failed");

  // If Jacobian and iteration matrix factorizations are not reused, there
  // is nothing else we can try.
  if (!this->get_reuse())
    return false;

  // Try StepAbstract again, first resetting xtplus to xt0. That method will
  // freshen Jacobians and iteration matrix factorizations as necessary.
  *xtplus = xt0;
  return false;
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
bool SecondOrderImplicitEulerIntegrator<T>::StepImplicitTrapezoid(const T& t0,
    const T& h, const VectorX<T>& xt0, const VectorX<T>& dx0,
    VectorX<T>* xtplus) {
      throw std::logic_error("Should not be running Implicit Trapezoid!");
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
bool SecondOrderImplicitEulerIntegrator<T>::AttemptStepPaired(const T& t0,
    const T& h, const VectorX<T>& xt0, VectorX<T>* xtplus_ie,
    VectorX<T>* xtplus_itr) {
  using std::abs;
  DRAKE_ASSERT(xtplus_ie);
  //DRAKE_ASSERT(xtplus_itr);


  // Do the Euler step.
  if (!StepImplicitEuler(t0, h, xt0, xtplus_ie)) {
    SPDLOG_DEBUG(drake::log(), "Implicit Euler approach did not converge for "
        "step size {}", h);
    return false;
  }
  Context<T>* context = this->get_mutable_context();
    context->SetTimeAndContinuousState(t0 + h, *xtplus_ie);
    
  return true;

}

/// Takes a given step of the requested size, if possible.
/// @returns `true` if successful and `false` otherwise; on `true`, the time
///          and continuous state will be advanced in the context (e.g., from
///          t0 to t0 + h). On `false` return, the time and continuous state in
///          the context will be restored to its original value (at t0).
template <class T>
bool SecondOrderImplicitEulerIntegrator<T>::
         DoImplicitIntegratorStep(const T& h) {
  // Save the current time and state.
  Context<T>* context = this->get_mutable_context();
  const T t0 = context->get_time();
  SPDLOG_DEBUG(drake::log(), "IE DoStep(h={}) t={}", h, t0);

  xt0_ = context->get_continuous_state().CopyToVector();
  xtplus_ie_.resize(xt0_.size());
  xtplus_tr_.resize(0);

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
