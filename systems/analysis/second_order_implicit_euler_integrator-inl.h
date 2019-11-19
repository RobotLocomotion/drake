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

// #define FULL_NEWTON
namespace drake {
namespace systems {

template <class T>
void SecondOrderImplicitEulerIntegrator<
    T>::DoResetImplicitIntegratorStatistics() {
  num_nr_iterations_ = 0;
  num_nr_iterations_that_end_in_failure_ = 0;
  num_nr_failures_ = 0;
  num_err_est_nr_iterations_ = 0;
  num_err_est_function_evaluations_ = 0;
  num_err_est_jacobian_function_evaluations_ = 0;
  num_err_est_jacobian_reforms_ = 0;
  num_err_est_iter_factorizations_ = 0;
  num_err_est_nr_iterations_that_end_in_failure_ = 0;
  num_err_est_nr_failures_ = 0;
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
      throw std::logic_error(
          "Neither initial step size target nor maximum "
          "step size has been set!");

    this->request_initial_step_size_target(this->get_maximum_step_size());
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
  this->get_mutable_velocity_jacobian_implicit_euler().resize(0, 0);
  this->get_mutable_velocity_jacobian_half_implicit_euler().resize(0, 0);

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
  iteration_matrix->SetAndFactorIterationMatrix(J * -h +
                                                MatrixX<T>::Identity(n, n));
}

// Computes the Jacobian, Jₗₖ(y), of the function lₖ(y), used in this
// integrator's residual computation, with respect to y. As defined before, y =
// (v, z). This Jacobian is then defined as:
//     lₖ(y)  = f(tⁿ⁺¹, qⁿ + h N(qₖ) v, y)    (9)
//     Jₗₖ(y) = ∂lₖ(y)/∂y                     (10)
//
// In this method, we compute the Jacobian Jₗₖ(y) using a first-order forward
// difference (i.e. numerical differentiation),
//   Jₗₖ(y)ᵢⱼ = (lₖ(y')ᵢ - lₖ(y)ᵢ )/ δy(j),
//      where y' = y + δy(j) eⱼ and δy(j) = (√ε) max(1,|yⱼ|).
// In the code we hereby refer to y as "the baseline" and y' as "prime".
// @param t refers to tⁿ⁺¹, the time used in the definition of lₖ(y)
// @param h is the timestep size parameter, h, used in the definition of lₖ(y)
// @param x is (qₖ, y), the continuous state around which to evaluate Jₗₖ(y)
// @param qt0 refers to qⁿ, the initial position used in lₖ(y)
// @param context the Context of the system, at time and continuous state
//        unknown.
// @param [out] the Jacobian matrix, Jₗₖ(y).
// @note The continuous state will be indeterminate on return.
// For full Newton, we recommend this method to be evaluated at time t = t0 + h
// and x = xₖ; however, this recommendation is not necessary for the
// integrator and more optimized versions will modify this.
template <class T>
void SecondOrderImplicitEulerIntegrator<T>::ComputeForwardDiffVelocityJacobian(
    const T& t, const T& h, const VectorX<T>& x, const VectorX<T>& qt0,
    Context<T>* context, MatrixX<T>* Jv) {
  using std::abs;
  using std::max;

  // Set the finite difference tolerance, used to compute δy in the notation
  // above, to √ε, the square root of machine precision.
  const double sqrt_eps = std::sqrt(std::numeric_limits<double>::epsilon());

  // Get the number of q, v, y state variables in x.
  const ContinuousState<T>& cstate = context->get_continuous_state();
  const int nq = cstate.num_q();
  const int nv = cstate.num_v();
  const int ny = nv + cstate.num_z();

  DRAKE_LOGGER_DEBUG(
      "  ImplicitIntegrator Compute Forwarddiff "
      "{}-Jacobian t={}",
      ny, t);
  DRAKE_LOGGER_DEBUG("  computing from state {}", x.transpose());

  // Initialize the Jacobian.
  Jv->resize(ny, ny);

  // qk, v, y refer to the baseline state from x.
  const VectorX<T>& qk = x.head(nq);
  const VectorX<T>& v = x.segment(nq, nv);
  const VectorX<T>& y = x.tail(ny);

  // Evaluate qt0_plus_hNv = qⁿ + h N(qₖ) v with (qₖ,v) from x
  VectorX<T> qt0_plus_hNv(nq);
  BasicVector<T> qdot(nq);
  context->SetTimeAndContinuousState(t, x);
  this->get_system().MapVelocityToQDot(*context, v, &qdot);
  qt0_plus_hNv = qt0 + h * qdot.get_value();

  // Compute the Jacobian.

  // Define x' = (qⁿ + h N(qₖ) v, y), to compute the baseline l(y), and then
  // reuse x' = (qⁿ + h N(qₖ) v', y') to compute each prime l(y')
  VectorX<T> x_prime = x;
  Eigen::Ref<VectorX<T>> q_prime = x_prime.head(nq);
  Eigen::Ref<VectorX<T>> v_prime = x_prime.segment(nq, nv);
  Eigen::Ref<VectorX<T>> y_prime = x_prime.tail(ny);
  q_prime = qt0_plus_hNv;

  // Initialize the finite-difference baseline, l(y), by
  //  evaluating the context at l(y) = f(t, qⁿ + h N(qₖ) v, y) = f(t,x).
  context->get_mutable_continuous_state()
      .get_mutable_generalized_position()
      .SetFromVector(q_prime);
  const VectorX<T> l_of_y = this->EvalTimeDerivatives(*context).CopyToVector();

  // Now evaluate each l(y') where y' modifies one value of y at a time
  for (int j = 0; j < ny; ++j) {
    // Compute a good increment, δy(j), to the dimension using approximately
    // log(1/√ε) digits of precision. Note that if |yⱼ| is large, the increment
    // will be large as well. If |yⱼ| is small, the increment will be no smaller
    // than √ε.
    const T abs_yj = abs(y(j));
    T dyj = sqrt_eps * max(T(1), abs_yj);

    // Update y', minimizing the effect of roundoff error by ensuring that
    // y and y' differ by an exactly representable number. See p. 192 of
    // Press, W., Teukolsky, S., Vetterling, W., and Flannery, P. Numerical
    //   Recipes in C++, 2nd Ed., Cambridge University Press, 2002.
    y_prime(j) = y(j) + dyj;
    if (j < nv) {
      // Set q' = qⁿ + h N(qₖ) v' with v' from y'
      context->get_mutable_continuous_state()
          .get_mutable_generalized_position()
          .SetFromVector(qk);
      this->get_system().MapVelocityToQDot(*context, v_prime, &qdot);
      q_prime = qt0 + h * qdot.get_value();
    } else {
      // In this scenario, v' = v, and so q' is unchanged.
      q_prime = qt0_plus_hNv;
    }
    dyj = y_prime(j) - y(j);

    // TODO(sherm1) This is invalidating q, v, and z but we only changed a
    //              subset. Switch to a method that invalides just the relevant
    //              partition, and ideally modify only the changed elements.
    // Compute l(y') and set the relevant column of the Jacobian matrix.
    context->SetTimeAndContinuousState(t, x_prime);
    Jv->col(j) =
        (this->EvalTimeDerivatives(*context).CopyToVector() - l_of_y).tail(ny) /
        dyj;

    // Reset y' to y.
    y_prime(j) = y(j);
  }
}

// Compute the partial derivative of the ordinary differential equations with
// respect to the state variables for a given x(t).
// @post the context's time and continuous state will be temporarily set during
//       this call (and then reset to their original values) on return.
template <class T>
void SecondOrderImplicitEulerIntegrator<T>::CalcVelocityJacobian(
    const T& t, const T& h, const VectorX<T>& x, const VectorX<T>& qt0,
    MatrixX<T>* Jv) {
  // We change the context but will change it back.
  Context<T>* context = this->get_mutable_context();

  // Get the current time and state.
  const T t_current = context->get_time();
  const ContinuousState<T>& cstate = context->get_continuous_state();
  const VectorX<T> x_current = cstate.CopyToVector();
  // const VectorX<T>& y_current =
  // x_current.block(cstate.num_q(),0,cstate.num_v()
  //   + cstate.num_z(),1);

  // Update the time and state.
  context->SetTimeAndContinuousState(t, x);
  this->increment_jacobian_evaluations();

  // Get the current number of ODE evaluations.
  int64_t current_ODE_evals = this->get_num_derivative_evaluations();

  //// Get the system.
  // const System<T>& system = this->get_system();
  // forward diff the Jacobian
  switch (this->get_jacobian_computation_scheme()) {
    case SecondOrderImplicitEulerIntegrator<
        T>::JacobianComputationScheme::kForwardDifference:
      ComputeForwardDiffVelocityJacobian(t, h, x, qt0, &*context, Jv);
      break;
    case SecondOrderImplicitEulerIntegrator<
        T>::JacobianComputationScheme::kCentralDifference:
      throw std::runtime_error("Central difference not supported yet!");
      break;
    case SecondOrderImplicitEulerIntegrator<
        T>::JacobianComputationScheme::kAutomatic:
      throw std::runtime_error("AutoDiff'd Jacobian not supported yet!");
      break;
    default:
      throw new std::logic_error("Invalid Jacobian computation scheme!");
  }

  // Use the new number of ODE evaluations to determine the number of Jacobian
  // evaluations.
  this->increment_jacobian_function_evaluations(
      this->get_num_derivative_evaluations() - current_ODE_evals);

  // Reset the time and state.
  context->SetTimeAndContinuousState(t_current, x_current);
}

// COPIED FROM IMPLICIT INTEGRATOR H
template <class T>
bool SecondOrderImplicitEulerIntegrator<T>::MaybeFreshenVelocityMatrices(
    const T& t, const VectorX<T>& xt, const VectorX<T>& qt0, const T& h,
    int trial,
    const std::function<void(const MatrixX<T>&, const T&,
                             typename ImplicitIntegrator<T>::IterationMatrix*)>&
        compute_and_factor_iteration_matrix,
    typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix,
    MatrixX<T>* Jv) {
  DRAKE_DEMAND(Jv != nullptr);
  // Compute the initial Jacobian and iteration matrices and factor them, if
  // necessary.

  if (!this->get_reuse() || Jv->rows() == 0 || this->IsBadJacobian(*Jv)) {
    CalcVelocityJacobian(t, h, xt, qt0, Jv);
    this->increment_num_iter_factorizations();
    compute_and_factor_iteration_matrix(*Jv, h, iteration_matrix);
    return true;  // Indicate success.
  }

  // Reuse is activated, Jacobian is fully sized, and Jacobian is not "bad".
  // If the iteration matrix has not been set and factored, do only that.
  if (!iteration_matrix->matrix_factored()) {
    this->increment_num_iter_factorizations();
    compute_and_factor_iteration_matrix(*Jv, h, iteration_matrix);
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
      this->increment_num_iter_factorizations();
      compute_and_factor_iteration_matrix(*Jv, h, iteration_matrix);
      return true;
    }

    case 3: {
      // For the third trial, the Jacobian matrix may already be "fresh",
      // meaning that there is nothing more that can be tried (Jacobian and
      // iteration matrix are both fresh) and we need to indicate failure.
      // JACOBIAN IS NEVER FRESH, since it depends on h.
      // if (IsJacobianFresh())
      // return false;

      // Reform the Jacobian matrix and refactor the iteration matrix.
      CalcVelocityJacobian(t, h, xt, qt0, Jv);
      this->increment_num_iter_factorizations();
      compute_and_factor_iteration_matrix(*Jv, h, iteration_matrix);
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
//                calls MaybeFreshenVelocityMatrices()) in a unit test).
template <class T>
bool SecondOrderImplicitEulerIntegrator<T>::StepAbstract(
    const T&, const T&, const VectorX<T>&, const std::function<VectorX<T>()>&,
    const std::function<
        void(const MatrixX<T>&, const T&,
             typename ImplicitIntegrator<T>::IterationMatrix*)>&,
    VectorX<T>*, int) {
  throw std::logic_error("Should not be running StepAbstract!");
  return true;
}

// evaluates g(y) with y from the context. Context should be at time tf
template <class T>
void SecondOrderImplicitEulerIntegrator<T>::eval_g_with_y_from_context(
    const VectorX<T>& qt0, const T& h, const VectorX<T>& qk,
    VectorX<T>* result) {
  Context<T>* context = this->get_mutable_context();
  const systems::ContinuousState<T>& cstate = context->get_continuous_state();
  // compute q = qt0 + h N(qk) vt
  context->get_mutable_continuous_state()
      .get_mutable_generalized_position()
      .SetFromVector(qk);
  BasicVector<T> qdot(qt0.rows());
  this->get_system().MapVelocityToQDot(
      *context, cstate.get_generalized_velocity(), &qdot);
  VectorX<T> q = qt0 + h * qdot.get_value();

  // evaluate g from q
  context->get_mutable_continuous_state()
      .get_mutable_generalized_position()
      .SetFromVector(q);
  const ContinuousState<T>& xc_deriv = this->EvalTimeDerivatives(*context);
  *result = xc_deriv.CopyToVector().block(
      xc_deriv.num_q(), 0, xc_deriv.num_v() + xc_deriv.num_z(), 1);
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
bool SecondOrderImplicitEulerIntegrator<T>::StepImplicitEuler(
    const T& t0, const T& h, const VectorX<T>& xt0,
    const VectorX<T>& xtplus_guess, VectorX<T>* xtplus,
    typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix,
    MatrixX<T>* Jv, int trial) {
  using std::abs;

  // Verify the trial number is valid.
  DRAKE_ASSERT(trial >= 1 && trial <= 4);
  DRAKE_LOGGER_DEBUG("StepImplicitEuler(h={}) t={}", h, t0);

  const System<T>& system = this->get_system();
  // Verify xtplus
  DRAKE_ASSERT(xtplus != nullptr && xtplus->size() == xt0.size() &&
               xtplus_guess.size() == xt0.size());

  *xtplus = xtplus_guess;

  // set up a residual evaluator called velocity_residual_R
  Context<T>* context = this->get_mutable_context();
  const systems::ContinuousState<T>& cstate = context->get_continuous_state();
  const VectorX<T>& qt0 = xt0.block(0, 0, cstate.num_q(), 1);
  const VectorX<T>& yt0 =
      xt0.block(cstate.num_q(), 0, cstate.num_v() + cstate.num_z(), 1);
  // const Vector<T>& doesn't keep a live reference (it forces eigen to copy
  // it),
  //   so use Eigen::Ref to refer to qtplus
  Eigen::Ref<VectorX<T>> qtplus = xtplus->block(0, 0, cstate.num_q(), 1);
  // this reference is just for readibility: qk is qtplus.
  const Eigen::Ref<VectorX<T>>& qk = qtplus;
  std::function<VectorX<T>()> velocity_residual_R = [&qt0, &yt0, h, context,
                                                     &qk, this]() {
    const systems::ContinuousState<T>& cstate_res =
        context->get_continuous_state();
    VectorX<T> y = cstate_res.CopyToVector().block(
        cstate_res.num_q(), 0, cstate_res.num_v() + cstate_res.num_z(), 1);
    VectorX<T> g_of_y(cstate_res.num_v() + cstate_res.num_z());
    this->eval_g_with_y_from_context(qt0, h, qk, &g_of_y);
    return (y - yt0 - h * g_of_y).eval();
  };

  // references to y and v portions of xtplus
  Eigen::Ref<VectorX<T>> ytplus =
      xtplus->block(cstate.num_q(), 0, cstate.num_v() + cstate.num_z(), 1);
  const Eigen::Ref<VectorX<T>> vtplus =
      xtplus->block(cstate.num_q(), 0, cstate.num_v(), 1);

  // Set last_qtplus to qk
  VectorX<T> last_qtplus = qtplus;

  // Initialize the vector for qdot
  BasicVector<T> qdot(qt0.rows());

  // Advance the context time and state to compute derivatives at t0 + h.
  const T tf = t0 + h;
  context->SetTimeAndContinuousState(tf, *xtplus);

  // Evaluate the residual error for equation (11) from Ante's doc
  VectorX<T> residual = velocity_residual_R();

  // Initialize the "last" state update norm; this will be used to detect
  // convergence.
  VectorX<T> dx(xt0.size());
  T last_dx_norm = std::numeric_limits<double>::infinity();
  // Full Newton: Calculate Jacobian and iteration matrices (and
  // factorizations), as needed, around (tf, xtplus).
#ifdef FULL_NEWTON
  if (trial < 3) trial = 3;
#endif
  if (!this->MaybeFreshenVelocityMatrices(
          tf, *xtplus, qt0, h, trial,
          ComputeAndFactorImplicitEulerIterationMatrix, iteration_matrix, Jv)) {
    return false;
  }
  int num_nr_iterations_at_beginning = num_nr_iterations_;

  // Do the Newton-Raphson iterations.
  for (int i = 0; i < this->max_newton_raphson_iterations(); ++i) {
    // Update the number of Newton-Raphson iterations.
    num_nr_iterations_++;

    // Compute the state update using the equation A*y = -r(), where A is the
    // iteration matrix.
    VectorX<T> dy = iteration_matrix->Solve(-residual);

    if (i > 0 && this->IsUpdateZero(ytplus, dy)) return true;
    // Update the y portion of xtplus
    ytplus += dy;

    // assume context has the last qk, evaluate N(qk) and update q
    system.MapVelocityToQDot(*context, vtplus, &qdot);
    last_qtplus = qtplus;
    // Update the q portion of xtplus
    qtplus = qt0 + h * qdot.get_value();
    dx << dy, qtplus - last_qtplus;

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
      DRAKE_LOGGER_DEBUG(
          "Newton-Raphson loop {}, dx {}, last dx {}, theta: {}, eta: {}", i,
          dx_norm, last_dx_norm, theta, eta);

      // Look for divergence.
      if (theta > 1) {
        DRAKE_LOGGER_DEBUG(
            "Newton-Raphson divergence detected for "
            "h={}",
            h);
        break;
      }

      // Look for convergence using Equation 8.10 from [Hairer, 1996].
      // [Hairer, 1996] determined values of kappa in [0.01, 0.1] work most
      // efficiently on a number of test problems with Radau5 (a fifth order
      // implicit integrator), p. 121. We select a value halfway in-between.
      const double kappa = 0.05;
      const double k_dot_tol = kappa * this->get_accuracy_in_use();
      if (eta * dx_norm < k_dot_tol) {
        DRAKE_LOGGER_DEBUG("Newton-Raphson converged; η = {}, h = {}", eta, h);
        return true;
      }
    }
    last_dx_norm = dx_norm;
    residual = velocity_residual_R();

#ifdef FULL_NEWTON
    // update the Jacobian for full newton
    if (!this->MaybeFreshenVelocityMatrices(
            tf, *xtplus, qt0, h, 3,
            ComputeAndFactorImplicitEulerIterationMatrix, iteration_matrix,
            Jv)) {
      return false;
    }
#endif
  }

  DRAKE_LOGGER_DEBUG("SOIE convergence failed");
  num_nr_iterations_that_end_in_failure_ +=
      num_nr_iterations_ - num_nr_iterations_at_beginning;
  num_nr_failures_++;

  // If Jacobian and iteration matrix factorizations are not reused, there
  // is nothing else we can try.
  if (!this->get_reuse()) return false;

  // Try StepImplicitEuler again. That method will
  // freshen Jacobians and iteration matrix factorizations as necessary.
  return StepImplicitEuler(t0, h, xt0, xtplus_guess, xtplus, iteration_matrix,
                           Jv, trial + 1);
}

// Steps forward by two half implicit euler steps, if possible
template <class T>
bool SecondOrderImplicitEulerIntegrator<T>::StepHalfImplicitEulers(
    const T& t0, const T& h, const VectorX<T>& xt0,
    const VectorX<T>& xtplus_guess, VectorX<T>* xtplus,
    typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix,
    MatrixX<T>* Jv) {
  // Store statistics before "error control". The difference between
  // the modified statistics and the stored statistics will be used to compute
  // the half-sized-step-specific statistics.
  int stored_num_jacobian_evaluations = this->get_num_jacobian_evaluations();
  int stored_num_iter_factorizations =
      this->get_num_iteration_matrix_factorizations();
  int64_t stored_num_function_evaluations =
      this->get_num_derivative_evaluations();
  int64_t stored_num_jacobian_function_evaluations =
      this->get_num_derivative_evaluations_for_jacobian();
  int stored_num_nr_iterations = this->get_num_newton_raphson_iterations();
  int stored_num_nr_iterations_that_end_in_failure =
      this->get_num_newton_raphson_iterations_that_end_in_failure();
  int stored_num_nr_failures = this->get_num_newton_raphson_failures();

  bool success = StepImplicitEuler(t0, 0.5 * h, xt0, 0.5 * (xt0 + xtplus_guess),
                                   xtplus, iteration_matrix, Jv);
  if (!success) {
    DRAKE_LOGGER_DEBUG("First Half SOIE convergence failed");
  } else {
    // set new xt0 to xthalf
    VectorX<T> xthalf = *xtplus;
    success = StepImplicitEuler(t0 + 0.5 * h, 0.5 * h, xthalf, xthalf, xtplus,
                                iteration_matrix, Jv);
    if (!success) {
      DRAKE_LOGGER_DEBUG("Second Half SOIE convergence failed");
    }
  }
  // Move statistics to error estimate-specific.
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
  num_err_est_nr_iterations_ +=
      this->get_num_newton_raphson_iterations() - stored_num_nr_iterations;
  num_err_est_nr_iterations_that_end_in_failure_ +=
      this->get_num_newton_raphson_iterations_that_end_in_failure() -
      stored_num_nr_iterations_that_end_in_failure;
  num_err_est_nr_failures_ +=
      this->get_num_newton_raphson_failures() - stored_num_nr_failures;

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
bool SecondOrderImplicitEulerIntegrator<T>::AttemptStepPaired(
    const T& t0, const T& h, const VectorX<T>& xt0, VectorX<T>* xtplus_ie,
    VectorX<T>* xtplus_hie) {
  using std::abs;
  DRAKE_ASSERT(xtplus_ie);
  DRAKE_ASSERT(xtplus_hie);

  // Use the current state as the candidate value for the next state.
  // [Hairer 1996] validates this choice (p. 120).
  const VectorX<T>& xtplus_guess = xt0;

  // Do the Implicit Euler step.
  if (!StepImplicitEuler(t0, h, xt0, xtplus_guess, xtplus_ie,
                         &iteration_matrix_ie_, &Jv_ie_)) {
    DRAKE_LOGGER_DEBUG(
        "SO Implicit Euler approach did not converge for "
        "step size {}",
        h);
    return false;
  }

  // Do the half Implicit Euler steps.
  // TODO(antequ): consider turning this off for fixed step mode (this needs
  //   to stay on to pass some of the error-control tests.). To do so, just
  //   begin the if statement with "if (this->get_fixed_step_mode() || "
  // REUSE the Jacobian and A Matrix from the big step because it works quite
  //   well from the examples Ante ran.
  if (StepHalfImplicitEulers(t0, h, xt0, *xtplus_ie, xtplus_hie,
                             &iteration_matrix_ie_, &Jv_ie_)) {
    Context<T>* context = this->get_mutable_context();
    // As per discussions, we propagate the half steps
    context->SetTimeAndContinuousState(t0 + h, *xtplus_hie);

    return true;
  } else {
    DRAKE_LOGGER_DEBUG(
        "SO Implicit Euler half-step approach failed with a step size "
        "that succeeded for the normal approach {}",
        h);
    return false;
  }
}

/// Takes a given step of the requested size, if possible.
/// @returns `true` if successful and `false` otherwise; on `true`, the time
///          and continuous state will be advanced in the context (e.g., from
///          t0 to t0 + h). On `false` return, the time and continuous state in
///          the context will be restored to its original value (at t0).
template <class T>
bool SecondOrderImplicitEulerIntegrator<T>::DoImplicitIntegratorStep(
    const T& h) {
  // Save the current time and state.
  Context<T>* context = this->get_mutable_context();
  const T t0 = context->get_time();
  DRAKE_LOGGER_DEBUG("IE DoStep(h={}) t={}", h, t0);

  xt0_ = context->get_continuous_state().CopyToVector();
  xtplus_ie_.resize(xt0_.size());
  xtplus_hie_.resize(xt0_.size());

  // If the requested h is less than the minimum step size, we'll advance time
  // using an explicit Euler step.
  if (h < this->get_working_minimum_step_size()) {
    DRAKE_LOGGER_DEBUG(
        "-- requested step too small, taking explicit "
        "step instead");

    // The error estimation process for explicit Euler uses an explicit second
    // order Runge-Kutta method so that the order of the asymptotic term
    // matches that used for estimating the error of the implicit Euler
    // integrator.

    // Compute the explicit Euler step.
    xdot_ = this->EvalTimeDerivatives(*context).CopyToVector();
    xtplus_ie_ = xt0_ + h * xdot_;

    // Compute the RK2 step.
    const int evals_before_rk2 = rk2_->get_num_derivative_evaluations();
    if (!rk2_->IntegrateWithSingleFixedStepToTime(t0 + h)) {
      throw std::runtime_error(
          "Embedded RK2 integrator failed to take a single"
          "fixed step to the requested time.");
    }

    const int evals_after_rk2 = rk2_->get_num_derivative_evaluations();
    xtplus_hie_ = context->get_continuous_state().CopyToVector();

    // Update the error estimation ODE counts.
    num_err_est_function_evaluations_ += (evals_after_rk2 - evals_before_rk2);

    // Revert the state to that computed by explicit Euler.
    context->SetTimeAndContinuousState(t0 + h, xtplus_ie_);
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
  err_est_vec_ = (xtplus_ie_ - xtplus_hie_);

  // Update the caller-accessible error estimate.
  this->get_mutable_error_estimate()->get_mutable_vector().SetFromVector(
      err_est_vec_);

  return true;
}

}  // namespace systems
}  // namespace drake
