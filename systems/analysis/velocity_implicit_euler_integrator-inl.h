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

/// Uses first-order forward differencing to compute the Jacobian, Jₗₖ(y), of
/// the function lₖ(y), used in this integrator's residual computation, with
/// respect to y. As defined before, y = (v, z). This Jacobian is then defined
/// as:
///     lₖ(y)  = f(tⁿ⁺¹, qⁿ + h N(qₖ) v, y)    (9)
///     Jₗₖ(y) = ∂lₖ(y)/∂y                     (10)
//
/// In this method, we compute the Jacobian Jₗₖ(y) using a first-order forward
/// difference (i.e. numerical differentiation),
///   Jₗₖ(y)ᵢⱼ = (lₖ(y')ᵢ - lₖ(y)ᵢ )/ δy(j),
///      where y' = y + δy(j) eⱼ and δy(j) = (√ε) max(1,|yⱼ|).
/// In the code we hereby refer to y as "the baseline" and y' as "prime".
/// @param t refers to tⁿ⁺¹, the time used in the definition of lₖ(y)
/// @param h is the timestep size parameter, h, used in the definition of
///        lₖ(y)
/// @param x is (qₖ, y), the continuous state around which to evaluate Jₗₖ(y)
/// @param qt0 refers to qⁿ, the initial position used in lₖ(y)
/// @param context the Context of the system, at time and continuous state
///        unknown.
/// @param [out] Jv is the Jacobian matrix, Jₗₖ(y).
/// @note The continuous state will be indeterminate on return.
/// For full Newton, we recommend this method to be evaluated at time t = t0 +
/// h and x = xₖ; however, this recommendation is not necessary for the
/// integrator and the logic in MaybeFreshenVelocityMatrices modifies this.
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
      "  ImplicitEulerIntegrator Compute ForwardDiffVelocityJacobian "
      "{}-Jacobian t={}",
      ny, t);
  DRAKE_LOGGER_DEBUG("  computing from state {}", x.transpose());

  // Initialize the Jacobian.
  Jv->resize(ny, ny);

  // qk, v, y refer to the baseline state from x.
  const auto& qk = x.head(nq);
  const auto& v = x.segment(nq, nv);
  const auto& y = x.tail(ny);

  // Evaluate qt0_plus_hNv = qⁿ + h N(qₖ) v with (qₖ,v) from x
  VectorX<T> qt0_plus_hNv(nq);
  BasicVector<T> qdot(nq);
  context->SetTimeAndContinuousState(t, x);
  this->get_system().MapVelocityToQDot(*context, v, &qdot);
  qt0_plus_hNv = qt0 + h * qdot.get_value();

  // Compute the Jacobian.

  // Define x' = (qⁿ + h N(qₖ) v, y), to compute the baseline lₖ(y), and then
  // reuse x' = (qⁿ + h N(qₖ) v', y') to compute each prime lₖ(y')
  VectorX<T> x_prime = x;
  Eigen::Ref<VectorX<T>> q_prime = x_prime.head(nq);
  Eigen::Ref<VectorX<T>> v_prime = x_prime.segment(nq, nv);
  Eigen::Ref<VectorX<T>> y_prime = x_prime.tail(ny);
  q_prime = qt0_plus_hNv;

  // Initialize the finite-difference baseline, lₖ(y), by
  //  evaluating the context at lₖ(y) = f(t, qⁿ + h N(qₖ) v, y) = f(t,x).
  context->get_mutable_continuous_state()
      .get_mutable_generalized_position()
      .SetFromVector(q_prime);
  const VectorX<T> l_of_y = this->EvalTimeDerivatives(*context).CopyToVector();

  // Now evaluate each lₖ(y') where y' modifies one value of y at a time
  for (int j = 0; j < ny; ++j) {
    // Compute a good increment, δy(j), to dimension j of y using approximately
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
    // Compute lₖ(y') and set the relevant column of the Jacobian matrix.
    context->SetTimeAndContinuousState(t, x_prime);
    Jv->col(j) =
        (this->EvalTimeDerivatives(*context).CopyToVector() - l_of_y).tail(ny) /
        dyj;

    // Reset y' to y.
    y_prime(j) = y(j);
  }
}

/// Compute the partial derivative of the ordinary differential equations with
/// respect to the y variables of a given x(t). In particular, we compute the
/// Jacobian, Jₗₖ(y), of the function lₖ(y), used in this integrator's
/// residual computation, with respect to y. As defined before, y = (v,z) and
/// x = (q,v,z). This Jacobian is then defined as:
///     lₖ(y)  = f(tⁿ⁺¹, qⁿ + h N(qₖ) v, y)    (9)
///     Jₗₖ(y) = ∂lₖ(y)/∂y                     (10)
/// @param t refers to tⁿ⁺¹, the time used in the definition of lₖ(y)
/// @param h is the timestep size parameter, h, used in the definition of
///        lₖ(y)
/// @param x is (qₖ, y), the continuous state around which to evaluate Jₗₖ(y)
/// @param qt0 refers to qⁿ, the initial position used in lₖ(y)
/// @param [out] Jv is the Jacobian matrix, Jₗₖ(y).
/// @post the context's time and continuous state will be temporarily set
///       during this call (and then reset to their original values) on
///       return.
template <class T>
void SecondOrderImplicitEulerIntegrator<T>::CalcVelocityJacobian(
    const T& t, const T& h, const VectorX<T>& x, const VectorX<T>& qt0,
    MatrixX<T>* Jv) {
  // Just like ImplicitIntegrator<T>::CalcJacobian, we change the context but
  // will change it back. The user should not assume any caches remain after we
  // finish.
  Context<T>* context = this->get_mutable_context();

  // Get the current time and state.
  const T t_current = context->get_time();
  const ContinuousState<T>& cstate = context->get_continuous_state();
  const VectorX<T> x_current = cstate.CopyToVector();

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

  // Use the new number of ODE evaluations to determine the number of ODE
  // evaluations used in computing Jacobians.
  this->increment_jacobian_computation_derivative_evaluations(
      this->get_num_derivative_evaluations() - current_ODE_evals);

  // Reset the time and state.
  context->SetTimeAndContinuousState(t_current, x_current);
}

/// Computes necessary matrices (Jacobian and iteration matrix) for
/// Newton-Raphson (NR) iterations, as necessary. This method is based off of
/// ImplicitIntegrator<T>::MaybeFreshenMatrices. We implement our own version
/// here to use a specialized Velocity Jacobian. The aformentioned method was
/// designed for use in DoImplicitIntegratorStep() processes that follow this
/// model:
/// 1. DoImplicitIntegratorStep(h) is called;
/// 2. One or more NR iterations is performed until either (a) convergence is
///    identified, (b) the iteration is found to diverge, or (c) too many
///    iterations were taken. In the case of (a), DoImplicitIntegratorStep(h)
///    will return success. Otherwise, the Newton-Raphson process is attempted
///    again with (i) a recomputed and refactored iteration matrix and (ii) a
///    recomputed Jacobian and a recomputed an refactored iteration matrix, in
///    that order. The process stage of that NR algorithm is indicated by the
///    `trial` parameter below. In this model, DoImplicitIntegratorStep()
///    returns failure if the NR iterations reach a fourth trial.
///
/// Note that the sophisticated logic above only applies when the Jacobian
/// reuse is activated (default, see get_reuse()).
///
/// @param t the time at which to compute the Jacobian.
/// @param xt the continuous state at which the Jacobian is computed.
/// @param qt0 the generalized position at the beginning of the step
/// @param h the integration step size
/// @param trial which trial (1-4) the Newton-Raphson process is in when
///        calling this method.
/// @param compute_and_factor_iteration_matrix a function pointer for
///        computing and factoring the iteration matrix.
/// @param [out] iteration_matrix the updated and factored iteration matrix on
///             return.
/// @param [out] Jv the updated and factored velocity Jacobian matrix on
///             return.
/// @returns `false` if the calling stepping method should indicate failure;
///          `true` otherwise.
/// @pre 1 <= `trial` <= 4.
/// @post the state in the internal context may or may not be altered on
///       return; if altered, it will be set to (t, xt).
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
      // For the third trial, we reform the Jacobian matrix and refactor the
      // iteration matrix.

      // note: Based on a few simple experimental tests, we found that the
      // optimization when matrices are already fresh in
      // ImplicitIntegrator<T>::MaybeFreshenMatrices does not significantly help
      // here, especially because our Jacobian depends on step size h.
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

/// This helper method evaluates the Newton-Raphson residual R(y), defined as
/// the following:
///   R(y)  = y - yⁿ - h lₖ(y),
///   lₖ(y) = f(tⁿ⁺¹, qⁿ + h N(qₖ) v, y),    (9)
///  with tⁿ⁺¹, qₖ, y derived from the context and qⁿ, yⁿ, h passed in.
/// @param qt0 is qⁿ, the generalized position at the beginning of the step
/// @param yt0 is yⁿ, the generalized velocity and miscellaneous states at the
///        beginning of the step
/// @param h is the step size
/// @param [out] result is set to R(y)
/// @post The position of the context is first altered and then restored to
///       the original position. This might invalidate some caches that depend
///       on the position.
template <class T>
VectorX<T> SecondOrderImplicitEulerIntegrator<T>::ComputeResidualR(
    const VectorX<T>& qt0, const VectorX<T>& yt0, const T& h) {
  Context<T>* context = this->get_mutable_context();
  const systems::ContinuousState<T>& cstate = context->get_continuous_state();
  // Save the initial qk, yk to reset at the end of this method.
  const VectorX<T> xk = cstate.CopyToVector();
  int nq = qt0.rows();
  int ny = yt0.rows();

  const auto& qk = xk.head(nq);
  const auto& yk = xk.tail(ny);

  // Suppose context has state (qk, v, z).
  // Compute q = qt0 + h N(qk) v
  BasicVector<T> qdot(nq);
  this->get_system().MapVelocityToQDot(
      *context, cstate.get_generalized_velocity(), &qdot);
  const VectorX<T> q = qt0 + h * qdot.get_value();

  // Evaluate l = f(q, v, z)
  context->get_mutable_continuous_state()
      .get_mutable_generalized_position()
      .SetFromVector(q);
  const ContinuousState<T>& xc_deriv = this->EvalTimeDerivatives(*context);
  const VectorX<T> l_of_y = xc_deriv.CopyToVector().tail(ny);

  // reset the context back.
  context->get_mutable_continuous_state()
      .get_mutable_generalized_position()
      .SetFromVector(qk);
  return (yk - yt0 - h * l_of_y).eval();
}

/// Steps the system forward by a single step of at most h using the
/// Second-Order Implicit Euler method.
/// @param t0 the time at the left end of the integration interval.
/// @param h the maximum time increment to step forward.
/// @param xt0 the continuous state at t0.
/// @param xtplus_guess the starting guess for x(t0+h).
/// @param [out] xtplus the computed value for `x(t0+h)` on successful return.
/// @param [in, out] iteration_matrix the cached iteration matrix
/// @param [in, out] Jv the cached velocity Jacobian
/// @param trial the attempt for this approach (1-4). StepImplicitEuler() uses
///        more computationally expensive methods as the trial numbers
///        increase.
/// @returns `true` if the step of size `h` was successful, `false` otherwise.
/// @note The time and continuous state in the context are indeterminate upon
///       exit.
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

  // Initialize xtplus to the guess
  *xtplus = xtplus_guess;

  Context<T>* context = this->get_mutable_context();
  const systems::ContinuousState<T>& cstate = context->get_continuous_state();
  int nq = cstate.num_q();
  int nv = cstate.num_v();
  int nz = cstate.num_z();
  const auto& qt0 = xt0.head(nq);
  const auto& yt0 = xt0.tail(nv + nz);

  // Define references to q, y, v, and z portions of xtplus for readibility.
  Eigen::Ref<VectorX<T>> qtplus = xtplus->head(nq);
  Eigen::Ref<VectorX<T>> ytplus = xtplus->tail(nv + nz);
  const auto& vtplus = xtplus->segment(nq, nv);
  const auto& ztplus = xtplus->tail(nz);
  unused(ztplus);

  // Set last_qtplus to qk. This will be used in computing dx to determine
  // convergence.
  VectorX<T> last_qtplus = qtplus;

  // Initialize the vector for qdot.
  BasicVector<T> qdot(nq);

  // Advance the context time and state to compute derivatives at t0 + h.
  const T tf = t0 + h;
  context->SetTimeAndContinuousState(tf, *xtplus);

  // Initialize the "last" state update norm; this will be used to detect
  // convergence.
  VectorX<T> dx(xt0.size());
  T last_dx_norm = std::numeric_limits<double>::infinity();

  // Refresh Jacobian and iteration matrices (and factorizations),
  // as needed, around (t0, xt0).
#ifdef FULL_NEWTON
  // Set the trial number to 3 to force a Jacobian recomputation.
  if (trial < 3) trial = 3;
  if (!this->MaybeFreshenVelocityMatrices(
          tf, *xtplus, qt0, h, trial,
#else
  if (!this->MaybeFreshenVelocityMatrices(
          t0, xt0, qt0, h, trial,
#endif
          ComputeAndFactorImplicitEulerIterationMatrix, iteration_matrix, Jv)) {
    return false;
  }

  // Evaluate the residual error, which is the negation of the RHS of the update
  // equation:
  //     (I - h Jₗₖ) Δy = yⁿ - yₖ + h lₖ(yₖ), Δy = yₖ₊₁ - yₖ
  VectorX<T> residual = ComputeResidualR(qt0, yt0, h);

  int num_nr_iterations_at_beginning = num_nr_iterations_;

  // Do the Newton-Raphson iterations.
  for (int i = 0; i < this->max_newton_raphson_iterations(); ++i) {
    // Update the number of Newton-Raphson iterations.
    num_nr_iterations_++;

    // Compute the state update using the equation A*y = -R(), where A is the
    // iteration matrix.
    const VectorX<T> dy = iteration_matrix->Solve(-residual);

    // When dy is 0, we exit because the implicit step no longer provides any
    // improvement; this is necessary because if dq = 0 as well after it
    // converges, we will have theta = nan in the next iteration, which will
    // fail to trigger the divergence check.
    if (i > 0 && this->IsUpdateZero(ytplus, dy)) return true;

    // Update the y portion of xtplus.
    ytplus += dy;

    // Update the q portion of xtplus. Note that at this point, the context has
    // its position q equal to qtplus = qₖ, because ComputeResidualR was the
    // last function to modify the context. This means that we can directly call
    // MapVelocityToQDot on the context, which will evaluate N(qₖ).
    system.MapVelocityToQDot(*context, vtplus, &qdot);
    qtplus = qt0 + h * qdot.get_value();
    dx << qtplus - last_qtplus, dy;

    // Get the infinity norm of the weighted update vector.
    dx_state_->get_mutable_vector().SetFromVector(dx);
    T dx_norm = this->CalcStateChangeNorm(*dx_state_);

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
    last_qtplus = qtplus;

#ifdef FULL_NEWTON
    // update the Jacobian for full newton
    if (!this->MaybeFreshenVelocityMatrices(
            tf, *xtplus, qt0, h, trial,
            ComputeAndFactorImplicitEulerIterationMatrix, iteration_matrix,
            Jv)) {
      return false;
    }
#endif
    residual = ComputeResidualR(qt0, yt0, h);
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

  // We set our guess for the state after a half-step to the average of the
  // guess for the final state, xtplus_guess, and the initial state, xt0.
  const VectorX<T> xthalf_guess = 0.5 * (xt0 + xtplus_guess);
  bool success = StepImplicitEuler(t0, 0.5 * h, xt0, xthalf_guess, xtplus,
                                   iteration_matrix, Jv);
  if (!success) {
    DRAKE_LOGGER_DEBUG("First Half SOIE convergence failed");
  } else {
    // set new xt0 to xthalf
    const VectorX<T> xthalf = *xtplus;
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

  // Compute and update the error estimate. IntegratorBase will use the norm of
  // this vector to adjust step size.
  err_est_vec_ = (xtplus_ie_ - xtplus_hie_);

  // Update the caller-accessible error estimate.
  this->get_mutable_error_estimate()->get_mutable_vector().SetFromVector(
      err_est_vec_);

  return true;
}

}  // namespace systems
}  // namespace drake
