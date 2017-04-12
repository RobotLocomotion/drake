#pragma once

/// @file
/// Template method implementations.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>

#include "drake/common/text_logging.h"
#include "drake/math/autodiff.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"

namespace drake {
namespace systems {

template <class T>
void ImplicitEulerIntegrator<T>::DoResetStatistics() {
  num_nr_loops_ = num_err_est_nr_loops_ = 0;
  num_err_est_function_evaluations_ = 0;
  num_jacobian_function_evaluations_ =
    num_err_est_jacobian_function_evaluations_ = 0;
  num_jacobian_evaluations_ = num_err_est_jacobian_reforms_ = 0;
  num_iter_factorizations_ = num_err_est_iter_factorizations_ = 0;
  num_shrinkages_from_error_control_ = 0;
  num_shrinkages_from_substep_failures_ = 0;
  num_substep_failures_ = 0;
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
  if (working_accuracy > kLoosestAccuracy)
    working_accuracy = kLoosestAccuracy;
  else if (isnan(working_accuracy))
    working_accuracy = kDefaultAccuracy;
  this->set_accuracy_in_use(working_accuracy);
}

// Computes the Jacobian of the ordinary differential equations, evaluated at
// xtplus, taken with respect to the state variables using automatic
// differentiation.
template <>
MatrixX<AutoDiffXd> ImplicitEulerIntegrator<AutoDiffXd>::
    ComputeAutoDiffJacobian(const System<AutoDiffXd>& system,
                            const Context<AutoDiffXd>& context,
                            ContinuousState<AutoDiffXd>* state) {
        throw std::runtime_error("AutoDiff'd Jacobian not supported from "
                                     "AutoDiff'd ImplicitEulerIntegrator");
}

// Computes the Jacobian of the ordinary differential equations, evaluated at
// xtplus, taken with respect to the state variables using automatic
// differentiation.
template <class T>
MatrixX<T> ImplicitEulerIntegrator<T>::ComputeAutoDiffJacobian(
    const System<T>& system, const Context<T>& context,
    ContinuousState<T>* state) {
  SPDLOG_DEBUG(drake::log(), "  IE Compute Autodiff Jacobian t={}",
               context.get_time());
  // Create AutoDiff versions of the state vector.
  typedef AutoDiffXd Scalar;
  VectorX<Scalar> a_xtplus = state->CopyToVector();

  // Set the size of the derivatives and prepare for Jacobian calculation.
  const int n_state_dim = a_xtplus.size();
  for (int i = 0; i < n_state_dim; ++i)
    a_xtplus[i].derivatives() = VectorX<T>::Unit(n_state_dim, i);

  // Get the system and the context in AutoDiffable format. Inputs must also
  // be copied to the AutoDiff'd system (which is accomplished using
  // FixInputPortsFrom()).
  const auto adiff_system = system.ToAutoDiffXd();
  std::unique_ptr<Context<Scalar>> adiff_context = adiff_system->
      AllocateContext();
  adiff_context->SetTimeStateAndParametersFrom(context);
  adiff_system->FixInputPortsFrom(system, context, adiff_context.get());

  // Set the continuous state in the context.
  adiff_context->get_mutable_continuous_state()->get_mutable_vector()->
      SetFromVector(a_xtplus);

  // Evaluate the derivatives at that state.
  std::unique_ptr<ContinuousState<Scalar>> derivs =
      adiff_system->AllocateTimeDerivatives();
  IntegratorBase<T>::CalcTimeDerivatives(*adiff_system, *adiff_context,
                                         derivs.get());

  // Get the Jacobian.
  auto result = derivs->CopyToVector().eval();
  return math::autoDiffToGradientMatrix(result);
}

// Evaluates the ordinary differential equations at a given state. Permits
// counting the number of function evaluations for a given integration step.
template <class T>
VectorX<T> ImplicitEulerIntegrator<T>::CalcTimeDerivatives() {
  IntegratorBase<T>::CalcTimeDerivatives(this->get_context(), derivs_.get());
  return derivs_->CopyToVector();
}

// Computes the Jacobian of the ordinary differential equations, evaluated at
// xtplus, taken with respect to the state variables, using a first-order
// central difference (i.e., numerical differentiation).
template <class T>
MatrixX<T> ImplicitEulerIntegrator<T>::ComputeForwardDiffJacobian(
    const System<T>& system, const Context<T>& context,
    ContinuousState<T>* state) {
  using std::abs;

  // Set epsilon to the square root of machine precision.
  const double eps = std::sqrt(std::numeric_limits<double>::epsilon());

  // Get the number of state variables.
  const int n = state->size();

  // Get the current continuous state.
  const VectorX<T> xtplus = state->CopyToVector();

  SPDLOG_DEBUG(drake::log(), "  IE Compute Forwarddiff {}-Jacobian t={}",
               n, context.get_time());
  SPDLOG_DEBUG(drake::log(), "  computing from state {}", xtplus.transpose());

  // Initialize the Jacobian.
  MatrixX<T> J(n, n);

  // Evaluate f(t+h,xtplus) for the current state (current xtplus).
  VectorX<T> f = CalcTimeDerivatives();

  // Compute the Jacobian.
  VectorX<T> xtplus_prime = xtplus;
  for (int i = 0; i < n; ++i) {
    // Compute a good increment to the dimension.
    const T abs_xi = abs(xtplus(i));
    T dxi(eps * abs_xi);
    if (abs_xi <= 0)
      dxi = eps;

    // Update xtplus', minimizing the effect of roundoff error by ensuring that
    // x and dx differ by an exactly representable number. See p. 192 of
    // Press, W., Teukolsky, S., Vetterling, W., and Flannery, P. Numerical
    //   Recipes in C++, 2nd Ed., Cambridge University Press, 2002.
    xtplus_prime(i) = xtplus(i) + dxi;
    dxi = xtplus_prime(i) - xtplus(i);

    // Compute f' and set the relevant column of the Jacobian matrix.
    state->SetFromVector(xtplus_prime);
    J.col(i) = (CalcTimeDerivatives() - f) / dxi;

    // Reset xtplus' to xtplus.
    xtplus_prime(i) = xtplus(i);
  }

  return J;
}

// Computes the Jacobian of the ordinary differential equations, evaluated at
// xtplus, taken with respect to the state variables, using a second-order
// central difference  (i.e., numerical differentiation).
template <class T>
MatrixX<T> ImplicitEulerIntegrator<T>::ComputeCentralDiffJacobian(
    const System<T>& system, const Context<T>& context,
    ContinuousState<T>* state) {
  using std::abs;

  // Cube root of machine precision (indicated by theory) seems a bit coarse.
  // Pick power of eps halfway between 6/12 (i.e., 1/2) and 4/12 (i.e., 1/3).
  const double eps = std::pow(std::numeric_limits<double>::epsilon(), 5.0/12);

  // Get the number of state variables.
  const int n = state->size();

  SPDLOG_DEBUG(drake::log(), "  IE Compute Centraldiff {}-Jacobian t={}",
               n, context.get_time());

  // Initialize the Jacobian.
  MatrixX<T> J(n, n);

  // Get the current continuous state.
  const VectorX<T> xtplus = state->CopyToVector();

  // Compute the Jacobian.
  VectorX<T> xtplus_prime = xtplus;
  for (int i = 0; i < n; ++i) {
    // Compute a good increment to the dimension.
    const T abs_xi = abs(xtplus(i));
    T dxi(eps * abs_xi);
    if (abs_xi <= 0)
      dxi = eps;

    // Update xtplus', minimizing the effect of roundoff error, by ensuring that
    // x and dx differ by an exactly representable number. See p. 192 of
    // Press, W., Teukolsky, S., Vetterling, W., and Flannery, P. Numerical
    //   Recipes in C++, 2nd Ed., Cambridge University Press, 2002.
    xtplus_prime(i) = xtplus(i) + dxi;
    const T dxi_plus = xtplus_prime(i) - xtplus(i);

    // Compute f(x+dx).
    state->SetFromVector(xtplus_prime);
    VectorX<T> fprime_plus = CalcTimeDerivatives();

    // Update xtplus' again, minimizing the effect of roundoff error.
    xtplus_prime(i) = xtplus(i) - dxi;
    const T dxi_minus = xtplus(i) - xtplus_prime(i);

    // Compute f(x-dx).
    state->SetFromVector(xtplus_prime);
    VectorX<T> fprime_minus = CalcTimeDerivatives();

    // Set the Jacobian column.
    J.col(i) = (fprime_plus - fprime_minus) / (dxi_plus + dxi_minus);

    // Reset xtplus' to xtplus.
    xtplus_prime(i) = xtplus(i);
  }

  return J;
}

// Solves a linear system
template <class T>
VectorX<T> ImplicitEulerIntegrator<T>::Solve(const MatrixX<T>& A,
                                             const VectorX<T>& b) {
  LU_.compute(A);
  return LU_.solve(b);
}

// Solves a linear system
template <>
VectorX<AutoDiffXd> ImplicitEulerIntegrator<AutoDiffXd>::Solve(
    const MatrixX<AutoDiffXd>& A,
    const VectorX<AutoDiffXd>& b) {
  QR_.compute(A);
  return QR_.solve(b);
}

// Performs the bulk of the stepping computation for both implicit Euler and
// implicit trapezoid method- they're very similar.
// @param dt the integration step size to attempt.
// @param g the particular implicit function to zero.
// @param scale a scale factor- either 1 or 2- that allows this method to be
//        used by both implicit Euler and implicit trapezoid methods.
// @param shrink_ok if set to `true`, the method will recursively call
//        StepAbstract() with smaller step sizes (as necessary) until the
//        Newton-Raphson process converges.
// @param [in,out] the starting guess for x(t+dt); the value for x(t+h) on
//        return (assuming that h > 0)
// @retval h if `shrink_ok` is `true`, `h` will be the eventual successful
//           integration step size; if `shrink_ok` is `false`, `h` will either
//           be equal to `dt` (if StepAbstract() was successful), or zero
//           (if StepAbstract() was not successful).
// @pre The time and state of the system are t0 and x(t0) on entry.
// @post The time and state of the system will be set to t0+h and x(t0+h)
//       on exit, where h <= dt.
template <class T>
T ImplicitEulerIntegrator<T>::StepAbstract(T dt,
                          const std::function<VectorX<T>()>& g,
                          double scale,
                          bool shrink_ok,
                          VectorX<T>* xtplus) {
  using std::max;

  // Verify xtplus
  Context<T>* context = this->get_mutable_context();
  DRAKE_ASSERT(xtplus &&
               xtplus->size() == context->get_continuous_state_vector().size());

  // Save initial value of xtplus and initial time.
  const T t0 = context->get_time();
  const VectorX<T> xtplus_star = *xtplus;

  // Get the initial state.
  VectorX<T> xt0 = context->get_continuous_state_vector().CopyToVector();

  SPDLOG_DEBUG(drake::log(), "StepAbstract() entered for t={}, h={}", t0, dt);

  // If the step is below the smallest allowed, take an explicit Euler step.
  // (This could happen from recursive calls of StepAbstract(), each of
  // which shrinks the step size).
  if (dt < this->get_minimum_step_size()) {
    SPDLOG_DEBUG(drake::log(), "StepAbstract()-- taking explicit Euler step");

    // Update the time
    context->set_time(t0 + dt);

    // Update the state.
    IntegratorBase<T>::CalcTimeDerivatives(*context, derivs_.get());
    context->get_mutable_continuous_state()->SetFromVector(
        xt0 + dt*derivs_->CopyToVector());

    return dt;
  }

  // Advance the context time; this means that all derivatives will be computed
  // at t+dt.
  const T tf = context->get_time() + dt;
  context->set_time(tf);
  context->get_mutable_continuous_state()->SetFromVector(*xtplus);

  // Evaluate the residual error using the current x(t+h) as x⁰(t+h):
  // g(x⁰(t+h)) = x⁰(t+h) - x(t) - h f(t+h,x⁰(t+h)), where h = dt;
  VectorX<T> goutput = g();

  // Initialize the "last" state update norm; this will be used to detect
  // convergence.
  T last_dx_norm = std::numeric_limits<double>::infinity();

  // Copy the Jacobian matrix. Copying the Jacobian allows this same step to
  // be repeated multiple times at smaller steps (if necessary) without
  // reforming the Jacobian.
  MatrixX<T> J = J_;

  // The maximum number of Newton-Raphson loops to take before declaring
  // failure. [Hairer, 1996] states, "It is our experience that the code becomes
  // more efficient when we allow a relatively high number of iterations (e.g.,
  // [7 or 10])", p. 121.
  // TODO(edrumwri): Consider making this a settable parameter. Not putting it
  //                 toward staving off parameter overflow.
  const int max_loops = 10;

  // Do the Newton-Raphson loops.
  for (int i = 0; i < max_loops; ++i) {
    // Update the number of Newton-Raphson loops.
    num_nr_loops_++;

    // Compute the state update by computing the negation of the iteration
    // matrix, factorizing it, and solving it. The idea of using the negation
    // of this matrix is that an O(n^2) identity matrix is not formed as would
    // be the case with MatrixX<T>::Identity(n, n) - J * (dt / scale).
    // TODO(edrumwri): Allow caller to provide their own solver.
    const int n = xtplus->size();
    A_ = J * (dt / scale) - MatrixX<T>::Identity(n, n);
    num_iter_factorizations_++;
    VectorX<T> dx = Solve(A_, goutput);

    // Get the infinity norm of the weighted update vector.
    dx_state_->get_mutable_vector()->SetFromVector(dx);
    T dx_norm = this->CalcStateChangeNorm(*dx_state_);

    // Update the state vector.
    *xtplus += dx;

    // Compute the convergence rate and check convergence.
    // [Hairer, 1996] notes that this convergence strategy should only be
    // applied after *at least* two iterations (p. 121).
    if (i >= 1) {
      // [Hairer, 1996] determined values of kappa in [0.01, 0.1] work most
      // efficiently on a number of test problems with *RADAU5* (a fifth order
      // implicit integrator), p. 121. We select a value halfway in-between.
      const double kappa = 0.05;
      const T theta = dx_norm / last_dx_norm;
      const T eta = theta / (1 - theta);

      // Look for divergence.
      if (theta > 1)
        break;

      // Look for convergence using Equation 8.10 from [Hairer, 1996].
      const double k_dot_tol = kappa * this->get_accuracy_in_use();
      if (eta * dx_norm < k_dot_tol) {
        SPDLOG_DEBUG(drake::log(), "Newton-Raphson converged; η = {}", eta);
        return dt;
      }
    }

    // Update the norm of the state update.
    last_dx_norm = dx_norm;

    // Update the state in the context and compute g(xⁱ⁺¹).
    context->get_mutable_continuous_state()->SetFromVector(*xtplus);
    goutput = g();

    // Recompute the Jacobian matrix.
    J = CalcJacobian(tf, *xtplus);
  }

  // If we're here, then the simple Newton-Raphson approach failed. Record a
  // StepAbstract() failure.
  num_substep_failures_++;

  // If this function was called by StepImplicitEuler() (implying that shrink_ok
  // is true), we want to continue shrinking the step size until StepAbstract()
  // no longer fails.
  if (shrink_ok) {
    SPDLOG_DEBUG(drake::log(), "StepAbstract() failed;"
        "trying with smaller step size");

    // Failed because of divergence or after the maximum number of iterations;
    // retry using a smaller step size.
    context->set_time(t0);
    context->get_mutable_continuous_state()->SetFromVector(xt0);
    *xtplus = xtplus_star;
    return StepAbstract(dt / 2, g, scale, true, xtplus);
  } else {
    // This function must have been called by StepImplicitTrapezoid(); failure
    // at this point means that StepImplicitEuler() succeeded with a step of
    // size dt but StepImplicitTrapezoid() did not. Denote failure.
    return 0.0;
  }
}

// Steps the system forward by a single step of at most dt using the implicit
// Euler method.
// @param dt the maximum time increment to step forward.
// @returns the amount actually stepped forward by a single step.
template <class T>
T ImplicitEulerIntegrator<T>::StepImplicitEuler(const T& dt) {
  using std::abs;

  // Get the current continuous state.
  Context<T>* context = this->get_mutable_context();
  const VectorX<T> xt0 = context->get_continuous_state_vector().CopyToVector();

  // Get the target time
  const T t0 = context->get_time();
  T tf = t0 + dt;

  SPDLOG_DEBUG(drake::log(), "StepImplicitEuler(h={}) t={}",
               dt, context->get_time());

  // Set g for the implicit Euler method.
  std::function<VectorX<T>()> g =
      [&xt0, dt, context, this]() {
        return (context->get_continuous_state()->CopyToVector() - xt0 -
            dt*CalcTimeDerivatives()).eval();
      };

  // Use the current state as the candidate value for the next state.
  // [Hairer 1996] validates this choice (p. 120).
  VectorX<T> xtplus = xt0;

  // Compute the initial Jacobian matrix.
  J_ = CalcJacobian(tf, xtplus);

  // Step until successful.
  return StepAbstract(dt, g, 1, true, &xtplus);
}

// Steps forward by a single step of @p dt using the implicit trapezoid
// method, if possible.
// @param dt the maximum time increment to step forward.
// @param dx0 the time derivatives computed at the beginning of the integration
//        step.
// @param xtplus the state computed by the implicit Euler method.
// @returns `true` if the step was successful and `false` otherwise.
template <class T>
bool ImplicitEulerIntegrator<T>::StepImplicitTrapezoid(const T& dt,
                                                    const VectorX<T>& dx0,
                                                    VectorX<T>* xtplus) {
  using std::abs;

  // Get the current continuous state.
  Context<T>* context = this->get_mutable_context();
  const VectorX<T> xt0 = context->get_continuous_state_vector().CopyToVector();

  // Get the target time
  const T t0 = context->get_time();
  T tf = t0 + dt;

  SPDLOG_DEBUG(drake::log(), "StepImplicitTrapezoid(h={}) t={}",
               dt, context->get_time());

  // Set g for the implicit trapezoid method.
  // Define g(x(t+h)) ≡ x(t+h) - x(t) - h/2 (f(t,x(t)) + f(t+h,x(t+h)) and
  // evaluate it at the current x(t+h).
  std::function<VectorX<T>()> g =
      [&xt0, dt, &dx0, context, this]() {
        return (context->get_continuous_state()->CopyToVector() - xt0 -
            dt/2*(dx0 + CalcTimeDerivatives().eval())).eval();
      };

  // Save statistics.
  int saved_num_jacobian_evaluations = num_jacobian_evaluations_;
  int saved_num_iter_factorizations = num_iter_factorizations_;
  int64_t saved_num_function_evaluations =
      this->get_num_derivative_evaluations();
  int64_t saved_num_jacobian_function_evaluations =
      num_jacobian_function_evaluations_;
  int saved_num_nr_loops = num_nr_loops_;

  // Compute the initial Jacobian matrix.
  J_ = CalcJacobian(tf, *xtplus);

  // Step ("false" indicates that the method will not attempt integration step
  // subdivisions).
  T stepped = StepAbstract(dt, g, 2, false, xtplus);

  // Move statistics to implicit trapezoid-specific.
  num_err_est_jacobian_reforms_ +=
      num_jacobian_evaluations_ - saved_num_jacobian_evaluations;
  num_err_est_iter_factorizations_ += num_iter_factorizations_ -
      saved_num_iter_factorizations;
  num_err_est_function_evaluations_ +=
      this->get_num_derivative_evaluations() - saved_num_function_evaluations;
  num_err_est_jacobian_function_evaluations_ +=
      num_jacobian_function_evaluations_ -
          saved_num_jacobian_function_evaluations;
  num_err_est_nr_loops_ += num_nr_loops_ - saved_num_nr_loops;

  return (stepped > 0);
}

// Compute the partial derivative of the ordinary differential equations with
// respect to the new state variables for a given x(t+dt).
// @post the context's time and continuous state will be temporarily set during
//       this call (and then reset to their original values) on return.
template <class T>
MatrixX<T> ImplicitEulerIntegrator<T>::CalcJacobian(const T& tf,
                                                    const VectorX<T>& xtplus) {
  // We change the context but will change it back.
  Context<T>* context = this->get_mutable_context();

  // Get the current time and state.
  T t_current = context->get_time();
  const VectorX<T> x_current = context->get_continuous_state_vector().
      CopyToVector();

  // Update the time and state.
  context->set_time(tf);
  context->get_mutable_continuous_state_vector()->SetFromVector(xtplus);
  num_jacobian_evaluations_++;

  // Get the current number of ODE evaluations.
  int64_t current_ODE_evals = this->get_num_derivative_evaluations();

  // Get a the system.
  const System<T>& system = this->get_system();

  // Get the mutable continuous state.
  ContinuousState<T>* continuous_state = context->
      get_mutable_continuous_state();

  // TODO(edrumwri): Give the caller the option to provide their own Jacobian.
  MatrixX<T> J;
  switch (jacobian_scheme_) {
    case JacobianComputationScheme::kForwardDifference:
      J = ComputeForwardDiffJacobian(system, *context, continuous_state);
      break;

    case JacobianComputationScheme::kCentralDifference:
      J = ComputeCentralDiffJacobian(system, *context, continuous_state);
      break;

    case JacobianComputationScheme::kAutomatic:
      J = ComputeAutoDiffJacobian(system, *context, continuous_state);
      break;

    default:
      // Should never get here.
      DRAKE_ABORT();
  }

  // Use the new number of ODE evaluations to determine the number of Jacobian
  // evaluations.
  num_jacobian_function_evaluations_ += this->get_num_derivative_evaluations()
      - current_ODE_evals;

  // Reset the time and state.
  context->set_time(t_current);
  continuous_state->SetFromVector(x_current);
  return J;
}

template <class T>
std::pair<bool, T> ImplicitEulerIntegrator<T>::DoStepOnceAtMost(
    const T& max_dt) {
  using std::max;
  using std::isnan;
  VectorX<T> xtplus_ie, xtplus_itr;

  Context<T>* context = this->get_mutable_context();

  if (this->get_fixed_step_mode()) {
    this->get_mutable_interval_start_state() =
        context->get_continuous_state_vector().CopyToVector();
    this->DoStepOnceFixedSize(max_dt);
    return std::make_pair(true, max_dt);
  } else {
    // Since the implicit Euler integrator can take its own sub-steps, we
    // adapt IntegratorBase::StepErrorControlled() here. This also allows us
    // to collect fine-grained statistics.
    // *******************************
    // If the directed step is less than the minimum, just go ahead and take it.
    if (max_dt < this->get_minimum_step_size()) {
      this->DoStepOnceFixedSize(max_dt);
      return std::make_pair(true, max_dt);
    }

    // TODO(edrumwri): Move these into IntegratorBase so that they can be
    //                 reused for StepErrorControlled() too.
    // Constants for step size growth and shrinkage.
    const double kDTShrink = 0.95;
    const double kDTGrow = 1.001;

    // Save time and continuous variables, because we'll possibly revert time
    // and state.
    const T t0 = context->get_time();
    VectorX<T> xt0 = context->get_mutable_continuous_state_vector()->
        CopyToVector();

    // Set the "current" step size.
    T dt = this->get_ideal_next_step_size();
    if (isnan(dt)) {
      // Integrator has not taken a step. Set the current step size to the
      // initial step size.
      dt = this->get_initial_step_size_target();
      DRAKE_DEMAND(!isnan(dt));
    }

    T dt_actual;
    bool step_succeeded = false;
    do {
      // If we lose more than a small fraction of the step size we wanted
      // to take due to a need to stop at max_dt, make a note of that so the
      // step size adjuster won't try to grow from the current step.
      bool dt_was_artificially_limited = false;
      if (max_dt < kDTShrink * dt) {
        // max_dt much smaller than current step size.
        dt_was_artificially_limited = true;
        dt = max_dt;
      } else {
        if (max_dt < kDTGrow * dt)
          dt = max_dt;  // max_dt is roughly current step.
      }

      // Take a paired step: implicit Euler and implicit trapezoid must take
      // the same size step so that an error estimate can be computed.
      dt_actual = StepOnceAtMostPaired(dt, &xtplus_ie, &xtplus_itr);
      if (dt_actual < dt) {
        dt_was_artificially_limited = true;
        num_shrinkages_from_substep_failures_++;
      }

      // Compute the error estimate.
      err_est_vec_ = xtplus_ie - xtplus_itr;
      this->get_mutable_error_estimate()->get_mutable_vector()->
          SetFromVector(err_est_vec_);

      //--------------------------------------------------------------------
      T err_norm = this->CalcStateChangeNorm(*this->get_error_estimate());
      SPDLOG_DEBUG(drake::log(), "Norm of error estimate: {}", err_norm);
      std::tie(step_succeeded, dt) = this->CalcAdjustedStepSize(
          err_norm, dt_was_artificially_limited, dt_actual);
      SPDLOG_DEBUG(drake::log(), "Adjusted step size: {}", dt);

      if (step_succeeded) {
        this->set_ideal_next_step_size(dt);
        if (isnan(this->get_actual_initial_step_size_taken()))
          this->set_actual_initial_step_size_taken(dt_actual);

        // Record the adapted step size taken.
        if (isnan(this->get_smallest_adapted_step_size_taken()) ||
            (dt_actual < max_dt &&
                dt_actual < this->get_smallest_adapted_step_size_taken()))
          this->set_smallest_adapted_step_size_taken(dt_actual);
      } else {
        this->report_error_check_failure();
        num_shrinkages_from_error_control_++;

        // Reset the time, state, and time derivative at t0.
        context->set_time(t0);
        context->get_mutable_continuous_state()->SetFromVector(xt0);
      }
    } while (!step_succeeded);

    return std::make_pair(dt_actual == max_dt, dt_actual);
  }
}

// Steps both implicit Euler and implicit trapezoid forward by at most dt
// until both integrators succeed.
// @param dt the initial integration step size to attempt
// @param [out] xtplus_ie contains the Euler integrator solution on return
// @param [out] xtplus_itr contains the implicit trapezoid solution on return
// @pre The time and state are set to {t0,x0} on entry (those at the beginning
//      of the interval.
// @post The time and state are set to {t0+h,x0(t0+h)} on return.
// @retval h The step that integration succeeded at.
template <class T>
T ImplicitEulerIntegrator<T>::StepOnceAtMostPaired(const T& dt,
                                                   VectorX<T>* xtplus_ie,
                                                   VectorX<T>* xtplus_itr) {
  using std::abs;
  DRAKE_ASSERT(xtplus_ie);
  DRAKE_ASSERT(xtplus_itr);

  // Save the time and state at the beginning of this interval.
  Context<T>* context = this->get_mutable_context();
  T t0 = context->get_time();
  const VectorX<T> xt0 = context->get_continuous_state_vector().
      CopyToVector();

  // Compute the derivative at xt0.
  const VectorX<T> dx0 = CalcTimeDerivatives();

  // Do the Euler step.
  T stepped_ie = StepImplicitEuler(dt);

  // Get the new state.
  *xtplus_ie = context->get_continuous_state_vector().CopyToVector();

  // Reset the state.
  context->set_time(t0);
  context->get_mutable_continuous_state()->SetFromVector(xt0);

  // If dt was smaller than the minimum step size, an explicit Euler step
  // was taken. Computing the error estimate using two half steps is now the
  // best bet.
  if (dt < this->get_minimum_step_size()) {
    const T half_dt = dt / 2;

    // Do one half step.
    IntegratorBase<T>::CalcTimeDerivatives(*context, derivs_.get());
    context->get_mutable_continuous_state()->SetFromVector(
        xt0 + half_dt*derivs_->CopyToVector());
    context->set_time(t0 + half_dt);

    // Do another half step.
    const VectorX<T> xtpoint5 = context->get_continuous_state_vector().
        CopyToVector();
    IntegratorBase<T>::CalcTimeDerivatives(*context, derivs_.get());
    context->get_mutable_continuous_state()->SetFromVector(
        xtpoint5 + half_dt*derivs_->CopyToVector());
    context->set_time(t0 + dt);

    // Update the error estimation ODE counts.
    num_err_est_function_evaluations_ += 2;

    // Set the "trapezoid" state.
    *xtplus_itr = context->get_continuous_state()->CopyToVector();

    // Update the statistics.
    this->UpdateStatistics(dt);
    return dt;
  }

  // The error estimation process uses the implicit trapezoid method, which
  // is defined as:
  // x(t+h) = x(t) + h/2 (f(t, x(t) + f(t+h, x(t+h))
  // x(t+h) from the implicit Euler method is presumably a good starting point.

  // The error estimate is derived as follows (thanks to Michael Sherman):
  // x*(t+h) = xᵢₑ(t+h) + O(h²)      [implicit Euler]
  //         = xₜᵣ(t+h) + O(h³)      [implicit trapezoid]
  // where x*(t+h) is the true (generally unknown) answer that we seek.
  // This implies:
  // xᵢₑ(t+h) + O(h²) = xₜᵣ(t+h) + O(h³)
  // Given that the second order term subsumes the third order one:
  // xᵢₑ(t+h) - xₜᵣ(t+h) = O(h²)
  // Therefore the difference between the implicit trapezoid solution and the
  // implicit Euler solution gives a second-order error estimate for the
  // implicit Euler result xᵢₑ.

  // Attempt to compute the implicit trapezoid solution.
  *xtplus_itr = *xtplus_ie;
  bool itr_successful = StepImplicitTrapezoid(stepped_ie, dx0, xtplus_itr);
  if (itr_successful) {
    // Reset the state to that computed by implicit Euler.
    context->set_time(t0 + dt);
    context->get_mutable_continuous_state()->SetFromVector(*xtplus_ie);

    // Update statistics.
    this->UpdateStatistics(dt);
    return dt;
  } else {
    context->set_time(t0);
    context->get_mutable_continuous_state()->SetFromVector(xt0);
    SPDLOG_DEBUG(drake::log(), "Implicit trapezoid approach FAILED with a step"
        "size that succeeded on implicit Euler; trying both again at a smaller "
        "step size.");
    return StepOnceAtMostPaired(dt/2, xtplus_ie, xtplus_itr);
  }
}

/// Takes a given step of the requested size.
template <class T>
void ImplicitEulerIntegrator<T>::DoStepOnceFixedSize(const T &dt) {
  VectorX<T> xtplus_ie, xtplus_itr;

  // Save the current state and time.
  Context<T>* context = this->get_mutable_context();

  SPDLOG_DEBUG(drake::log(), "IE DoStepOnceFixedSize(h={}) t={}",
                      dt, context->get_time());

  // Reset the error estimate.
  err_est_vec_.setZero(context->get_continuous_state()->size());

  // Take a single step.
  T t_remaining = dt;
  t_remaining -= StepOnceAtMostPaired(t_remaining, &xtplus_ie,
                                      &xtplus_itr);

  // If the single step didn't cover the entire interval, either throw an
  // exception or continue integrating.
  if (t_remaining > 0 && multistep_in_step_exactly_fixed_throws_) {
    throw std::runtime_error("ImplicitEulerIntegrator::DoStepOnceFixedSize()"
                                 " needs to take multiple substeps to attain"
                                 " the requested step size. Calling"
                                 " set_multi_step_in_step_once_fixed(true) will"
                                 " suppress this exception.");
  }

  // Compute and update the error estimate. We assume that the error estimates
  // can be summed.
  err_est_vec_ += xtplus_ie - xtplus_itr;

  // Loop until there is no time remaining.
  while (t_remaining > 0) {
    // Perform the paired step.
    t_remaining -= StepOnceAtMostPaired(t_remaining, &xtplus_ie,
                                       &xtplus_itr);

    // Compute and update the error estimate. We assume that the error estimates
    // can be summed.
    err_est_vec_ += xtplus_ie - xtplus_itr;
  }

  // Update the caller-accessible error estimate.
  this->get_mutable_error_estimate()->get_mutable_vector()->
      SetFromVector(err_est_vec_);
}

}  // namespace systems
}  // namespace drake
