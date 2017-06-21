#pragma once

/// @file
/// Template method implementations for implicit_euler_integrator.h.
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
  num_nr_iterations_ = num_err_est_nr_iterations_ = 0;
  num_err_est_function_evaluations_ = 0;
  num_jacobian_function_evaluations_ =
    num_err_est_jacobian_function_evaluations_ = 0;
  num_jacobian_evaluations_ = num_err_est_jacobian_reforms_ = 0;
  num_iter_factorizations_ = num_err_est_iter_factorizations_ = 0;
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
  J_.resize(0, 0);
}

// Computes the Jacobian of the ordinary differential equations taken with
// respect to the continuous state (at a point specified by @p state) using
// automatic differentiation.
template <>
MatrixX<AutoDiffXd> ImplicitEulerIntegrator<AutoDiffXd>::
    ComputeAutoDiffJacobian(const System<AutoDiffXd>&,
                            const Context<AutoDiffXd>&) {
        throw std::runtime_error("AutoDiff'd Jacobian not supported from "
                                     "AutoDiff'd ImplicitEulerIntegrator");
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
MatrixX<T> ImplicitEulerIntegrator<T>::ComputeAutoDiffJacobian(
    const System<T>& system, const Context<T>& context) {
  SPDLOG_DEBUG(drake::log(), "  IE Compute Autodiff Jacobian t={}",
               context.get_time());
  // Create AutoDiff versions of the state vector.
  typedef AutoDiffXd Scalar;
  VectorX<Scalar> a_xtplus = context.get_continuous_state()->CopyToVector();

  // Set the size of the derivatives and prepare for Jacobian calculation.
  const int n_state_dim = a_xtplus.size();
  for (int i = 0; i < n_state_dim; ++i)
    a_xtplus[i].derivatives() = VectorX<T>::Unit(n_state_dim, i);

  // Get the system and the context in AutoDiffable format. Inputs must also
  // be copied to the context used by the AutoDiff'd system (which is
  // accomplished using FixInputPortsFrom()).
  // TODO(edrumwri): Investigate means for moving as many of the operations
  //                 below offline (or with lower frequency than once-per-
  //                 Jacobian calculation) as is possible. These operations
  //                 are likely to be expensive.
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
  this->CalcTimeDerivatives(*adiff_system, *adiff_context, derivs.get());

  // Get the Jacobian.
  auto result = derivs->CopyToVector().eval();
  return math::autoDiffToGradientMatrix(result);
}

// Evaluates the ordinary differential equations at the time and state in
// the system's context (stored by the integrator).
template <class T>
VectorX<T> ImplicitEulerIntegrator<T>::CalcTimeDerivativesUsingContext() {
    this->CalcTimeDerivatives(this->get_context(), derivs_.get());
    return derivs_->CopyToVector();
}

// Computes the Jacobian of the ordinary differential equations taken with
// respect to the continuous state (at a point specified by @p state) using
// a first-order forward difference (i.e., numerical differentiation).
// @param system The dynamical system.
// @param context The context at which to compute the time derivatives.
// @param state The continuous state at which to compute the time derivatives.
//              The function can modify this continuous state during the
//              Jacobian computation.
// @post The continuous state will be indeterminate on return.
template <class T>
MatrixX<T> ImplicitEulerIntegrator<T>::ComputeForwardDiffJacobian(
    const System<T>&, const Context<T>& context, ContinuousState<T>* state) {
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

  // Prevent compiler warnings for context.
  unused(context);

  // Initialize the Jacobian.
  MatrixX<T> J(n, n);

  // Evaluate f(t+h,xtplus) for the current state (current xtplus).
  VectorX<T> f = CalcTimeDerivativesUsingContext();

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
    J.col(i) = (CalcTimeDerivativesUsingContext() - f) / dxi;

    // Reset xtplus' to xtplus.
    xtplus_prime(i) = xtplus(i);
  }

  return J;
}

// Computes the Jacobian of the ordinary differential equations taken with
// respect to the continuous state (at a point specified by @p state) using
// a second-order central difference (i.e., numerical differentiation).
// @param system The dynamical system.
// @param context The context at which to compute the time derivatives.
// @param state The continuous state at which to compute the time derivatives.
//              The function can modify this continuous state during the
//              Jacobian computation.
// @post The continuous state will be indeterminate on return.
template <class T>
MatrixX<T> ImplicitEulerIntegrator<T>::ComputeCentralDiffJacobian(
    const System<T>&, const Context<T>& context, ContinuousState<T>* state) {
  using std::abs;

  // Cube root of machine precision (indicated by theory) seems a bit coarse.
  // Pick power of eps halfway between 6/12 (i.e., 1/2) and 4/12 (i.e., 1/3).
  const double eps = std::pow(std::numeric_limits<double>::epsilon(), 5.0/12);

  // Get the number of state variables.
  const int n = state->size();

  SPDLOG_DEBUG(drake::log(), "  IE Compute Centraldiff {}-Jacobian t={}",
               n, context.get_time());

  // Prevent compiler warnings for context.
  unused(context);

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
    VectorX<T> fprime_plus = CalcTimeDerivativesUsingContext();

    // Update xtplus' again, minimizing the effect of roundoff error.
    xtplus_prime(i) = xtplus(i) - dxi;
    const T dxi_minus = xtplus(i) - xtplus_prime(i);

    // Compute f(x-dx).
    state->SetFromVector(xtplus_prime);
    VectorX<T> fprime_minus = CalcTimeDerivativesUsingContext();

    // Set the Jacobian column.
    J.col(i) = (fprime_plus - fprime_minus) / (dxi_plus + dxi_minus);

    // Reset xtplus' to xtplus.
    xtplus_prime(i) = xtplus(i);
  }

  return J;
}

// Factors a dense matrix (the negated iteration matrix) using LU factorization,
// which should be faster than the QR factorization used in the specialized
// template method immediately below.
template <class T>
void ImplicitEulerIntegrator<T>::Factor(const MatrixX<T>& A) {
  num_iter_factorizations_++;
  LU_.compute(A);
}

// Factors a dense matrix (the negated iteration matrix). This
// AutoDiff-specialized method is necessary because Eigen's LU factorization,
// which should be faster than the QR factorization used here, is not currently
// AutoDiff-able (while the QR factorization *is* AutoDiff-able).
template <>
void ImplicitEulerIntegrator<AutoDiffXd>::Factor(
    const MatrixX<AutoDiffXd>& A) {
  num_iter_factorizations_++;
  QR_.compute(A);
}

// Solves a linear system Ax = b for x using a negated iteration matrix (A)
// factored using LU decomposition.
// @sa Factor()
template <class T>
VectorX<T> ImplicitEulerIntegrator<T>::Solve(const VectorX<T>& b) const {
  return LU_.solve(b);
}

// Solves the linear system Ax = b for x using a negated iteration matrix (A)
// factored using QR decomposition.
// @sa Factor()
template <>
VectorX<AutoDiffXd> ImplicitEulerIntegrator<AutoDiffXd>::Solve(
    const VectorX<AutoDiffXd>& b) const {
  return QR_.solve(b);
}

// Computes any necessary matrices for the Newton-Raphson iteration in
// StepAbstract(). Parameters are identical to those for StepAbstract;
// @see StepAbstract() for their documentation.
// @returns `false` if the calling StepAbstract method should indicate failure;
//          `true` otherwise.
template <class T>
bool ImplicitEulerIntegrator<T>::CalcMatrices(const T& tf, const T& dt,
                                              int scale,
                                              const VectorX<T>& xtplus,
                                              int trial) {
  // Compute the initial Jacobian and negated iteration matrices (see
  // rationale for the negation below) and factor them, if necessary.
  if (!reuse_ || J_.rows() == 0) {
    J_ = CalcJacobian(tf, xtplus);
    const int n = xtplus.size();
    neg_iteration_matrix_ = J_ * (dt / scale) - MatrixX<T>::Identity(n, n);
    Factor(neg_iteration_matrix_);
    return true;
  }

  switch (trial) {
    case 1:
      // For the first trial, we do nothing special.
      return true;

    case 2: {
      // For the second trial, re-construct and factor the iteration matrix.
      const int n = xtplus.size();
      neg_iteration_matrix_ = J_ * (dt / scale) - MatrixX<T>::Identity(n, n);
      Factor(neg_iteration_matrix_);
      return true;
    }

    case 3: {
      // If the last call to StepAbstract() ended in failure, we know that
      // the Jacobian matrix is fresh and the iteration matrix has been newly
      // formed and factored (on Trial #2), so there is nothing more to be
      // done.
      if (last_call_failed_) {
        return false;
      } else {
        // Reform the Jacobian matrix and refactor the negation of
        // the iteration matrix. The idea of using the negation of this matrix
        // is that an O(n^2) subtraction is not necessary as would
        // be the case with MatrixX<T>::Identity(n, n) - J * (dt / scale).
        J_ = CalcJacobian(tf, xtplus);
        const int n = xtplus.size();
        neg_iteration_matrix_ = J_ * (dt / scale) -
            MatrixX<T>::Identity(n, n);
        Factor(neg_iteration_matrix_);
      }
      return true;

      case 4: {
        // Trial #4 indicates failure.
        return false;
      }

      default:
        DRAKE_ABORT_MSG("Unexpected trial number.");
    }
  }
}

// Performs the bulk of the stepping computation for both implicit Euler and
// implicit trapezoid method; all those methods need to do is provide a
// residual function (@p g) and a scale factor (@p scale) specific to the
// particular integrator scheme and this method does the rest.
// @param dt the integration step size to attempt.
// @param g the particular implicit function to zero.
// @param scale a scale factor- either 1 or 2- that allows this method to be
//        used by both implicit Euler and implicit trapezoid methods.
// @param [in,out] the starting guess for x(t+dt); the value for x(t+h) on
//        return (assuming that h > 0)
// @param trial the attempt for this approach (1-4). StepAbstract() uses more
//        computationally expensive methods as the trial numbers increase.
// @returns `true` if the method was successfully able to take an integration
//           step of size @p dt (or `false` otherwise).
// @pre The time and state of the system's context (stored by the integrator)
//      are t0 and x(t0) on entry.
// @post The time and state of the system's context (stored by the integrator)
//       will be set to t0+dt and x(t0+dt) on successful exit (indicated by
//       this function returning `true`) and will be indeterminate on
//       unsuccessful exit (indicated by this function returning `false`).
template <class T>
bool ImplicitEulerIntegrator<T>::StepAbstract(const T& dt,
                          const std::function<VectorX<T>()>& g,
                          int scale, VectorX<T>* xtplus, int trial) {
  using std::max;
  using std::min;

  // Verify the trial number is valid.
  DRAKE_ASSERT(trial >= 1 && trial <= 4);

  // Verify the scale factor is correct.
  DRAKE_ASSERT(scale == 1 || scale == 2);

  // Verify xtplus
  Context<T>* context = this->get_mutable_context();
  DRAKE_ASSERT(xtplus &&
               xtplus->size() == context->get_continuous_state_vector().size());

  // Get the initial state.
  VectorX<T> xt0 = context->get_continuous_state_vector().CopyToVector();

  SPDLOG_DEBUG(drake::log(), "StepAbstract() entered for t={}, h={}",
               context->get_time(), dt);

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

  // Calculate Jacobian and iteration matrices (and factorizations), as needed.
  if (!CalcMatrices(tf, dt, scale, *xtplus, trial)) {
    last_call_failed_ = true;
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
    // iteration matrix. Using nA as the negation of the iteration matrix, we
    // instead solve nA*x = g().
    // TODO(edrumwri): Allow caller to provide their own solver.
    VectorX<T> dx = Solve(goutput);

    // Get the infinity norm of the weighted update vector.
    dx_state_->get_mutable_vector()->SetFromVector(dx);
    T dx_norm = this->CalcStateChangeNorm(*dx_state_);

    // Update the state vector.
    *xtplus += dx;

    // The check below looks for convergence using machine epsilon. Without
    // this check, the convergence criteria can be applied when
    // |dx_norm| ~ 1e-22 (one example taken from practice), which does not
    // allow the norm to be reduced further. What happens: dx_norm will become
    // equivalent to last_dx_norm, making theta = 1, and eta = infinity. Thus,
    // convergence would never be identified.
    if (dx_norm < 10 * std::numeric_limits<double>::epsilon()) {
      context->get_mutable_continuous_state()->SetFromVector(*xtplus);
      last_call_failed_ = false;
      return true;
    }

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
      SPDLOG_DEBUG(drake::log(), "Newton-Raphson loop {} theta: {}, eta: {}",
                   i, theta, eta);

      // Look for divergence.
      if (theta > 1)
        break;

      // Look for convergence using Equation 8.10 from [Hairer, 1996].
      const double k_dot_tol = kappa * this->get_accuracy_in_use();
      if (eta * dx_norm < k_dot_tol) {
        SPDLOG_DEBUG(drake::log(), "Newton-Raphson converged; η = {}, h = {}",
                     eta, dt);
        context->get_mutable_continuous_state()->SetFromVector(*xtplus);
        last_call_failed_ = false;
        return true;
      }
    }

    // Update the norm of the state update.
    last_dx_norm = dx_norm;

    // Update the state in the context and compute g(xⁱ⁺¹).
    context->get_mutable_continuous_state()->SetFromVector(*xtplus);
    goutput = g();
  }

  SPDLOG_DEBUG(drake::log(), "StepAbstract() convergence failed");

  // If Jacobian and iteration matrix factorizations are not reused, there
  // is nothing else we can try.
  if (!reuse_) {
    last_call_failed_ = true;
    return false;
  }

  // Try StepAbstract again, freshening Jacobians and iteration matrix
  // factorizations as helpful.
  return StepAbstract(dt, g, scale, xtplus, trial+1);
}

// Steps the system forward by a single step of at most dt using the implicit
// Euler method.
// @param dt the maximum time increment to step forward.
// @returns the amount actually stepped forward by a single step.
template <class T>
bool ImplicitEulerIntegrator<T>::StepImplicitEuler(const T& dt) {
  using std::abs;

  // Get the current continuous state.
  Context<T>* context = this->get_mutable_context();
  const VectorX<T> xt0 = context->get_continuous_state_vector().CopyToVector();

  SPDLOG_DEBUG(drake::log(), "StepImplicitEuler(h={}) t={}",
               dt, context->get_time());

  // Set g for the implicit Euler method.
  std::function<VectorX<T>()> g =
      [&xt0, dt, context, this]() {
        return (context->get_continuous_state()->CopyToVector() - xt0 -
            dt*CalcTimeDerivativesUsingContext()).eval();
      };

  // Use the current state as the candidate value for the next state.
  // [Hairer 1996] validates this choice (p. 120).
  VectorX<T> xtplus = xt0;

  // Attempt the step.
  return StepAbstract(dt, g, 1, &xtplus);
}

// Steps forward by a single step of @p dt using the implicit trapezoid
// method, if possible.
// @param dt the maximum time increment to step forward.
// @param dx0 the time derivatives computed at the beginning of the integration
//        step.
// @param xtplus the state computed by the implicit Euler method.
// @returns `true` if the step was successful and `false` otherwise.
// @pre The time and state in the system's context (stored within the
//      integrator) are set to those at t0 (the beginning of the integration
//      step).
// @post The time and state in the system's context (stored within the
//       integrator) are set to those at t0+dt on successful return (i.e., when
//       the function returns `true`). State will be indeterminate on `false`
//       return.
template <class T>
bool ImplicitEulerIntegrator<T>::StepImplicitTrapezoid(const T& dt,
                                                       const VectorX<T>& dx0,
                                                       VectorX<T>* xtplus) {
  using std::abs;

  // Get the current continuous state.
  Context<T>* context = this->get_mutable_context();
  const VectorX<T> xt0 = context->get_continuous_state_vector().CopyToVector();

  SPDLOG_DEBUG(drake::log(), "StepImplicitTrapezoid(h={}) t={}",
               dt, context->get_time());

  // Set g for the implicit trapezoid method.
  // Define g(x(t+h)) ≡ x(t+h) - x(t) - h/2 (f(t,x(t)) + f(t+h,x(t+h)) and
  // evaluate it at the current x(t+h).
  std::function<VectorX<T>()> g =
      [&xt0, dt, &dx0, context, this]() {
        return (context->get_continuous_state()->CopyToVector() - xt0 -
            dt/2*(dx0 + CalcTimeDerivativesUsingContext().eval())).eval();
      };

  // Store statistics before calling StepAbstract(). The difference between
  // the modified statistics and the stored statistics will be used to compute
  // the trapezoid method-specific statistics.
  int stored_num_jacobian_evaluations = num_jacobian_evaluations_;
  int stored_num_iter_factorizations = num_iter_factorizations_;
  int64_t stored_num_function_evaluations =
      this->get_num_derivative_evaluations();
  int64_t stored_num_jacobian_function_evaluations =
      num_jacobian_function_evaluations_;
  int stored_num_nr_iterations = num_nr_iterations_;

  // Step.
  bool success = StepAbstract(dt, g, 2, xtplus);

  // Move statistics to implicit trapezoid-specific.
  num_err_est_jacobian_reforms_ +=
      num_jacobian_evaluations_ - stored_num_jacobian_evaluations;
  num_err_est_iter_factorizations_ += num_iter_factorizations_ -
      stored_num_iter_factorizations;
  num_err_est_function_evaluations_ +=
      this->get_num_derivative_evaluations() - stored_num_function_evaluations;
  num_err_est_jacobian_function_evaluations_ +=
      num_jacobian_function_evaluations_ -
          stored_num_jacobian_function_evaluations;
  num_err_est_nr_iterations_ += num_nr_iterations_ - stored_num_nr_iterations;

  return success;
}

// Compute the partial derivative of the ordinary differential equations with
// respect to the state variables for a given x(t).
// @post the context's time and continuous state will be temporarily set during
//       this call (and then reset to their original values) on return.
template <class T>
MatrixX<T> ImplicitEulerIntegrator<T>::CalcJacobian(const T& t,
                                                    const VectorX<T>& x) {
  // We change the context but will change it back.
  Context<T>* context = this->get_mutable_context();

  // Get the current time and state.
  T t_current = context->get_time();
  const VectorX<T> x_current = context->get_continuous_state_vector().
      CopyToVector();

  // Update the time and state.
  context->set_time(t);
  context->get_mutable_continuous_state_vector()->SetFromVector(x);
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
      J = ComputeAutoDiffJacobian(system, *context);
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

// Steps both implicit Euler and implicit trapezoid forward by dt, if possible.
// @param dt the integration step size to attempt.
// @param [out] xtplus_ie contains the Euler integrator solution on return
// @param [out] xtplus_itr contains the implicit trapezoid solution on return
// @returns `true` if the integration was successful at the requested step size
//          and `false` otherwise.
// @pre The time and state in the system's context (stored by the integrator)
//      are set to {t0,x0} on entry (those at the beginning of the interval.
// @post The time and state of the system's context (stored by the integrator)
//       will be set to t0+dt and @p xtplus_ie on successful exit (indicated by
//       this function returning `true`) and will be indeterminate on
//       unsuccessful exit (indicated by this function returning `false`).
template <class T>
bool ImplicitEulerIntegrator<T>::AttemptStepPaired(const T& dt,
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

  // Compute the derivative at xt0. NOTE: the derivative is calculated at this
  // point (early on in the integration process) in order to reuse the
  // derivative evaluation, via the cache, from the last integration step (if
  // possible).
  const VectorX<T> dx0 = CalcTimeDerivativesUsingContext();

  // Do the Euler step.
  if (!StepImplicitEuler(dt)) {
    SPDLOG_DEBUG(drake::log(), "Implicit Euler approach did not converge for "
        "step size {}", dt);
    return false;
  }

  // Get the new state.
  *xtplus_ie = context->get_continuous_state_vector().CopyToVector();

  // Reset the state.
  context->set_time(t0);
  context->get_mutable_continuous_state()->SetFromVector(xt0);

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
  if (StepImplicitTrapezoid(dt, dx0, xtplus_itr)) {
    // Reset the state to that computed by implicit Euler.
    // TODO(edrumwri): Explore using the implicit trapezoid method solution
    //                 instead as *the* solution, rather than the implicit
    //                 Euler. Refer to [Lambert, 1991], Ch 6.
    context->set_time(t0 + dt);
    context->get_mutable_continuous_state()->SetFromVector(*xtplus_ie);
    return true;
  } else {
    SPDLOG_DEBUG(drake::log(), "Implicit trapezoid approach FAILED with a step"
        "size that succeeded on implicit Euler.");
    return false;
  }
}

/// Takes a given step of the requested size, if possible.
/// @returns `true` if successful and `false` otherwise.
/// @post the time and continuous state will be advanced only if `true` is
///       returned.
template <class T>
bool ImplicitEulerIntegrator<T>::DoStep(const T& dt) {
  VectorX<T> xtplus_ie, xtplus_itr;

  // Save the current time and state.
  Context<T>* context = this->get_mutable_context();
  const T t0 = context->get_time();
  const VectorX<T> xt0 = context->get_continuous_state()->CopyToVector();

  SPDLOG_DEBUG(drake::log(), "IE DoStep(h={}) t={}", dt, t0);

  // If the requested dt is less than or equal to the minimum step size, an
  // explicit Euler step will be taken. We compute the error estimate using two
  // half steps.
  if (dt < this->get_working_minimum_step_size()) {
    SPDLOG_DEBUG(drake::log(), "-- requested step too small, taking explicit "
        "step instead");
    const T half_dt = dt / 2;

    // TODO(edrumwri): Investigate replacing this with an explicit trapezoid
    //                 step, which would be expected to give better accuracy.
    //                 The mitigating factor is that dt is already small, so a
    //                 test of, e.g., a square wave function, should quantify
    //                 the improvement (if any).

    // The error estimation process for explicit Euler uses two half-steps
    // of explicit Euler (for a total of two derivative evaluations). The error
    // estimation process is derived as follows:
    // (1) x*(t+h) = xₑ(t+h) + h²/2 df/dt + ...                [full step]
    // (2) x*(t+h) = ̅xₑ(t+h) + h²/8 (df/dt + df₊/dt) + ...    [two 1/2-steps]
    //
    // where x*(t+h) is the true (generally unknown) answer that we seek,
    // f() is the ordinary differential equation evaluated at x(t), and
    // f₊() is the derivative evaluated at x(t + h/2). Subtracting (1) from
    // (2), the above equations are rewritten as:
    // 0 = x̅ₑ(t+h) - xₑ(t+h) + h²/8 (-3df/dt + df₊/dt) + ...
    // The sum of all but the first two terms on the right hand side
    // of the above equation is less in magnitude than ch², for some
    // sufficiently large c. Or, written using Big-Oh notation:
    // x̅ₑ(t+h) - xₑ(t+h) = O(h²)
    // Thus, subtracting the two solutions yields a second order error estimate
    // (we compute norms on the error estimate, so the apparent sign error
    // in the algebra when arriving at the final equation is inconsequential).

    // Compute the Euler step.
    this->CalcTimeDerivatives(*context, derivs_.get());
    xtplus_ie = xt0 + dt*derivs_->CopyToVector();

    // Do one half step.
    context->get_mutable_continuous_state()->SetFromVector(
        xt0 + half_dt*derivs_->CopyToVector());
    context->set_time(t0 + half_dt);

    // Do another half step.
    const VectorX<T> xtpoint5 = context->get_continuous_state_vector().
        CopyToVector();
    this->CalcTimeDerivatives(*context, derivs_.get());
    context->get_mutable_continuous_state()->SetFromVector(
        xtpoint5 + half_dt*derivs_->CopyToVector());
    context->set_time(t0 + dt);

    // Update the error estimation ODE counts.
    num_err_est_function_evaluations_ += 2;

    // Set the "trapezoid" state to be the result of taking two explicit Euler
    // half steps. since the code below the if/then/else block
    // simply subtracts the two results to obtain the error estimate.
    xtplus_itr = context->get_continuous_state()->CopyToVector();
  } else {
    // Try taking the requested step.
    bool success = AttemptStepPaired(dt, &xtplus_ie, &xtplus_itr);

    // If the step was not successful, reset the time and state.
    if (!success) {
      context->set_time(t0);
      context->get_mutable_continuous_state()->SetFromVector(xt0);
      return false;
    }
  }

  // Reset the error estimate.
  err_est_vec_.setZero(context->get_continuous_state()->size());

  // Compute and update the error estimate. We assume that the error estimates
  // can be summed.
  err_est_vec_ += xtplus_ie - xtplus_itr;

  // Update the caller-accessible error estimate.
  this->get_mutable_error_estimate()->get_mutable_vector()->
      SetFromVector(err_est_vec_);

  return true;
}

}  // namespace systems
}  // namespace drake
