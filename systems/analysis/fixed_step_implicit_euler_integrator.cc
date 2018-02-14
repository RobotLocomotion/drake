#include "drake/systems/analysis/fixed_step_implicit_euler_integrator.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>

#include "drake/common/text_logging.h"
#include "drake/math/autodiff_gradient.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a"\n" << a << std::endl;
//#define PRINT_VARn(a) (void)a;
//#define PRINT_VAR(a) (void)a;

namespace drake {
namespace systems {

template <class T>
void FixedStepImplicitEulerIntegrator<T>::DoResetStatistics() {
  num_nr_iterations_ = 0;
  num_jacobian_function_evaluations_ = 0;
  num_jacobian_evaluations_ = 0;
  num_factorizations_ = 0;
}

template <class T>
void FixedStepImplicitEulerIntegrator<T>::DoInitialize() {
  using std::isnan;

  // Allocate storage for changes to state variables during Newton-Raphson.
  //dx_state_ = this->get_system().AllocateTimeDerivatives();

  // Reset the Jacobian matrix (so that recomputation is forced).
  J_.resize(0, 0);
}

// Computes the Jacobian of the ordinary differential equations taken with
// respect to the continuous state (at a point specified by @p state) using
// automatic differentiation.
template <>
MatrixX<AutoDiffXd> FixedStepImplicitEulerIntegrator<AutoDiffXd>::
    ComputeAutoDiffJacobian(const System<AutoDiffXd>&,
                            const Context<AutoDiffXd>&) {
        throw std::runtime_error("AutoDiff'd Jacobian not supported from "
                                     "AutoDiff'd FixedStepImplicitEulerIntegrator");
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
MatrixX<T> FixedStepImplicitEulerIntegrator<T>::ComputeAutoDiffJacobian(
    const System<T>& system, const Context<T>& context) {
  SPDLOG_DEBUG(drake::log(), "  IE Compute Autodiff Jacobian t={}",
               context.get_time());
  // Create AutoDiff versions of the state vector.
  typedef AutoDiffXd Scalar;
  VectorX<Scalar> a_xtplus = context.get_continuous_state().CopyToVector();

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
  adiff_context->get_mutable_continuous_state().get_mutable_vector().
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
VectorX<T> FixedStepImplicitEulerIntegrator<T>::CalcTimeDerivativesUsingContext() {
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
MatrixX<T> FixedStepImplicitEulerIntegrator<T>::ComputeForwardDiffJacobian(
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
    // Compute a good increment to the dimension using approximately 1/eps
    // digits of precision. Note that if |xtplus| is large, the increment will
    // be large as well. If |xtplus| is small, the increment will be no smaller
    // than eps.
    const T abs_xi = abs(xtplus(i));
    T dxi(abs_xi);
    if (dxi <= 1) {
      // When |xtplus[i]| is small, increment will be eps.
      dxi = eps;
    } else {
      // |xtplus[i]| not small; make increment a fraction of |xtplus[i]|.
      dxi = eps * abs_xi;
    }

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
MatrixX<T> FixedStepImplicitEulerIntegrator<T>::ComputeCentralDiffJacobian(
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
    // Compute a good increment to the dimension using approximately 1/eps
    // digits of precision. Note that if |xtplus| is large, the increment will
    // be large as well. If |xtplus| is small, the increment will be no smaller
    // than eps.
    const T abs_xi = abs(xtplus(i));
    T dxi(abs_xi);
    if (dxi <= 1) {
      // When |xtplus[i]| is small, increment will be eps.
      dxi = eps;
    } else {
      // |xtplus[i]| not small; make increment a fraction of |xtplus[i]|.
      dxi = eps * abs_xi;
    }

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
void FixedStepImplicitEulerIntegrator<T>::Factor(const MatrixX<T>& A) {
  num_factorizations_++;
  LU_.compute(A);
}

// Factors a dense matrix (the negated iteration matrix). This
// AutoDiff-specialized method is necessary because Eigen's LU factorization,
// which should be faster than the QR factorization used here, is not currently
// AutoDiff-able (while the QR factorization *is* AutoDiff-able).
template <>
void FixedStepImplicitEulerIntegrator<AutoDiffXd>::Factor(
    const MatrixX<AutoDiffXd>& A) {
  num_factorizations_++;
  QR_.compute(A);
}

// Solves a linear system Ax = b for x using a negated iteration matrix (A)
// factored using LU decomposition.
// @sa Factor()
template <class T>
VectorX<T> FixedStepImplicitEulerIntegrator<T>::Solve(const VectorX<T>& b) const {
  return LU_.solve(b);
}

// Solves the linear system Ax = b for x using a negated iteration matrix (A)
// factored using QR decomposition.
// @sa Factor()
template <>
VectorX<AutoDiffXd> FixedStepImplicitEulerIntegrator<AutoDiffXd>::Solve(
    const VectorX<AutoDiffXd>& b) const {
  return QR_.solve(b);
}

// Checks to see whether a Jacobian matrix has "become bad" and needs to be
// refactorized.
template <class T>
bool FixedStepImplicitEulerIntegrator<T>::IsBadJacobian(const MatrixX<T>& J) const {
  return !J.allFinite();
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
bool FixedStepImplicitEulerIntegrator<T>::StepAbstract(const T& dt,
                          const std::function<VectorX<T>()>& g,
                          int scale, VectorX<T>* xtplus) {
  using std::max;
  using std::min;

  // Verify the scale factor is correct.
  DRAKE_ASSERT(scale == 1 || scale == 2);

  // Verify xtplus
  Context<T>* context = this->get_mutable_context();
  DRAKE_ASSERT(xtplus &&
               xtplus->size() == context->get_continuous_state_vector().size());

  SPDLOG_DEBUG(drake::log(), "StepAbstract() entered for t={}, h={}",
               context->get_time(), dt);

  // Advance the context time; this means that all derivatives will be computed
  // at t+dt.
  const T tf = context->get_time() + dt;
  context->set_time(tf);
  context->get_mutable_continuous_state().SetFromVector(*xtplus);

  // Evaluate the residual error using the current x(t+h) as x⁰(t+h):
  // g(x⁰(t+h)) = x⁰(t+h) - x(t) - h f(t+h,x⁰(t+h)), where h = dt;
  VectorX<T> goutput = g();

  J_ = CalcJacobian(tf, *xtplus);
  const int n = xtplus->size();
  neg_iteration_matrix_ = J_ * (dt / scale) - MatrixX<T>::Identity(n, n);
  Factor(neg_iteration_matrix_);

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
    T dx_norm = dx.norm();

    // Update the state vector.
    *xtplus += dx;

    // Update with xtplus
    context->get_mutable_continuous_state().SetFromVector(*xtplus);

    T dx_infnorm = dx.template lpNorm<Eigen::Infinity>();
    T xtplus_infnorm = xtplus->template lpNorm<Eigen::Infinity>();
    (void) dx_infnorm;
    (void) xtplus_infnorm;

    SPDLOG_DEBUG(drake::log(),
                 "NR loop: tf={}, i={}, dx_norm={}, x_norm={}",
                 tf, i, dx_infnorm, xtplus_infnorm);

    // The check below looks for convergence using machine epsilon. Without
    // this check, the convergence criteria can be applied when
    // |dx_norm| ~ 1e-22 (one example taken from practice), which does not
    // allow the norm to be reduced further. What happens: dx_norm will become
    // equivalent to last_dx_norm, making theta = 1, and eta = infinity. Thus,
    // convergence would never be identified.
    if (dx_norm < 10 * std::numeric_limits<double>::epsilon()) {
      return true;
    }

    // Simply do:
    //   |dx_i| < max(a * |x_i|, epsilon).
    if (i >= 1) {
      const double a = this->get_accuracy_in_use();

      // that is: |dx_i| < max(a * |x_i|, epsilon).
      auto absdx_i = dx.array().abs();
      auto a_times_absxi = a * xtplus->array().abs();
      auto dxi_bound = a_times_absxi.max(10 * std::numeric_limits<double>::epsilon());

      if ((absdx_i < dxi_bound).all()) {
        return true;  //converged.
      }
    }

    // Update the state in the context and compute g(xⁱ⁺¹).
    //context->get_mutable_continuous_state().SetFromVector(*xtplus);
    goutput = g();
  }

  SPDLOG_DEBUG(drake::log(), "StepAbstract() convergence failed");

  // If we are here, we failed to converge.
  return false;
}

// Steps the system forward by a single step of at most dt using the implicit
// Euler method.
// @param dt the maximum time increment to step forward.
// @returns true if successful.
template <class T>
bool FixedStepImplicitEulerIntegrator<T>::StepImplicitEuler(const T& dt) {
  using std::abs;

  // Get the current continuous state.
  Context<T>* context = this->get_mutable_context();
  const VectorX<T> xt0 = context->get_continuous_state_vector().CopyToVector();

  SPDLOG_DEBUG(drake::log(), "StepImplicitEuler(h={}) t={}",
               dt, context->get_time());

  // Set g for the implicit Euler method.
  std::function<VectorX<T>()> g =
      [&xt0, dt, context, this]() {
        return (context->get_continuous_state().CopyToVector() - xt0 -
            dt*CalcTimeDerivativesUsingContext()).eval();
      };

  // Use the current state as the candidate value for the next state.
  // [Hairer 1996] validates this choice (p. 120).
  VectorX<T> xtplus = xt0;

  // Attempt the step.
  return StepAbstract(dt, g, 1, &xtplus);
}

// Compute the partial derivative of the ordinary differential equations with
// respect to the state variables for a given x(t).
// @post the context's time and continuous state will be temporarily set during
//       this call (and then reset to their original values) on return.
template <class T>
MatrixX<T> FixedStepImplicitEulerIntegrator<T>::CalcJacobian(const T& t,
                                                    const VectorX<T>& x) {
  // We change the context but will change it back.
  Context<T>* context = this->get_mutable_context();

  // Get the current time and state.
  T t_current = context->get_time();
  const VectorX<T> x_current = context->get_continuous_state_vector().
      CopyToVector();

  // Update the time and state.
  context->set_time(t);
  context->get_mutable_continuous_state_vector().SetFromVector(x);
  num_jacobian_evaluations_++;

  // Get the current number of ODE evaluations.
  int64_t current_ODE_evals = this->get_num_derivative_evaluations();

  // Get a the system.
  const System<T>& system = this->get_system();

  // Get the mutable continuous state.
  ContinuousState<T>& continuous_state = context->
      get_mutable_continuous_state();

  // TODO(edrumwri): Give the caller the option to provide their own Jacobian.
  MatrixX<T> J;
  switch (jacobian_scheme_) {
    case JacobianComputationScheme::kForwardDifference:
      J = ComputeForwardDiffJacobian(system, *context, &continuous_state);
      break;

    case JacobianComputationScheme::kCentralDifference:
      J = ComputeCentralDiffJacobian(system, *context, &continuous_state);
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
  continuous_state.SetFromVector(x_current);

  return J;
}

#if 0
/**
 * Integrates the system forward in time by dt. This value is determined
 * by IntegratorBase::Step().
 */
template <class T>
bool FixedStepImplicitEulerIntegrator<T>::DoStep(const T& dt) {
  // Find the continuous state xc within the Context, just once.
  auto context = IntegratorBase<T>::get_mutable_context();
  VectorBase<T>& xc = context->get_mutable_continuous_state_vector();

  // TODO(sherm1) This should be calculating into the cache so that
  // Publish() doesn't have to recalculate if it wants to output derivatives.
  this->CalcTimeDerivatives(*context, derivs_.get());

  // Compute derivative and update configuration and velocity.
  // xc(t+h) = xc(t) + dt * xcdot(t, xc(t), u(t))
  const auto& xcdot = derivs_->get_vector();
  xc.PlusEqScaled(dt, xcdot);  // xc += dt * xcdot
  context->set_time(context->get_time() + dt);

  // This integrator always succeeds at taking the step.
  return true;
}
#endif

/// Takes a given step of the requested size, if possible.
/// @returns `true` if successful and `false` otherwise.
/// @post the time and continuous state will be advanced only if `true` is
///       returned.
template <class T>
bool FixedStepImplicitEulerIntegrator<T>::DoStep(const T& dt) {
  SPDLOG_DEBUG(drake::log(), "IE DoStep(h={}) t={}", dt, this->get_context().get_time());

  if (!StepImplicitEuler(dt)) {
    SPDLOG_DEBUG(drake::log(), "Implicit Euler approach did not converge for "
        "step size {}", dt);
    return false;
  }

  return true;
}

template class FixedStepImplicitEulerIntegrator<double>;
template class FixedStepImplicitEulerIntegrator<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
