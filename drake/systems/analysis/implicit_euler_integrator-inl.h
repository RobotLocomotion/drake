#pragma once

/// @file
/// Template method implementations.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include <limits>
#include <memory>
#include <utility>

#include "drake/math/autodiff.h"
#include "drake/systems/analysis/line_search.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"

namespace drake {
namespace systems {

template <class T>
void ImplicitEulerIntegrator<T>::DoResetStatistics() {
  num_nr_loops_ = 0;
  num_function_evaluations_ = 0;
  num_jacobian_function_evaluations_ = 0;
  alpha_sum_ = 0.0;
}

template <class T>
void ImplicitEulerIntegrator<T>::DoInitialize() {
  using std::isnan;

  // Allocate storage for changes to state variables during Newton-Raphson.
  dx_state_ = this->get_system().AllocateTimeDerivatives();

  const double kDefaultAccuracy = 1e-1;  // Good for this particular integrator.
  const double kLoosestAccuracy = 1e-2;  // Integrator specific.

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
template <class T>
Eigen::MatrixXd ImplicitEulerIntegrator<T>::ComputeADiffJacobianF(
    const Eigen::VectorXd& xtplus) {

  // Create AutoDiff versions of the state vector.
  typedef Eigen::AutoDiffScalar<Eigen::VectorXd> Scalar;
  VectorX<Scalar> a_xtplus = xtplus;

  // Set the size of the derivatives and prepare for Jacobian calculation.
  const int n_state_dim = xtplus.size();
  for (int i = 0; i < n_state_dim; ++i)
    a_xtplus[i].derivatives() = VectorX<T>::Unit(n_state_dim, i);

  // Get the system and the context in AutoDiffable format.
  const auto system = this->get_system().ToAutoDiffXd();
  std::unique_ptr<Context<Scalar>> context = system->AllocateContext();
  context->SetTimeStateAndParametersFrom(this->get_context());
  system->FixInputPortsFrom(this->get_system(), this->get_context(),
                            context.get());

  // Set the continuous state in the context.
  context->get_mutable_continuous_state()->get_mutable_vector()->
      SetFromVector(a_xtplus);

  // Evaluate the derivatives at that state.
  std::unique_ptr<ContinuousState<Scalar>> derivs =
      system->AllocateTimeDerivatives();
  system->CalcTimeDerivatives(*context, derivs.get());
  num_jacobian_function_evaluations_++;

  // Get the Jacobian.
  auto result = derivs->CopyToVector().eval();
  return math::autoDiffToGradientMatrix(result);
}

// Evaluates the ordinary differential equations at a given state. Permits
// counting the number of function evaluations for a given integration step.
template <class T>
VectorX<T> ImplicitEulerIntegrator<T>::CalcTimeDerivatives(
    const VectorX<T>& x) {
  const System<T>& system = this->get_system();
  Context<T>* context = this->get_mutable_context();
  context->get_mutable_continuous_state()->get_mutable_vector()->
      SetFromVector(x);
  system.CalcTimeDerivatives(*context, derivs_.get());
  num_function_evaluations_++;
  return derivs_->CopyToVector();
}

// Computes the Jacobian of the ordinary differential equations, evaluated at
// xtplus, taken with respect to the state variables, using a first-order
// central difference (i.e., numerical differentiation).
template <class T>
MatrixX<T> ImplicitEulerIntegrator<T>::ComputeFDiffJacobianF(
    const VectorX<T>& xtplus) {
  // Set epsilon to the square root of machine precision.
  double eps = std::sqrt(std::numeric_limits<double>::epsilon());

  Context<T>* context = this->get_mutable_context();
  const int n = context->get_continuous_state()->size();

  // Initialize the Jacobian.
  MatrixX<T> J(n, n);

  // Evaluate f(t+h,xtplus) for the current xtplus.
  VectorX<T> f = CalcTimeDerivatives(xtplus);
  num_jacobian_function_evaluations_++;

  // Compute the Jacobian.
  VectorX<T> xtplus_prime = xtplus;
  for (int i = 0; i < n; ++i) {
    // Compute a good increment to the dimension.
    const double abs_xi = std::abs(xtplus(i));
    double dxi = (abs_xi > 0) ? eps * abs_xi : eps;

    // Update xtplus', minimizing the effect of roundoff error.
    xtplus_prime(i) = xtplus(i) + dxi;
    dxi = xtplus_prime(i) - xtplus(i);

    // Compute f' and set the relevant column of the Jacobian matrix.
    J.col(i) = (CalcTimeDerivatives(xtplus_prime) - f)/dxi;
    num_jacobian_function_evaluations_++;

    // Reset xtplus' to xtplus.
    xtplus_prime(i) = xtplus(i);
  }

  return J;
}

// Computes the Jacobian of the ordinary differential equations, evaluated at
// xtplus, taken with respect to the state variables, using a second-order
// central difference  (i.e., numerical differentiation).
template <class T>
MatrixX<T> ImplicitEulerIntegrator<T>::ComputeCDiffJacobianF(
    const VectorX<T>& xtplus) {
  // Cube root of machine precision (indicated by theory) seems a bit coarse.
  // Pick eps halfway between square root and cube root.
  double eps = std::pow(std::numeric_limits<double>::epsilon(), 5.0/12);

  Context<T>* context = this->get_mutable_context();
  const int n = context->get_continuous_state()->size();

  // Initialize the Jacobian.
  MatrixX<T> J(n, n);

  // Compute the Jacobian.
  VectorX<T> xtplus_prime = xtplus;
  for (int i = 0; i < n; ++i) {
    // Compute a good increment to the dimension.
    const double abs_xi = std::abs(xtplus(i));
    double dxi = (abs_xi > 0) ? eps * abs_xi : eps;

    // Update xtplus', minimizing the effect of roundoff error.
    xtplus_prime(i) = xtplus(i) + dxi;
    const double dxi_plus = xtplus_prime(i) - xtplus(i);

    // Compute f(x+dx).
    VectorX<T> fprime_plus = CalcTimeDerivatives(xtplus_prime);
    num_jacobian_function_evaluations_++;

    // Update xtplus' again, minimizing the effect of roundoff error.
    xtplus_prime(i) = xtplus(i) - dxi;
    const double dxi_minus = xtplus(i) - xtplus_prime(i);

    // Compute f(x-dx).
    VectorX<T> fprime_minus = CalcTimeDerivatives(xtplus_prime);
    num_jacobian_function_evaluations_++;

    // Set the Jacobian column.
    J.col(i) = (fprime_plus - fprime_minus) / (dxi_plus + dxi_minus);

    // Reset xtplus' to xtplus.
    xtplus_prime(i) = xtplus(i);
  }

  return J;
}

// Steps forward by dt.
template <class T>
void ImplicitEulerIntegrator<T>::Step(const T &dt) {
  using std::abs;

  // Calculate the Jacobian matrix at x0.
  Context<T>* context = this->get_mutable_context();
  const VectorX<T> x0 = context->get_continuous_state_vector().CopyToVector();

  // Advance the context time; this means that all derivatives will be computed
  // at t+dt.
  context->set_time(context->get_time() + dt);

  // Get the current continuous state.
  const VectorX<T> xt = context->get_continuous_state_vector().CopyToVector();

  // Use the current state as the candidate value for the next state.
  // [Hairer 1996] validates this choice (p. 120).
  VectorX<T> xtplus = xt;
  const int n = xtplus.size();

  // Setup g
  std::function<VectorX<T>(const VectorX<T>&)> g =
      [&xt, dt, this](const VectorX<T>& x) {
         return (x - xt - dt*CalcTimeDerivatives(x)).eval();
  };

  // Evaluate g(x(t+h)) = x(t+h) - x(t) - h f(t+h,x(t+h)), where h = dt;
  VectorX<T> goutput = g(xtplus);

  // Set the initial value of the objective function f = g^Tg
  double f_last = 0.5*goutput.norm();

  // The Jacobian matrix yields the relationship J dxtplus/dt = dg/dt.
  // Converting this from a derivative into a differential yields
  // J dxtplus = dg. Setting dg \equiv -g, we now solve this linear
  // equation for dxtplus, which gives a descent direction toward
  // reducing ||g||. J will be n x n, where n is the number of state
  // variables. It will generally possess no "nice" properties (e.g.,
  // symmetry) or even be guaranteed to be invertible.
  MatrixX<T> Jg = MatrixX<T>::Identity(n, n) - (CalcJacobian(xtplus) * dt);
  LU_.compute(Jg);
  int n_loops_since_fresh_J = 0;

  // Set value of last alpha (initialized to NaN to indicate not set).
  double last_alpha = std::nan("");

  // Evaluate the objective function.
  while (f_last > convergence_tol_) {
    // Update the number of Newton-Raphson loops.
    num_nr_loops_++;

    // Compute and factorize the Jacobian matrix if necessary.
    if (last_alpha < reform_J_tol_ &&
        n_loops_since_fresh_J > reform_J_min_loops_) {
      Jg = MatrixX<T>::Identity(n, n) - (CalcJacobian(xtplus) * dt);
      LU_.compute(Jg);
      n_loops_since_fresh_J = 0;
    }

    // Compute the state update.
    VectorX<T> dx = LU_.solve(-goutput);

    // Compute the gradient of f = 0.5*g'*g, used for line search, computed with
    // respect to x. This also allows us to handle a single Jacobian matrix.
    // If grad*dx > 0, f will be non-decreasing.
    const VectorX<T> grad = goutput.transpose() * Jg;
    double slope = grad.dot(dx);
    DRAKE_DEMAND(slope < 0);

    // Search the ray α ∈ [0, 1] for a "good" value that minimizes
    // || x(t+h)ᵢ + dx α - x(t) - h f(t+h, x(t+h)ᵢ + dx α) ||. The
    // ray search is relativelly expensive, as it requres evaluating f().
    // Therefore, a good ray search will optimize the computational tradeoff
    // between finding the value of \alpha that minimizes the norm above and
    // forming the next Jacobian and solving the next linear system.
    typename LineSearch<T>::LineSearchOutput lout =
        LineSearch<T>::search_line(xtplus, dx, f_last, grad, g, goutput.norm());

    // Store the last alpha and update the alpha sum statistic.
    last_alpha = lout.alpha;
    alpha_sum_ += last_alpha;

    // The six steps below check to see whether the update is close to zero.
    // This test is necessary because the ODE function output is not scaled, and
    // the outputs of that function are expected to be large in stiff regions.

    // 1. Get the change in configuration, velocity, and auxiliary variables.
    dx_state_->get_mutable_vector()->SetFromVector(xtplus - lout.x_new);
    const VectorBase<T>& dgq = dx_state_->get_generalized_position();
    const VectorBase<T>& dgv = dx_state_->get_generalized_velocity();
    const VectorBase<T>& dgz = dx_state_->get_misc_continuous_state();

    // 2. (re-)Initialize pinvN_dq_err_ and weighted_q_err_, if necessary.
    // Reinitialization might be required if the system state variables can
    // change during the course of the integration.
    if (pinvN_ddq_ == nullptr) {
      pinvN_ddq_ = std::make_unique<BasicVector<T>>(dgv.size());
      weighted_dq_ = std::make_unique<BasicVector<T>>(dgq.size());
    }
    DRAKE_DEMAND(pinvN_ddq_->size() == dgv.size());
    DRAKE_DEMAND(weighted_dq_->size() == dgq.size());

    // TODO(edrumwri): Acquire characteristic time properly from the system
    //                 (i.e., modify the System to provide this value).
    const double characteristic_time = 1.0;

    // 3. Compute the infinity norm of the weighted velocity variables.
    const auto& qbar_v_weight = this->get_generalized_state_weight_vector();
    unweighted_delta_ = dgv.CopyToVector();
    T v_nrm = qbar_v_weight.cwiseProduct(unweighted_delta_).
        template lpNorm<Eigen::Infinity>() * characteristic_time;

    // 4. Compute the infinity norm of the weighted auxiliary variables.
    const auto& z_weight = this->get_misc_state_weight_vector();
    unweighted_delta_ = dgz.CopyToVector();
    T z_nrm = (z_weight.cwiseProduct(unweighted_delta_))
        .template lpNorm<Eigen::Infinity>();

    // 5. Compute N * Wq * dq = N * Wꝗ * N+ * dq.
    const System<T>& system = this->get_system();
    unweighted_delta_ = dgq.CopyToVector();
    system.MapQDotToVelocity(*context, unweighted_delta_, pinvN_ddq_.get());
    system.MapVelocityToQDot(
        *context, qbar_v_weight.cwiseProduct(pinvN_ddq_->CopyToVector()),
        weighted_dq_.get());
    T q_nrm = weighted_dq_->CopyToVector().template lpNorm<Eigen::Infinity>();

    // 6. Check whether update was sufficiently small that convergence is
    //    indicated.
    if (q_nrm < delta_update_tol_ && v_nrm < delta_update_tol_ &&
        z_nrm < delta_update_tol_)
      return;

    // Now check whether the change in the objective function is sufficiently
    // small *and* Jacobian is fresh.
    if (std::abs(f_last - lout.f_new) < delta_f_tol_ &&
        n_loops_since_fresh_J == 0)
      return;

    // Update x, g_output, and f
    goutput = lout.g_output;
    xtplus = lout.x_new;
    f_last = lout.f_new;
  }
}

// Compute the partial derivative of the ordinary differential equations with
// respect to the new state variables for a given xt(t+dt).
template <class T>
MatrixX<T> ImplicitEulerIntegrator<T>::CalcJacobian(const VectorX<T>& xtplus) {
  switch (jacobian_scheme_) {
    case JacobianComputationScheme::kForwardDifference:
      return ComputeFDiffJacobianF(xtplus);

    case JacobianComputationScheme::kCentralDifference:
      return ComputeCDiffJacobianF(xtplus);

    case JacobianComputationScheme::kAutomatic:
      return ComputeADiffJacobianF(xtplus);

    default:
      // Should never get here.
      DRAKE_ABORT();
  }
}

template <class T>
std::pair<bool, T> ImplicitEulerIntegrator<T>::DoStepOnceAtMost(
    const T& max_dt) {
  using std::max;

  const Context<T>& context = this->get_context();

  // Call the generic error controlled stepper unless error control or
  // error estimation is disabled.
  if (this->get_fixed_step_mode()) {
    this->get_mutable_interval_start_state() =
        context.get_continuous_state_vector().CopyToVector();
    this->DoStepOnceFixedSize(max_dt);
    return std::make_pair(true, max_dt);
  } else {
    this->StepErrorControlled(max_dt, nullptr);
    const T& dt = this->get_previous_integration_step_size();
    return std::make_pair(dt == max_dt, dt);
  }
}

/// Takes a given step of the requested size.
template <class T>
void ImplicitEulerIntegrator<T>::DoStepOnceFixedSize(const T &dt) {
  // Save the current state and time.
  Context<T>* context = this->get_mutable_context();

  // Compute the derivative at x0.
  const System<T>& system = this->get_system();
  std::unique_ptr<ContinuousState<T>> derivs = system.AllocateTimeDerivatives();
  system.CalcTimeDerivatives(*context, derivs.get());
  const VectorX<T> dx0 = derivs->CopyToVector();

  // Do the single step.
  DoTrialStep(dt);

  // Compute the derivative at xtf.
  system.CalcTimeDerivatives(*context, derivs.get());
  const VectorX<T> dxf = derivs->CopyToVector();

  // Note: this second order error estimate comes from some code from @sherm1.
  // It *seems* to give reasonable error estimates, at least compared to a
  // previous, trusted error estimation process that used two half steps.
  // TODO(edrumwri): Verify this error estimate mathematically.

  // Compute the stiff second order error estimate.
  VectorX<T> e2s = dt/2 * (dx0 - dxf);

  // Compute the non-stiff second order error estimate using the last computed
  // Jacobian matrix factorization.
  err_est_vec_ = LU_.solve(e2s);

  // Update the caller-accessible error estimate and statistics.
  this->get_mutable_error_estimate()->get_mutable_vector()->
      SetFromVector(err_est_vec_);
  this->UpdateStatistics(dt);
}

}  // namespace systems
}  // namespace drake
