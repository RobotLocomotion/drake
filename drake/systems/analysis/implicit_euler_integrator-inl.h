#pragma once

/// @file
/// Template method implementations.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include <limits>
#include <memory>
#include <utility>

#include "drake/common/text_logging.h"
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
  SPDLOG_DEBUG(drake::log(), "  IE Compute Autodiff Jacobian t={}",
               this->get_context().get_time());
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
  num_function_evaluations_++;

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
  SPDLOG_DEBUG(drake::log(), "    IE Calc Derivatives t={}",
               context->get_time());
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

  SPDLOG_DEBUG(drake::log(), "  IE Compute Forwarddiff {}-Jacobian t={}",
               n, this->get_context().get_time());

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

  SPDLOG_DEBUG(drake::log(), "  IE Compute Centraldiff {}-Jacobian t={}",
               n, this->get_context().get_time());

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

// Calculates the norm of the errors of the configuration variables, velocity
// variables, and auxiliary variables.
template <class T>
void ImplicitEulerIntegrator<T>::CalcErrorNorms(const Context<T>& context,
                                                T* q_nrm, T* v_nrm, T* z_nrm) {
  DRAKE_DEMAND(q_nrm);
  DRAKE_DEMAND(v_nrm);
  DRAKE_DEMAND(z_nrm);

  // 1. Get the changes in state.
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
  *v_nrm = qbar_v_weight.cwiseProduct(unweighted_delta_).
      template lpNorm<Eigen::Infinity>() * characteristic_time;

  // 4. Compute the infinity norm of the weighted auxiliary variables.
  const auto& z_weight = this->get_misc_state_weight_vector();
  unweighted_delta_ = dgz.CopyToVector();
  *z_nrm = (z_weight.cwiseProduct(unweighted_delta_))
      .template lpNorm<Eigen::Infinity>();

  // 5. Compute N * Wq * dq = N * Wꝗ * N+ * dq.
  const System<T>& system = this->get_system();
  unweighted_delta_ = dgq.CopyToVector();
  system.MapQDotToVelocity(context, unweighted_delta_, pinvN_ddq_.get());
  system.MapVelocityToQDot(
      context, qbar_v_weight.cwiseProduct(pinvN_ddq_->CopyToVector()),
      weighted_dq_.get());
  *q_nrm = weighted_dq_->CopyToVector().template lpNorm<Eigen::Infinity>();
}

// Computes and factorizes the iteration matrix.
template <class T>
void ImplicitEulerIntegrator<T>::CalcIterationMatrix(const VectorX<T>& xtplus,
                                                     double scale) {
  // The Jacobian matrix of g() with respect to x(t+h) yields the relationship
  // Jg dxtplus/dt = dg/dt. Converting this from a derivative into a
  // differential yields Jg dxtplus = dg. Setting dg \equiv -g, we now solve
  // this linear equation for dxtplus, which gives a descent direction toward
  // reducing ||g||. Jg will be n x n, where n is the number of state
  // variables. It will generally possess no "nice" properties (e.g.,
  // symmetry) or even be guaranteed to be invertible. This matrix Jg is
  // commonly referred to as the "iteration matrix" in literature on implicit
  // integration.
  // NOTE: We compute the negation of this iteration matrix to encourage
  //       Eigen to subtract elements from the diagonal- a O(n) operation- in
  //       place of the O(n^2) operation naively implied by:
  //       Jg = MatrixX<T>::Identity(n, n) - (CalcJacobian(xtplus) * dt);
  const int n = xtplus.size();
  Jg_ = CalcJacobian(xtplus) * scale - MatrixX<T>::Identity(n, n);
  LU_.compute(Jg_);
  n_loops_since_fresh_J_ = 0;  
}

// Performs the bulk of the stepping computation for both implicit Euler and
// implicit trapezoid method- they're very similar.
template <class T>
void ImplicitEulerIntegrator<T>::StepAbstract(const T& dt,
                          const std::function<VectorX<T>(const VectorX<T>&)>& g,
                          double scale,
                          VectorX<T>* xtplus) {
  using std::abs;

  // Advance the context time; this means that all derivatives will be computed
  // at t+dt.
  Context<T>* context = this->get_mutable_context();
  context->set_time(context->get_time() + dt);

  // Verify xtplus
  DRAKE_ASSERT(xtplus &&
               xtplus->size() == context->get_continuous_state_vector().size());

  // Get the initial state.
  VectorX<T> xt0 = context->get_continuous_state_vector().CopyToVector();

  // Evaluate g(x(t+h)) = x(t+h) - x(t) - h f(t+h,x(t+h)), where h = dt;
  VectorX<T> goutput = g(*xtplus);

  // Set the initial value of the objective function f = gᵀg
  double f_last = 0.5*goutput.norm();

  // Calculate and factorize the iteration matrix if necessary.
  const int n = xt0.size();
  if (Jg_.rows() != n || Jg_.cols() != n)
    CalcIterationMatrix(*xtplus, dt);

  // Set value of last alpha (initialized to NaN to indicate not set).
  double last_alpha = std::nan("");

  // Evaluate the objective function.
  while (true) {
    // Update the number of Newton-Raphson loops.
    num_nr_loops_++;

    // Compute the state update.
    VectorX<T> dx = LU_.solve(goutput);

    // Check whether the unscaled update is close to zero.
    T q_nrm, v_nrm, z_nrm;
    dx_state_->get_mutable_vector()->SetFromVector(dx);
    CalcErrorNorms(*context, &q_nrm, &v_nrm, &z_nrm);
    if (q_nrm < delta_update_tol_ && v_nrm < delta_update_tol_ &&
        z_nrm < delta_update_tol_)
      return;

    // Evaluate f(x+dx). If it is not smaller than 99% of f(x),
    // redetermine and refactor the Jacobian matrix.
    VectorX<T> gcand = g(*xtplus + dx);
    double f_new = 0.5 * gcand.norm();
    if (f_new > std::pow(f_last,0.75)) {
      // Resolve.
      CalcIterationMatrix(*xtplus, dt / scale);
      dx = LU_.solve(goutput);

      // Since the Jacobian is fresh, the gradient of f = 0.5*g'*g (computed
      // with respect to x), can be accurately determined and used for line
      // search.
      const VectorX<T> grad = -goutput.transpose() * Jg_;
      double slope = grad.dot(dx);
      DRAKE_DEMAND(slope < 0);

      // Search the ray α ∈ [0, 1] for a "good" value that minimizes
      // ‖ x(t+h)ᵢ + dx α - x(t) - h f(t+h, x(t+h)ᵢ + dx α) ‖. The
      // ray search is relativelly expensive, as it requres evaluating f().
      // Therefore, a good ray search will optimize the computational tradeoff
      // between finding the value of \alpha that minimizes the norm above and
      // forming the next Jacobian and solving the next linear system.
      typename LineSearch<T>::LineSearchOutput lout =
          LineSearch<T>::search_line(*xtplus, dx, f_last, grad, g,
                                     goutput.norm());

      // Store the last alpha and update the alpha sum statistic.
      last_alpha = lout.alpha;
      alpha_sum_ += last_alpha;

      // Update x, g_output, and f.
      goutput = lout.g_output;
      *xtplus = lout.x_new;
      f_last = lout.f_new;
    } else {
      // Store the last alpha and update the alpha sum statistic.
      last_alpha = 1.0;
      alpha_sum_ += last_alpha;

      // Update x, g_output, and f
      *xtplus += dx;
      f_last = f_new;
      goutput = gcand;
    }
  }
}

// Steps forward by dt using the implicit Euler method.
template <class T>
void ImplicitEulerIntegrator<T>::StepImplicitEuler(const T &dt) {
  using std::abs;

  // Get the current continuous state.
  Context<T>* context = this->get_mutable_context();
  const VectorX<T> xt0 = context->get_continuous_state_vector().CopyToVector();

  SPDLOG_DEBUG(drake::log(), "StepImplicitEuler(h={}) t={}",
               dt, context->get_time());

  // Set g
  std::function<VectorX<T>(const VectorX<T>&)> g =
      [&xt0, dt, this](const VectorX<T>& x) {
        return (x - xt0 - dt*CalcTimeDerivatives(x)).eval();
      };

  // Use the current state as the candidate value for the next state.
  // [Hairer 1996] validates this choice (p. 120).
  VectorX<T> xtplus = xt0;

  StepAbstract(dt, g, 1, &xtplus);
}

// Steps forward by dt using the implicit trapezoid method.
// @param dt the integration step
// @param dx0 the time derivatives computed at the beginning of the integration
//        step.
// @param xtplus the state computed by the implicit Euler method.
template <class T>
void ImplicitEulerIntegrator<T>::StepImplicitTrapezoid(const T& dt,
                                                       const VectorX<T>& dx0,
                                                       VectorX<T>* xtplus) {
  using std::abs;

  // Get the current continuous state.
  Context<T>* context = this->get_mutable_context();
  const VectorX<T> xt0 = context->get_continuous_state_vector().CopyToVector();

  SPDLOG_DEBUG(drake::log(), "StepImplicitTrapezoid(h={}) t={}",
               dt, context->get_time());

  // Define g(x(t+h)) ≡ x(t+h) - x(t) - h/2 (f(t,x(t)) + f(t+h,x(t+h)) and
  // evaluate it at the current x(t+h).
  std::function<VectorX<T>(const VectorX<T>&)> g =
      [&xt0, dt, &dx0, this](const VectorX<T>& x) {
        return (x - xt0 - dt/2*(dx0 + CalcTimeDerivatives(x).eval())).eval();
      };

  StepAbstract(dt, g, 2, xtplus);
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
  T t0 = context->get_time();
  const VectorX<T> xt0 = context->get_continuous_state_vector().CopyToVector();
  const System<T>& system = this->get_system();
  std::unique_ptr<ContinuousState<T>> derivs = system.AllocateTimeDerivatives();
  const VectorX<T> dx0 = CalcTimeDerivatives(xt0);

  // Do the single step.
  StepImplicitEuler(dt);

  // Get the new state.
  const VectorX<T> xtplus_euler =
      context->get_continuous_state_vector().CopyToVector();

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
  // This implies:
  // xᵢₑ(t+h) + O(h²) = xₜᵣ(t+h) + O(h³)
  // Given that the second order term subsumes the third order one:
  // xᵢₑ(t+h) - xₜᵣ(t+h) = O(h²)
  // Therefore the difference between the implicit trapezoid solution and the
  // implicit Euler solution gives a second-order error estimate for the
  // implicit Euler result xᵢₑ.

  // Compute the implicit trapezoid solution.
  VectorX<T> xtplus_trapezoid = xtplus_euler;
  StepImplicitTrapezoid(dt, dx0, &xtplus_trapezoid);

  // Reset the state to that computed by implicit Euler.
  context->set_time(t0 + dt);
  context->get_mutable_continuous_state()->SetFromVector(xtplus_euler);

  // Compute the error estimate.
  err_est_vec_ = xtplus_euler - xtplus_trapezoid;

  // Update the caller-accessible error estimate and statistics.
  this->get_mutable_error_estimate()->get_mutable_vector()->
      SetFromVector(err_est_vec_);
  this->UpdateStatistics(dt);
}

}  // namespace systems
}  // namespace drake
