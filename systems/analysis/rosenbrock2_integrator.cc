#include "drake/systems/analysis/rosenbrock2_integrator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/fmt_eigen.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace systems {

template <class T>
Rosenbrock2Integrator<T>::~Rosenbrock2Integrator() = default;

template <class T>
std::unique_ptr<ImplicitIntegrator<T>>
Rosenbrock2Integrator<T>::DoImplicitIntegratorClone() const {
  return std::make_unique<Rosenbrock2Integrator>(this->get_system());
}

template <class T>
void Rosenbrock2Integrator<T>::DoInitialize() {
  using std::isnan;

  // Rosenbrock integrators generally perform well at low accuracies
  const double kDefaultAccuracy = 1e-1;

  // Set an artificial step size target, if not set already.
  if (isnan(this->get_initial_step_size_target())) {
    // Verify that maximum step size has been set.
    if (isnan(this->get_maximum_step_size()))
      throw std::logic_error(
          "Neither initial step size target nor maximum "
          "step size has been set!");

    this->request_initial_step_size_target(this->get_maximum_step_size());
  }

  // Allocate intermediate variables
  const int nx = this->get_system().num_continuous_states();
  x0_.resize(nx);
  x_.resize(nx);
  xdot_.resize(nx);
  k1_.resize(nx);
  k2_.resize(nx);
  error_est_vec_.resize(nx);

  // Set the working accuracy to a reasonable default
  double working_accuracy = this->get_target_accuracy();
  if (isnan(working_accuracy)) working_accuracy = kDefaultAccuracy;
  this->set_accuracy_in_use(working_accuracy);
}

template <class T>
void Rosenbrock2Integrator<T>::ComputeAndFactorIterationMatrix(
    const MatrixX<T>& J, const T& h,
    typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix) {
  using std::sqrt;
  const int n = J.rows();
  iteration_matrix->SetAndFactorIterationMatrix(
      -J + MatrixX<T>::Identity(n, n) / (h * params_.gamma));
}

template <class T>
bool Rosenbrock2Integrator<T>::ExplicitEulerFallbackStep(const T& h) {
  Context<T>* context = this->get_mutable_context();
  const T t0 = context->get_time();
  x0_ = context->get_continuous_state().CopyToVector();

  // Forward step with x = x₀ + h * f(t₀, x₀)
  xdot_ = this->EvalTimeDerivatives(*context).CopyToVector();
  x_ = x0_ + h * xdot_;

  // We'll do error estimation with two half-steps
  error_est_vec_ = x0_ + 0.5 * h * xdot_;
  context->SetTimeAndContinuousState(t0 + 0.5 * h, error_est_vec_);
  xdot_ = this->EvalTimeDerivatives(*context).CopyToVector();
  error_est_vec_ += 0.5 * h * xdot_;
  error_est_vec_ -= x_;

  this->get_mutable_error_estimate()->SetFromVector(error_est_vec_);

  // Set the time and state in the context
  context->SetTimeAndContinuousState(t0 + h, x_);

  return true;  // Explicit euler always succeeds
}

template <class T>
bool Rosenbrock2Integrator<T>::DoImplicitIntegratorStep(const T& h) {
  using std::sqrt;

  // Store the current time and state
  Context<T>* context = this->get_mutable_context();
  const T t0 = context->get_time();
  x0_ = context->get_continuous_state().CopyToVector();

  // If the requested step size is less than the minimum step size, we'll take a
  // single explicit Euler step instead.
  if (h < this->get_working_minimum_step_size()) {
    DRAKE_LOGGER_DEBUG(
        "-- requested step too small, taking explicit Euler step instead");
    return ExplicitEulerFallbackStep(h);
  }

  // Make sure everything is the correct size
  const int n = x0_.size();
  x_.resize(n);
  xdot_.resize(n);
  k1_.resize(n);
  k2_.resize(n);
  error_est_vec_.resize(n);

  // Compute and factor the iteration matrix G = [I/(hγ) − J], where J = ∂/∂x
  // f(t₀, x₀). Here using trial = 3 forces the Jacobian and factorization to be
  // recomputed.
  if (!this->MaybeFreshenMatrices(
          t0, x0_, h, 3, ComputeAndFactorIterationMatrix, &iteration_matrix_)) {
    // If factorization fails, reject the step so that error control selects a
    // smaller h.
    return false;
  }

  // Compute the first intermediate value k₁, where G k₁ = f(t₀, x₀)
  xdot_ = this->EvalTimeDerivatives(*context).CopyToVector();
  k1_ = iteration_matrix_.Solve(xdot_);

  // Advance the time and state:
  //    t = t₀ + h
  //    x = x₀ + a k₁
  context->SetTime(t0 + h);
  x_ = x0_ + params_.a * k1_;
  context->get_mutable_continuous_state().SetFromVector(x_);

  // Compute the second intermediate value k₂, where G k₂ = f(t, x) + c/h k₁
  xdot_ = this->EvalTimeDerivatives(*context).CopyToVector();
  xdot_ += (params_.c / h) * k1_;
  k2_ = iteration_matrix_.Solve(xdot_);

  // Advance the state as x = x₀ + m₁ k₁ + m₁ k₂
  x_ = x0_ + params_.m1 * k1_ + params_.m2 * k2_;
  context->get_mutable_continuous_state().SetFromVector(x_);

  // Compute the embedded error estimate with x̂ = x₀ + m̂₁ k₁
  // (N.B. m̂₂ = 0 for this integrator)
  error_est_vec_ = x0_ + params_.m_hat1 * k1_ - x_;
  this->get_mutable_error_estimate()->SetFromVector(error_est_vec_);

  return true;  // step was successful
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::systems::Rosenbrock2Integrator);
