#include "drake/systems/analysis/sdirk2_integrator.h"

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
Sdirk2Integrator<T>::~Sdirk2Integrator() = default;

template <class T>
std::unique_ptr<ImplicitIntegrator<T>>
Sdirk2Integrator<T>::DoImplicitIntegratorClone() const {
  return std::make_unique<Sdirk2Integrator>(this->get_system());
}

template <class T>
void Sdirk2Integrator<T>::DoResetImplicitIntegratorStatistics() {
  num_nr_iterations_ = 0;
}

template <class T>
void Sdirk2Integrator<T>::DoResetCachedJacobianRelatedMatrices() {
  iteration_matrix_ = {};
}

template <class T>
void Sdirk2Integrator<T>::DoInitialize() {
  using std::isnan;

  // Allocate storage for changes to state variables during Newton-Raphson.
  dx_state_ = this->get_system().AllocateTimeDerivatives();

  // Set default accuracy
  const double kDefaultAccuracy = 1e-4;

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
  k1_.resize(nx);
  k2_.resize(nx);
  g_.resize(nx);
  dk_.resize(nx);
  err_est_vec_.resize(nx);

  // Set the working accuracy to a reasonable default
  double working_accuracy = this->get_target_accuracy();
  if (isnan(working_accuracy)) working_accuracy = kDefaultAccuracy;
  this->set_accuracy_in_use(working_accuracy);
}

template <class T>
void Sdirk2Integrator<T>::ComputeAndFactorIterationMatrix(
    const MatrixX<T>& J, const T& h,
    typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix) {
  const int n = J.rows();
  iteration_matrix->SetAndFactorIterationMatrix(-gamma_ * h * J +
                                                MatrixX<T>::Identity(n, n));
}

template <class T>
bool Sdirk2Integrator<T>::NewtonSolve(const T& t, const T& h,
                                      const VectorX<T>& x0, VectorX<T>* k_ptr,
                                      int trial) {
  // Verify that we have a valid trial number
  DRAKE_DEMAND(trial >= 1 && trial <= 4);

  // Dereference k for convenience
  DRAKE_DEMAND(k_ptr != nullptr);
  VectorX<T>& k = *k_ptr;

  // Set time and state to (t, x₀ + γ h k)
  Context<T>* context = this->get_mutable_context();
  k.setZero();  // TODO(vincekurtz): consider using a better initial guess
  x_ = x0 + gamma_ * h * k;
  context->SetTimeAndContinuousState(t, x_);

  // Initialize the previous state update norm for convergence tests
  T last_dk_norm = std::numeric_limits<double>::infinity();

  // Update the iteration matrix A = [I - γ h J] according to the level of
  // detail requested by the `trial` parameter. If full Newton is enabled, this
  // will be computed at the beginning of the loop instead.
  if (!this->get_use_full_newton() &&
      !this->MaybeFreshenMatrices(t, x_, h, trial,
                                  ComputeAndFactorIterationMatrix,
                                  &iteration_matrix_)) {
    return false;  // early exit if factorization fails
  }

  // Do the Newton-Raphson iterations
  for (int i = 0; i < this->max_newton_raphson_iterations(); ++i) {
    num_nr_iterations_++;  // log number of newton iterations

    // Compute A = [I - γ h J]
    this->FreshenMatricesIfFullNewton(t, x_, h, ComputeAndFactorIterationMatrix,
                                      &iteration_matrix_);

    // Evaluate g(k) = k - f(t, x₀ + γ h k)
    g_ = k - this->EvalTimeDerivatives(*context).CopyToVector();

    // Compute the update using A * dk = -g
    dk_ = iteration_matrix_.Solve(-g_);
    k += dk_;

    // Compute the norm of the update
    dx_state_->get_mutable_vector().SetFromVector(dk_);
    T dk_norm = this->CalcStateChangeNorm(*dx_state_);

    // Check for convergence
    typename ImplicitIntegrator<T>::ConvergenceStatus status =
        this->CheckNewtonConvergence(i, k, dk_, dk_norm, last_dk_norm);
    // If it converged, we're done.
    if (status == ImplicitIntegrator<T>::ConvergenceStatus::kConverged) {
      return true;
    }
    // If it diverged, we have to abort and try again with a higher trial value.
    if (status == ImplicitIntegrator<T>::ConvergenceStatus::kDiverged) {
      break;
    }
    // Otherwise, continue to the next Newton-Raphson iteration.
    DRAKE_DEMAND(status ==
                 ImplicitIntegrator<T>::ConvergenceStatus::kNotConverged);

    last_dk_norm = dk_norm;

    // Update the state to prepare the next iteration
    x_ = x0 + gamma_ * h * k;
    context->SetTimeAndContinuousState(t, x_);
  }

  // If Jacobian and iteration matrix factorizations are not reused, there is
  // nothing else we can try. The solve has failed and we'll need error control
  // to reduce h.
  if (!this->get_reuse()) {
    return false;
  }

  // Try again with a higher trial number. This will be more aggressive about
  // recomputing Jacobians and matrix factorizations.
  return NewtonSolve(t, h, x0, k_ptr, trial + 1);
}

template <class T>
bool Sdirk2Integrator<T>::DoImplicitIntegratorStep(const T& h) {
  Context<T>* context = this->get_mutable_context();
  const T t0 = context->get_time();
  x0_ = context->get_continuous_state().CopyToVector();

  // First stage: solve for k₁ = f(t₀ + γh, x₀ + γhk₁)
  if (!NewtonSolve(t0 + gamma_ * h, h, x0_, &k1_)) {
    // Early exit and reset the state if the Newton-Raphson process fails
    DRAKE_LOGGER_DEBUG("SDIRK2: k1 solve failed");
    context->SetTimeAndContinuousState(t0, x0_);
    return false;
  }

  // Second stage: solve for k₂ = f(t₀ + h, x₀ + (1-γ)hk₁ + γhk₂)
  x1_ = x0_ + (1 - gamma_) * h * k1_;
  if (!NewtonSolve(t0 + h, h, x1_, &k2_)) {
    // Early exit and reset the state if the Newton-Raphson process fails
    DRAKE_LOGGER_DEBUG("SDIRK2: k2 solve failed");
    context->SetTimeAndContinuousState(t0, x0_);
    return false;
  }

  // Set the new state: x = x₀ + (1-γ)hk₁ + γhk₂
  context->SetTimeAndContinuousState(t0 + h, x1_ + gamma_ * h * k2_);

  // Set the error estimate using the lower-order embedd
  // x̂ = x₀ + (1-α)hk₁ + αhk₂
  err_est_vec_ = x0_ + (1 - alpha_) * h * k1_ + alpha_ * h * k2_;
  err_est_vec_ -= context->get_continuous_state().CopyToVector();

  this->get_mutable_error_estimate()->SetFromVector(err_est_vec_);

  return true;  // step was successful
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::systems::Sdirk2Integrator);
