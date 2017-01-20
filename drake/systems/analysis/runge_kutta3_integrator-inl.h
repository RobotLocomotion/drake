#pragma once

/// @file
/// Template method implementations for runge_kutta_3_integrator.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include <utility>

#include "drake/systems/analysis/runge_kutta3_integrator.h"

namespace drake {
namespace systems {

/**
 * RK3-specific initialization function.
 * @throws std::logic_error if *neither* the initial step size target nor the
 *           maximum step size have been set before calling.
 */
template <class T>
void RungeKutta3Integrator<T>::DoInitialize() {
  using std::isnan;
  const double kDefaultAccuracy = 1e-3;  // Good for this particular integrator.
  const double kLoosestAccuracy = 1e-1;  // Integrator specific.
  const double kMaxStepFraction = 0.1;   // Fraction of max step size for
                                         // less aggressive first step.

  // Set an artificial step size target, if not set already.
  if (isnan(this->get_initial_step_size_target())) {
    // Verify that maximum step size has been set.
    if (isnan(this->get_maximum_step_size()))
      throw std::logic_error("Neither initial step size target nor maximum "
                                 "step size has been set!");

    this->request_initial_step_size_target(
        this->get_maximum_step_size() * kMaxStepFraction);
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

/**
 * RK3-specific stepping function.
 */
template <class T>
std::pair<bool, T> RungeKutta3Integrator<T>::DoStepOnceAtMost(const T& max_dt) {
  using std::isnan;

  // TODO(edrumwri): move derivative evaluation at t0 to the simulator where
  // it can be used for efficient guard function zero finding.
  auto& system = this->get_system();
  auto& context = this->get_context();
  system.CalcTimeDerivatives(context, derivs0_.get());

  // Call the generic error controlled stepper unless error control is
  // disabled.
  if (this->get_fixed_step_mode()) {
    this->get_mutable_interval_start_state() =
        context.get_continuous_state_vector().CopyToVector();
    this->DoStepOnceFixedSize(max_dt);
    return std::make_pair(true, max_dt);
  } else {
    this->StepErrorControlled(max_dt, derivs0_.get());
    const T& dt = this->get_previous_integration_step_size();
    return std::make_pair(dt == max_dt, dt);
  }
}

template <class T>
void RungeKutta3Integrator<T>::DoStepOnceFixedSize(const T& dt) {
  using std::abs;

  // Find the continuous state xc within the Context, just once.
  VectorBase<T>* xc = this->get_mutable_context()
                          ->get_mutable_continuous_state_vector();

  // Setup ta and tb.
  T ta = this->get_context().get_time();
  T tb = ta + dt;

  // Get the derivative at the current state (x0) and time (t0).
  const auto& xcdot0 = derivs0_->get_vector();

  // Get the system and the context.
  const auto& system = this->get_system();
  auto& context = this->get_context();

  // Compute the first intermediate state and derivative (at t=0.5, x(0.5)).
  this->get_mutable_context()->set_time(ta + dt * 0.5);
  xc->PlusEqScaled(dt * 0.5, xcdot0);
  system.CalcTimeDerivatives(context, derivs1_.get());
  const auto& xcdot1 = derivs1_->get_vector();

  // Compute the second intermediate state and derivative (at t=1, x(1)).
  this->get_mutable_context()->set_time(tb);
  xc->SetFromVector(this->get_interval_start_state());
  xc->PlusEqScaled({{-dt, xcdot0}, {dt * 2, xcdot1}});
  system.CalcTimeDerivatives(context, derivs2_.get());
  const auto& xcdot2 = derivs2_->get_vector();

  // calculate the state at dt.
  const double kOneSixth = 1.0 / 6.0;
  xc->SetFromVector(this->get_interval_start_state());
  xc->PlusEqScaled({{dt * kOneSixth, xcdot0},
                    {4.0 * dt * kOneSixth, xcdot1},
                    {dt * kOneSixth, xcdot2}});

  // If the state of the system has changed, the error estimate will no
  // longer be sized correctly. Verify that the error estimate is the
  // correct size.
  DRAKE_DEMAND(this->get_error_estimate()->size() == xc->size());

  // Calculate the error estimate using an Eigen vector then copy it to the
  // continuous state vector, where the various state components can be
  // analyzed.
  err_est_vec_ = -this->get_interval_start_state();
  xcdot0.ScaleAndAddToVector(-dt, err_est_vec_);
  xc->ScaleAndAddToVector(1.0, err_est_vec_);
  err_est_vec_ = err_est_vec_.cwiseAbs();
  this->get_mutable_error_estimate()->SetFromVector(err_est_vec_);
}

}  // namespace systems
}  // namespace drake
