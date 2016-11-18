#pragma once

/// @file
/// Template method implementations for runge_kutta_3_integrator.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "drake/systems/analysis/runge_kutta3_integrator.h"

namespace drake {
namespace systems {

template <class T>
void RungeKutta3Integrator<T>::DoInitialize() {
  // Set an artificial step size target, if not set already.
  if (!(IntegratorBase<T>::get_initial_step_size_target() <
        IntegratorBase<T>::get_maximum_step_size()))
    IntegratorBase<T>::request_initial_step_size_target(
        IntegratorBase<T>::get_maximum_step_size());
}

template <class T>
bool RungeKutta3Integrator<T>::DoStepOnceAtMost(const T& dt_max) {
  // TODO(edrumwri): move derivative evaluation at t0 to the simulator where
  // it can be used for efficient guard function zero finding.
  auto& system = IntegratorBase<T>::get_system();
  auto& context = IntegratorBase<T>::get_context();
  system.EvalTimeDerivatives(context, derivs0_.get());

  // Call the generic error controlled stepper.
  IntegratorBase<T>::StepErrorControlled(dt_max, derivs0_.get());

  const T& dt = IntegratorBase<T>::get_previous_integration_step_size();
  return (dt == dt_max);
}

template <class T>
void RungeKutta3Integrator<T>::DoStepOnceFixedSize(const T& dt) {
  using std::abs;

  // Find the continuous state xc within the Context, just once.
  VectorBase<T>* xc = IntegratorBase<T>::get_mutable_context()
                          ->get_mutable_continuous_state_vector();

  // Setup ta and tb.
  T ta = IntegratorBase<T>::get_context().get_time();
  T tb = ta + dt;

  // Get the derivative at the current state (x0) and time (t0).
  const auto& xcdot0 = derivs0_->get_vector();

  // Get the system and the context.
  const auto& system = IntegratorBase<T>::get_system();
  auto& context = IntegratorBase<T>::get_context();

  // Compute the first intermediate state and derivative (at t=0.5, x(0.5)).
  IntegratorBase<T>::get_mutable_context()->set_time(ta + dt * 0.5);
  xc->PlusEqScaled(dt * 0.5, xcdot0);
  system.EvalTimeDerivatives(context, derivs1_.get());
  const auto& xcdot1 = derivs1_->get_vector();

  // Compute the second intermediate state and derivative (at t=1, x(1)).
  IntegratorBase<T>::get_mutable_context()->set_time(tb);
  xc->SetFromVector(IntegratorBase<T>::get_interval_start_state());
  xc->PlusEqScaled({{-dt, xcdot0}, {dt * 2, xcdot1}});
  system.EvalTimeDerivatives(context, derivs2_.get());
  const auto& xcdot2 = derivs2_->get_vector();

  // calculate the state at dt.
  const double kOneSixth = 1.0 / 6.0;
  xc->SetFromVector(IntegratorBase<T>::get_interval_start_state());
  xc->PlusEqScaled({{dt * kOneSixth, xcdot0},
                    {4.0 * dt * kOneSixth, xcdot1},
                    {dt * kOneSixth, xcdot2}});

  // If the state of the system has changed, the error estimate will no
  // longer be sized correctly. Verify that the error estimate is the
  // correct size.
  DRAKE_DEMAND(IntegratorBase<T>::get_error_estimate()->size() == xc->size());

  // Calculate the error estimate using an Eigen vector then copy it to the
  // continuous state vector, where the various state components can be
  // analyzed.
  err_est_vec_ = -IntegratorBase<T>::get_interval_start_state();
  xcdot0.ScaleAndAddToVector(-dt, err_est_vec_);
  xc->ScaleAndAddToVector(1.0, err_est_vec_);
  err_est_vec_ = err_est_vec_.cwiseAbs();
  IntegratorBase<T>::get_mutable_error_estimate()->SetFromVector(err_est_vec_);
}

}  // namespace systems
}  // namespace drake
