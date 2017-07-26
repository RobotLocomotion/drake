#pragma once

/// @file
/// Template method implementations for runge_kutta_merson_integrator.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "runge_kutta_merson_integrator.h"

namespace drake {
namespace systems {

template <class T>
void RungeKuttaMersonIntegrator<T>::DoInitialize() {
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

template <class T>
bool RungeKuttaMersonIntegrator<T>::DoStep(const T& dt) {
  auto context = IntegratorBase<T>::get_mutable_context();
  this->CalcTimeDerivatives(*context, derivs0_.get());

  // Find the continuous state xc within the Context, just once.
  VectorBase<T>* xc = context->get_mutable_state()->
      get_mutable_continuous_state()->get_mutable_vector();

  // Get the state at t0.
  const VectorX<T> xt0 = xc->CopyToVector();

  // Set ta and tb.
  T ta = context->get_time();
  T tb = ta + dt;

  // Get the derivative at the current state (x0) and time (t0).
  const auto& xcdot0 = derivs0_->get_vector();

  // Compute the first intermediate state and derivative (at t=1/3, x(1/3)).
  context->set_time(ta + dt * 1.0/3);
  xc->PlusEqScaled({{dt * 1.0/3, xcdot0}});
  this->CalcTimeDerivatives(*context, derivs1_.get());
  const auto& xcdot1 = derivs1_->get_vector();

  // Compute the second intermediate state and derivative (at t=1/3, x(1/6)).
  context->set_time(ta + dt * 1.0/3);
  xc->SetFromVector(xt0);
  xc->PlusEqScaled({{dt * 1.0/6, xcdot0}, {dt * 1.0/6, xcdot1}});
  this->CalcTimeDerivatives(*context, derivs2_.get());
  const auto& xcdot2 = derivs2_->get_vector();

  // Compute the third intermediate state and derivative (at t=1/2)
  context->set_time(ta + dt * 0.5);
  xc->SetFromVector(xt0);
  xc->PlusEqScaled({{dt * 1.0/8, xcdot0}, {dt * 3.0/8, xcdot2}});
  this->CalcTimeDerivatives(*context, derivs3_.get());
  const auto& xcdot3 = derivs3_->get_vector();

  // Compute the fourth intermediate state and derivative.
  context->set_time(ta + dt * 1.0);
  xc->SetFromVector(xt0);
  xc->PlusEqScaled({{dt * 0.5, xcdot0}, {dt * -3.0/2, xcdot2},
                    {dt * 2.0, xcdot3}});
  const VectorX<T> xsave = xc->CopyToVector();
  this->CalcTimeDerivatives(*context, derivs4_.get());
  const auto& xcdot4 = derivs4_->get_vector();

  // Compute the state at tb and derivative (needed for error estimation).
  context->set_time(tb);
  xc->SetFromVector(xt0);
  xc->PlusEqScaled({{dt * 1.0/6, xcdot0}, {dt * 2.0/3, xcdot3},
                    {dt * 1.0/6, xcdot4}});

  // Calculate the error estimate.
  this->get_mutable_error_estimate()->SetFromVector(
      (xc->CopyToVector() - xsave) * 2);

  // RK Merson always succeeds in taking its desired step.
  return true;
}

}  // systems
}  // drake
