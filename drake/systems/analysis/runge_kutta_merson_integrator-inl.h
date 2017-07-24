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
  auto& system = IntegratorBase<T>::get_system();
  auto context = IntegratorBase<T>::get_mutable_context();
  system.CalcTimeDerivatives(*context, derivs0_.get());

  // Setup constants for the tableaux.
  const double C21 = double(1.0 / 3.0);
  const double C22 = double(1.0 / 3.0);

  const double C31 = double(1.0 / 3.0);
  const double C32 = double(3.0 / 32.0);
  const double C33 = double(9.0 / 32.0);

  const double C41 = double(12.0 / 13.0);
  const double C42 = double(1932.0 / 2197.0);
  const double C43 = double(-7200.0 / 2197.0);
  const double C44 = double(7296.0 / 2197.0);

  const double C51 = double(1.0);
  const double C52 = double(439.0 / 216.0);
  const double C53 = double(-8.0);
  const double C54 = double(3680.0 / 513.0);
  const double C55 = double(-845.0 / 4104.0);

  const double C61 = double(1.0 / 2.0);
  const double C62 = double(-8.0 / 27.0);
  const double C63 = double(2.0);
  const double C64 = double(-3544.0 / 2565.0);
  const double C65 = double(1859.0 / 4104.0);
  const double C66 = double(-11.0 / 40.0);

  const double dCY1 = 25.0 / 216.0;
  const double dCY2 = 1408.0 / 2565.0;
  const double dCY3 = 2197.0 / 4104.0;
  const double dCY4 = -1.0 / 5.0;

  const double CE1 = double(16.0 / 135.0 - dCY1);
  const double CE2 = double(6656.0 / 12825.0 - dCY2);
  const double CE3 = double(28561.0 / 56430.0 - dCY3);
  const double CE4 = double(-9.0 / 50.0 - dCY4);
  const double CE5 = double(2.0 / 55.0);

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

  // Compute the first intermediate state and derivative (at t=1/4, x(1/4)).
  context->set_time(ta + dt * C21);
  xc->PlusEqScaled({{dt * C22, xcdot0}});
  system.CalcTimeDerivatives(*context, derivs1_.get());
  const auto& xcdot1 = derivs1_->get_vector();

  // Compute the second intermediate state and derivative (at t=3/8, x(3/8)).
  context->set_time(ta + dt * C31);
  xc->SetFromVector(xt0);
  xc->PlusEqScaled({{dt * C32, xcdot0}, {dt * C33, xcdot1}});
  system.CalcTimeDerivatives(*context, derivs2_.get());
  const auto& xcdot2 = derivs2_->get_vector();

  // Compute the third intermediate state and derivative.
  context->set_time(ta + dt * C41);
  xc->SetFromVector(xt0);
  xc->PlusEqScaled({{dt * C42, xcdot0}, {dt * C43, xcdot1},
                    {dt * C44, xcdot2}});
  system.CalcTimeDerivatives(*context, derivs3_.get());
  const auto& xcdot3 = derivs3_->get_vector();

  // Compute the fourth intermediate state and derivative.
  context->set_time(ta + dt * C51);
  xc->SetFromVector(xt0);
  xc->PlusEqScaled({{dt * C52, xcdot0}, {dt * C53, xcdot1}, {dt * C54, xcdot2},
                    {dt * C55, xcdot3}});
  system.CalcTimeDerivatives(*context, derivs4_.get());
  const auto& xcdot4 = derivs4_->get_vector();

  // Compute the fifth intermediate state and derivative.
  context->set_time(ta + dt * C61);
  xc->SetFromVector(xt0);
  xc->PlusEqScaled({{dt * C62, xcdot0}, {dt * C63, xcdot1}, {dt * C64, xcdot2},
                    {dt * C65, xcdot3}, {dt * C66, xcdot4}});
  system.CalcTimeDerivatives(*context, derivs5_.get());
  const auto& xcdot5 = derivs5_->get_vector();

  // Calculate the state at dt.
  context->set_time(tb);
  xc->SetFromVector(xt0);
  xc->PlusEqScaled({{dt * dCY1, xcdot0}, {dt * dCY2, xcdot2},
                    {dt * dCY3, xcdot3}, {dt * dCY4, xcdot4}});

  // Calculate the error estimate.
  if (!err_est_ || err_est_->size() != xcdot0.size())
    err_est_ = std::make_unique<BasicVector<T>>(xcdot0.size());
  err_est_->SetZero();
  err_est_->PlusEqScaled({{dt * CE1, xcdot0}, {dt * CE2, xcdot2},
                          {dt * CE3, xcdot3}, {dt * CE4, xcdot4},
                          {dt * CE5, xcdot5}});
  this->get_mutable_error_estimate()->SetFromVector(err_est_->CopyToVector());

  // RK Merson always succeeds in taking its desired step.
  return true;
}

}  // systems
}  // drake
