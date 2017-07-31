#pragma once

/// @file
/// Template method implementations for runge_kutta_merson_integrator.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "drake/systems/analysis/runge_kutta_merson_integrator.h"

namespace drake {
namespace systems {

template <class T>
void RungeKuttaMersonIntegrator<T>::DoInitialize() {
  const double kDefaultAccuracy = 1e-3;  // Good for this particular integrator.
  const double kLoosestAccuracy = 1e-1;  // Integrator specific.
  const double kMaxStepFraction = 0.1;   // Fraction of max step size for
                                         // less aggressive first step.

  this->InitializeAccuracy(kDefaultAccuracy, kLoosestAccuracy,
                           kMaxStepFraction);
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

  // Constants for the Butcher Tableaux.
  const T kOneSixthDt = 1.0/6 * dt;
  const T kOneThirdDt = kOneSixthDt * 2;
  const T kTwoThirdsDt = kOneThirdDt * 2;
  const T kOneEighthDt = 1.0/8 * dt;
  const T kOneHalfDt = kOneEighthDt * 4;
  const T kThreeEighthsDt = kOneEighthDt * 3;
  const T kNegThreeHalvesDt = kOneHalfDt * -3;
  const T kTwoDt = 2.0 * dt;

  // Compute the first intermediate state and derivative (at t=1/3, x(1/3)).
  context->set_time(ta + kOneThirdDt);
  xc->PlusEqScaled({{kOneThirdDt, xcdot0}});
  this->CalcTimeDerivatives(*context, derivs1_.get());
  const auto& xcdot1 = derivs1_->get_vector();

  // Compute the second intermediate state and derivative (at t=1/3, x(1/6)).
  context->set_time(ta + kOneThirdDt);
  xc->SetFromVector(xt0);
  xc->PlusEqScaled({{kOneSixthDt, xcdot0}, {kOneSixthDt, xcdot1}});
  this->CalcTimeDerivatives(*context, derivs2_.get());
  const auto& xcdot2 = derivs2_->get_vector();

  // Compute the third intermediate state and derivative (at t=1/2)
  context->set_time(ta + kOneHalfDt);
  xc->SetFromVector(xt0);
  xc->PlusEqScaled({{kOneEighthDt, xcdot0}, {kThreeEighthsDt, xcdot2}});
  this->CalcTimeDerivatives(*context, derivs3_.get());
  const auto& xcdot3 = derivs3_->get_vector();

  // Compute the fourth intermediate state and derivative.
  context->set_time(tb);
  xc->SetFromVector(xt0);
  xc->PlusEqScaled({{kOneHalfDt, xcdot0}, {kNegThreeHalvesDt, xcdot2},
                    {kTwoDt, xcdot3}});
  xsave_ = xc->CopyToVector();
  this->CalcTimeDerivatives(*context, derivs4_.get());
  const auto& xcdot4 = derivs4_->get_vector();

  // Compute the state at tb and derivative (needed for error estimation).
  context->set_time(tb);
  xc->SetFromVector(xt0);
  xc->PlusEqScaled({{kOneSixthDt, xcdot0}, {kTwoThirdsDt, xcdot3},
                    {kOneSixthDt, xcdot4}});

  // Calculate the error estimate.
  this->get_mutable_error_estimate()->SetFromVector(
      (xc->CopyToVector() - xsave_) * 0.2);

  // RK Merson always succeeds in taking its desired step.
  return true;
}

}  // namespace systems
}  // namespace drake
