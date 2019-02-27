#pragma once

/// @file
/// Template method implementations for runge_kutta_3_integrator.h.
/// Most users should only include that file, not this one.
/// For background, see https://drake.mit.edu/cxx_inl.html.

/* clang-format off to disable clang-format-includes */
#include "drake/systems/analysis/runge_kutta3_integrator.h"
/* clang-format on */

#include <utility>

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

template <class T>
bool RungeKutta3Integrator<T>::DoStep(const T& h) {
  using std::abs;
  Context<T>& context = *this->get_mutable_context();
  const T t0 = context.get_time();
  const T t1 = t0 + h;

  // Evaluate derivative xcdot(t₀) ← xcdot(t₀, x(t₀), u(t₀)). Copy the result
  // into a temporary since we'll be calculating more derivatives below.
  derivs0_->get_mutable_vector().SetFrom(
      this->EvalTimeDerivatives(context).get_vector());
  const VectorBase<T>& xcdot0 = derivs0_->get_vector();

  // Compute the first intermediate state and derivative (at t'=t₀+h/2, x(t')).
  // This call invalidates t- and xc-dependent cache entries.
  VectorBase<T>& xc = context.SetTimeAndGetMutableContinuousStateVector(
      t0 + h / 2);                     // t' ← t₀ + h/2
  xc.CopyToPreSizedVector(save_xc0_);  // Save xc₀ while we can.
  xc.PlusEqScaled(h / 2, xcdot0);      // xc' ← xc₀ + h/2 ẋc₀

  derivs1_->get_mutable_vector().SetFrom(
      this->EvalTimeDerivatives(context).get_vector());
  const VectorBase<T>& xcdot_p = derivs1_->get_vector();  // ẋc'

  // Compute the second intermediate state and derivative (at t=t₁, x(t₁)).
  // This call invalidates t- and xc-dependent cache entries.
  context.SetTimeAndNoteContinuousStateChange(t1);
  // xc'' ← xc₀ - h ẋc₀ + 2 h ẋc'
  xc.SetFromVector(save_xc0_);  // Restore xc ← xc₀.
  xc.PlusEqScaled({{-h, xcdot0}, {2 * h, xcdot_p}});
  const VectorBase<T>& xcdot_pp =  // ẋc''
      this->EvalTimeDerivatives(context).get_vector();

  // Calculate the final O(h³) state at t₁.
  context.NoteContinuousStateChange();
  // xc₁ ← xc₀ + h/6 ẋc₀ + 2/3 h ẋc' + h/6 ẋc''
  xc.SetFromVector(save_xc0_);  // Restore xc ← xc₀.
  const double kOneSixth = 1.0 / 6.0;
  xc.PlusEqScaled({{h * kOneSixth,     xcdot0},
                   {4 * h * kOneSixth, xcdot_p},
                   {h * kOneSixth,     xcdot_pp}});

  // If the size of the system has changed, the error estimate will no
  // longer be sized correctly. Verify that the error estimate is the
  // correct size.
  DRAKE_DEMAND(this->get_error_estimate()->size() == xc.size());

  // Calculate the error estimate using an Eigen vector then copy it to the
  // continuous state vector, where the various state components can be
  // analyzed.
  // ε = | xc₁ - (xc₀ + h ẋc') | = | xc₀ + h ẋc' - xc₁ |
  err_est_vec_ = save_xc0_;  // ε ← xc₀
  // TODO(sherm1) This is ẋc₀, not ẋc'! Should be xcdot_p; see issue #10633.
  xcdot0.ScaleAndAddToVector(h, err_est_vec_);  // ε += h ẋc' (should be!)
  xc.ScaleAndAddToVector(-1.0, err_est_vec_);   // ε -= xc₁
  err_est_vec_ = err_est_vec_.cwiseAbs();
  this->get_mutable_error_estimate()->SetFromVector(err_est_vec_);

  // RK3 always succeeds in taking its desired step.
  return true;
}

}  // namespace systems
}  // namespace drake
