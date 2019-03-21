#pragma once

/// @file
/// Template method implementations for runge_kutta_5_integrator.h.
/// Most users should only include that file, not this one.
/// For background, see https://drake.mit.edu/cxx_inl.html.

/* clang-format off to disable clang-format-includes */
#include "drake/systems/analysis/runge_kutta5_integrator.h"
/* clang-format on */

#include <utility>

namespace drake {
namespace systems {

/**
 * RK5-specific initialization function.
 * @throws std::logic_error if *neither* the initial step size target nor the
 *           maximum step size have been set before calling.
 */
template <class T>
void RungeKutta5Integrator<T>::DoInitialize() {
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
bool RungeKutta5Integrator<T>::DoStep(const T& h) {
  using std::abs;
  Context<T>& context = *this->get_mutable_context();
  const T t0 = context.get_time();
  const T t1 = t0 + h;

  // CAUTION: This is performance-sensitive inner loop code that uses dangerous
  // long-lived references into state and cache to avoid unnecessary copying and
  // cache invalidation. Be careful not to insert calls to methods that could
  // invalidate any of these references before they are used.

  // TODO(sherm1) Consider moving this notation description to IntegratorBase
  //              when it is more widely adopted.
  // Notation: we're using numeric subscripts for times t₀ and t₁, and
  // lower-case letter superscripts like t⁽ᵃ⁾ and t⁽ᵇ⁾ to indicate values
  // for intermediate stages of which there are two here, a and b.
  // State x₀ = {xc₀, xd₀, xa₀}. We modify only t and xc here, but
  // derivative calculations depend on everything in the context, including t,
  // x and inputs u (which may depend on t and x).
  // Define x⁽ᵃ⁾ ≜ {xc⁽ᵃ⁾, xd₀, xa₀} and u⁽ᵃ⁾ ≜ u(t⁽ᵃ⁾, x⁽ᵃ⁾).

  // Evaluate derivative xcdot₀ ← xcdot(t₀, x(t₀), u(t₀)). Copy the result
  // into a temporary since we'll be calculating more derivatives below.
  derivs1_->get_mutable_vector().SetFrom(
      this->EvalTimeDerivatives(context).get_vector());
  const VectorBase<T>& k1 = derivs1_->get_vector();

  // Cache: k1 references a *copy* of the derivative result so is immune
  // to subsequent evaluations.

  // Compute the first intermediate state and derivative
  // (at t⁽ᵃ⁾=t₀+h/5, x⁽ᵃ⁾, u⁽ᵃ⁾).
  const double c2 = 1.0 / 5;
  const double a21 = 1.0 / 5;

  // This call marks t- and xc-dependent cache entries out of date, including
  // the derivative cache entry. Note that xc is a live reference into the
  // context -- subsequent changes through that reference are unobservable so
  // will require manual out-of-date notifications.
  VectorBase<T>& xc = context.SetTimeAndGetMutableContinuousStateVector(
      t0 + c2 * h);                      // t⁽ᵃ⁾ ← t₀ + h/5
  xc.CopyToPreSizedVector(&save_xc0_);   // Save xc₀ while we can.
  xc.PlusEqScaled(a21 * h, k1);          // xc⁽ᵃ⁾ ← xc₀ + h/5 xcdot₀

  derivs2_->get_mutable_vector().SetFrom(
      this->EvalTimeDerivatives(context).get_vector());
  const VectorBase<T>& k2 = derivs2_->get_vector();  // xcdot⁽ᵃ⁾

  // Cache: xcdot_a references a *copy* of the derivative result so is immune
  // to subsequent evaluations.

  // Compute the second intermediate state and derivative
  // (at t⁽ᵇ⁾=t₁, x⁽ᵇ⁾, u⁽ᵇ⁾).

  // This call marks t- and xc-dependent cache entries out of date, including
  // the derivative cache entry. (We already have the xc reference but must
  // issue the out-of-date notification here since we're about to change it.)
  const double c3 = 3.0 / 10;
  const double a31 = 3.0 / 40;
  const double a32 = 9.0 / 40;
  context.SetTimeAndNoteContinuousStateChange(t0 + c3 * h);

  // xcⱼ ← xc₀ - h xcdot₀ + 2 h xcdot⁽ᵃ⁾
  xc.SetFromVector(save_xc0_);  // Restore xc ← xc₀.
  xc.PlusEqScaled({{a31 * h, k1}, {a32 * h, k2}});
  derivs3_->get_mutable_vector().SetFrom(
      this->EvalTimeDerivatives(context).get_vector());
  const VectorBase<T>& k3 =  derivs3_->get_vector();

  const double c4 = 4.0 / 5;
  const double a41 = 44.0 / 45;
  const double a42 = -56.0 / 15;
  const double a43 = 32.0 / 9;
  context.SetTimeAndNoteContinuousStateChange(t0 + c4 * h);

  xc.SetFromVector(save_xc0_);  // Restore xc ← xc₀.
  xc.PlusEqScaled({{a41 * h, k1}, {a42 * h, k2}, {a43 * h, k3}});
  derivs4_->get_mutable_vector().SetFrom(
      this->EvalTimeDerivatives(context).get_vector());
  const VectorBase<T>& k4 =  derivs4_->get_vector();


  const double c5 = 8.0 / 9;
  const double a51 = 19372.0 / 6561;
  const double a52 = -25360.0 / 2187;
  const double a53 = 64448.0 / 6561;
  const double a54 = -212.0 / 729;
  context.SetTimeAndNoteContinuousStateChange(t0 + c5 * h);
  xc.SetFromVector(save_xc0_);  // Restore xc ← xc₀.
  xc.PlusEqScaled({{a51 * h, k1}, {a52 * h, k2}, {a53 * h, k3}, {a54 * h, k4}});
  derivs5_->get_mutable_vector().SetFrom(
      this->EvalTimeDerivatives(context).get_vector());
  const VectorBase<T>& k5 =  derivs5_->get_vector();

  // Cache: we're about to write through the xc reference again, so need to
  // mark xc-dependent cache entries out of date, including xcdot_b; time
  // doesn't change here.

  const double a61 = 9017.0 / 3168;
  const double a62 = -355.0 /33;
  const double a63 = 46732.0 / 5247;
  const double a64 = 49.0 / 176;
  const double a65 = -5103.0 / 18656;
  context.SetTimeAndNoteContinuousStateChange(t1);
  xc.SetFromVector(save_xc0_);  // Restore xc ← xc₀.
  xc.PlusEqScaled({{a61 * h, k1}, {a62 * h, k2}, {a63 * h, k3}, {a64 * h, k4},
      {a65 * h, k5}});
  derivs6_->get_mutable_vector().SetFrom(
      this->EvalTimeDerivatives(context).get_vector());
  const VectorBase<T>& k6 =  derivs6_->get_vector();

  // Cache: we're about to write through the xc reference again, so need to
  // mark xc-dependent cache entries out of date, including xcdot_b; time
  // doesn't change here.
  context.NoteContinuousStateChange();

  // Note: a72 is 0.0; it has been removed from the formula below.
  const double a71 = 35.0 / 384;
  const double a73 = 500.0 / 1113;
  const double a74 = 125.0 / 192;
  const double a75 = -2187.0 / 6784;
  const double a76 = 11.0 / 84;
  xc.SetFromVector(save_xc0_);  // Restore xc ← xc₀.
  xc.PlusEqScaled({{a71 * h, k1}, {a73 * h, k3}, {a74 * h, k4}, {a75 * h, k5},
      {a76 * h, k6}});
  derivs7_->get_mutable_vector().SetFrom(
      this->EvalTimeDerivatives(context).get_vector());
  const VectorBase<T>& k7 =  derivs7_->get_vector();

  // Calculate the second solution that will be used for the error estimate.
  // Note: d2 is 0.0; it has been removed from the formula below.
  const double d1 = 5179.0 / 57600;
  const double d3 = 7571.0 / 16695;
  const double d4 = 393.0 / 640;
  const double d5 = -92097.0 / 339200;
  const double d6 = 187.0 / 2100;
  const double d7 = 1.0 / 40;
  xc.SetFromVector(save_xc0_);  // Restore xc ← xc₀.
  xc.PlusEqScaled({{d1 * h, k1}, {d3 * h, k3}, {d4 * h, k4}, {d5 * h, k5},
      {d6 * h, k6}, {d7 * h, k7}});
  err_est_vec_ = -xc.CopyToVector();

  // If the size of the system has changed, the error estimate will no longer
  // be sized correctly. Verify that the error estimate is the correct size.
  DRAKE_DEMAND(this->get_error_estimate()->size() == xc.size());

  // Calculate the final O(h5) state at t₁.
  // Note: b2 and b7 are 0.0; they have been removed from the formula below.
  const double b1 = 35.0 / 384;
  const double b3 = 500.0 / 1113;
  const double b4 = 125.0 / 192;
  const double b5 = -2187.0 / 6784;
  const double b6 = 11.0 / 84;
  xc.SetFromVector(save_xc0_);  // Restore xc ← xc₀.
  xc.PlusEqScaled({{b1 * h, k1}, {b3 * h, k3}, {b4 * h, k4}, {b5 * h, k5},
      {b6 * h, k6}});
  err_est_vec_ += xc.CopyToVector();  // ε ← xc₀
  err_est_vec_ = err_est_vec_.cwiseAbs();
  this->get_mutable_error_estimate()->SetFromVector(err_est_vec_);


  // RK5 always succeeds in taking its desired step.
  return true;
}

}  // namespace systems
}  // namespace drake
