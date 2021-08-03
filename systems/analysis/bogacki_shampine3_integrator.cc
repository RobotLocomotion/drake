#include "drake/systems/analysis/bogacki_shampine3_integrator.h"

#include <cmath>
#include <stdexcept>

#include "drake/common/drake_assert.h"
#include "drake/common/unused.h"

namespace drake {
namespace systems {

/*
 * Bogacki-Shampine-specific initialization function.
 * @throws std::exception if *neither* the initial step size target nor the
 *           maximum step size have been set before calling.
 */
template <class T>
void BogackiShampine3Integrator<T>::DoInitialize() {
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
bool BogackiShampine3Integrator<T>::DoStep(const T& h) {
  using std::abs;
  Context<T>& context = *this->get_mutable_context();
  const T t0 = context.get_time();

  // CAUTION: This is performance-sensitive inner loop code that uses dangerous
  // long-lived references into state and cache to avoid unnecessary copying and
  // cache invalidation. Be careful not to insert calls to methods that could
  // invalidate any of these references before they are used.

  // We use Butcher tableau notation with labels for each coefficient:
  // 0   (c1)  |
  // 1/2 (c2)  |  1/2  (a21)
  // 3/4 (c3)  |  0    (a31)      3/4 (a32)
  // 1   (c4)  |  2/9  (a41)      1/3 (a42)    4/9 (a43)
  // ---------------------------------------------------------------------------
  //              2/9  (b1)       1/3 (b2)     4/9 (b3)     0   (b4)
  //              7/24 (d1)       1/4 (d2)     1/3 (d3)     1/8 (d4)

  // Save the continuous state at t₀.
  context.get_continuous_state_vector().CopyToPreSizedVector(&save_xc0_);

  // Evaluate the derivative at t₀, xc₀.
  derivs1_->get_mutable_vector().SetFrom(
      this->EvalTimeDerivatives(context).get_vector());
  const VectorBase<T>& k1 = derivs1_->get_vector();

  // Cache: k1 references a *copy* of the derivative result so is immune
  // to subsequent evaluations.

  // Compute the first intermediate state and derivative (i.e., Stage 2).
  // This call marks t- and xc-dependent cache entries out of date, including
  // the derivative cache entry. Note that xc is a live reference into the
  // context -- subsequent changes through that reference are unobservable so
  // will require manual out-of-date notifications.
  const double c2 = 1.0 / 2;
  const double a21 = 1.0 / 2;
  VectorBase<T>& xc = context.SetTimeAndGetMutableContinuousStateVector(
      t0 + c2 * h);
  xc.PlusEqScaled(a21 * h, k1);

  // Evaluate the derivative (denoted k2) at t₀ + c2 * h, xc₀ + a21 * h * k1.
  derivs2_->get_mutable_vector().SetFrom(
      this->EvalTimeDerivatives(context).get_vector());
  const VectorBase<T>& k2 = derivs2_->get_vector();  // xcdot⁽ᵃ⁾

  // Cache: k2 references a *copy* of the derivative result so is immune
  // to subsequent evaluations.

  // Compute the second intermediate state and derivative (i.e., Stage 3).
  const double c3 = 3.0 / 4;
  const double a31 = 0.0;
  const double a32 = 3.0 / 4;
  // This call marks t- and xc-dependent cache entries out of date, including
  // the derivative cache entry. (We already have the xc reference but must
  // issue the out-of-date notification here since we're about to change it.)
  context.SetTimeAndNoteContinuousStateChange(t0 + c3 * h);

  // Evaluate the derivative (denoted k3) at t₀ + c3 * h,
  //   xc₀ + a31 * h * k1 + a32 * h * k2.
  // Note that a31 is zero, so we leave that term out.
  unused(a31);
  xc.SetFromVector(save_xc0_);  // Restore xc ← xc₀.
  xc.PlusEqScaled({{a32 * h, k2}});
  derivs3_->get_mutable_vector().SetFrom(
      this->EvalTimeDerivatives(context).get_vector());
  const VectorBase<T>& k3 =  derivs3_->get_vector();

  // Compute the propagated solution (we're able to do this because b1 = a41,
  // b2 = a42, b3 = a43, and b4 = 0).
  const double c4 = 1.0;
  const double a41 = 2.0 / 9;
  const double a42 = 1.0 / 3;
  const double a43 = 4.0 / 9;
  // This call marks t- and xc-dependent cache entries out of date, including
  // the derivative cache entry. (We already have the xc reference but must
  // issue the out-of-date notification here since we're about to change it.)
  context.SetTimeAndNoteContinuousStateChange(t0 + c4 * h);

  // Evaluate the derivative (denoted k4) at t₀ + c4 * h, xc₀ + a41 * h * k1 +
  // a42 * h * k2 + a43 * h * k3. This will be used to compute the second
  // order solution.
  xc.SetFromVector(save_xc0_);  // Restore xc ← xc₀.
  xc.PlusEqScaled({{a41 * h, k1}, {a42 * h, k2}, {a43 * h, k3}});
  const ContinuousState<T>& derivs4 = this->EvalTimeDerivatives(context);
  const VectorBase<T>& k4 = derivs4.get_vector();

  // WARNING: k4 is a live reference into the cache. Be careful of adding
  // code below that modifies the context until after k4 is used below. In fact,
  // it is best not to modify the context from here on out, as modifying the
  // context will effectively destroy the FSAL benefit that this integrator
  // provides.

  // Compute the second order solution used for the error estimate and then
  // the error estimate itself. The first part of this formula (the part that
  // uses the d coefficients) computes the second error solution. The last part
  // subtracts the third order propagated solution from that second order
  // solution, thereby yielding the error estimate.
  const double d1 = 7.0 / 24;
  const double d2 = 1.0 / 4;
  const double d3 = 1.0 / 3;
  const double d4 = 1.0 / 8;
  err_est_vec_->SetZero();
  err_est_vec_->PlusEqScaled({{(a41 - d1) * h, k1},
                              {(a42 - d2) * h, k2},
                              {(a43 - d3) * h, k3},
                              {(-d4) * h, k4}});

  // If the size of the system has changed, the error estimate will no longer
  // be sized correctly. Verify that the error estimate is the correct size.
  DRAKE_DEMAND(this->get_error_estimate()->size() == xc.size());
  this->get_mutable_error_estimate()->SetFromVector(err_est_vec_->
      CopyToVector().cwiseAbs());

  // Bogacki-Shampine always succeeds in taking its desired step.
  return true;
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::systems::BogackiShampine3Integrator)
