#include "drake/systems/analysis/runge_kutta5_integrator.h"

namespace drake {
namespace systems {

/*
 * RK5-specific initialization function.
 * @throws std::exception if *neither* the initial step size target nor the
 *           maximum step size has been set before calling.
 */
template <typename T>
void RungeKutta5Integrator<T>::DoInitialize() {
  using std::isnan;
  // TODO(drum) Verify the integrator-specific accuracy settings below.
  const double kDefaultAccuracy = 1e-5;
  const double kLoosestAccuracy = 1e-3;
  const double kMaxStepFraction = 0.1;   // Fraction of max step size for
                                         // less aggressive first step.

  // Set an artificial step size target, if not set already.
  if (isnan(this->get_initial_step_size_target())) {
    // Verify that maximum step size has been set.
    if (isnan(this->get_maximum_step_size()))
      throw std::logic_error(
          "Neither initial step size target nor maximum "
          "step size has been set!");

    this->request_initial_step_size_target(this->get_maximum_step_size() *
                                           kMaxStepFraction);
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

template <typename T>
bool RungeKutta5Integrator<T>::DoStep(const T& h) {
  using std::abs;
  Context<T>& context = *this->get_mutable_context();
  const T t0 = context.get_time();
  const T t1 = t0 + h;

  // CAUTION: This is performance-sensitive inner loop code that uses dangerous
  // long-lived references into state and cache to avoid unnecessary copying and
  // cache invalidation. Be careful not to insert calls to methods that could
  // invalidate any of these references before they are used.

  // We use Butcher tableau notation with labels for each coefficient:
  /*
   0    (c1) |
   1/5  (c2) |        1/5 (a21)
   3/10 (c3) |       3/40 (a31)          9/40 (a32)
   4/5  (c4) |      44/45 (a41)        -56/15 (a42)         32/9 (a43)
   8/9  (c5) | 19372/6561 (a51)   −25360/2187 (a52)   64448/6561 (a53)   −212/729 (a54)                                                  // NOLINT(*)
   1    (c6) |  9017/3168 (a61)       −355/33 (a62)   46732/5247 (a63)     49/176 (a64)    −5103/18656 (a65)                             // NOLINT(*)
   1    (c7) |     35/384 (a71)             0 (a72)     500/1113 (a73)    125/192 (a74)     −2187/6784 (a75)     11/84 (a76)             // NOLINT(*)
   ------------------------------------------------------------------------------------------------------------------------------------  // NOLINT(*)
                   35/384  (b1)             0 (b2)      500/1113 (b3)     125/192 (b4)      −2187/6784 (b5)      11/84 (b6)      0 (b7)  // NOLINT(*)
               5179/57600  (d1)             0 (d2)    7571/16695 (d3)     393/640 (d4)   −92097/339200 (d5)   187/2100 (d6)   1/40 (d7)  // NOLINT(*)
  */

  // Save the continuous state at t₀.
  context.get_continuous_state_vector().CopyToPreSizedVector(&save_xc0_);

  // Evaluate the derivative at t₀, xc₀ and copy the result into a temporary.
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
  const double c2 = 1.0 / 5;
  const double a21 = 1.0 / 5;
  VectorBase<T>& xc =
      context.SetTimeAndGetMutableContinuousStateVector(t0 + c2 * h);
  xc.PlusEqScaled(a21 * h, k1);

  // Evaluate the derivative (denoted k2) at t₀ + c2 * h, xc₀ + a21 * h * k1.
  derivs2_->get_mutable_vector().SetFrom(
      this->EvalTimeDerivatives(context).get_vector());
  const VectorBase<T>& k2 = derivs2_->get_vector();

  // Cache: k2 references a *copy* of the derivative result so is immune
  // to subsequent evaluations.

  // Compute the second intermediate state and derivative (i.e., Stage 3).
  const double c3 = 3.0 / 10;
  const double a31 = 3.0 / 40;
  const double a32 = 9.0 / 40;
  // This call marks t- and xc-dependent cache entries out of date, including
  // the derivative cache entry. (We already have the xc reference but must
  // issue the out-of-date notification here since we're about to change it.)
  context.SetTimeAndNoteContinuousStateChange(t0 + c3 * h);

  // Evaluate the derivative (denoted k3) at t₀ + c3 * h,
  //   xc₀ + a31 * h * k1 + a32 * h * k2.
  xc.SetFromVector(save_xc0_);  // Restore xc ← xc₀.
  xc.PlusEqScaled({{a31 * h, k1}, {a32 * h, k2}});
  derivs3_->get_mutable_vector().SetFrom(
      this->EvalTimeDerivatives(context).get_vector());
  const VectorBase<T>& k3 = derivs3_->get_vector();

  // Compute the third intermediate state and derivative (i.e., Stage 4).
  const double c4 = 4.0 / 5;
  const double a41 = 44.0 / 45;
  const double a42 = -56.0 / 15;
  const double a43 = 32.0 / 9;
  // This call marks t- and xc-dependent cache entries out of date, including
  // the derivative cache entry. (We already have the xc reference but must
  // issue the out-of-date notification here since we're about to change it.)
  context.SetTimeAndNoteContinuousStateChange(t0 + c4 * h);

  // Evaluate the derivative (denoted k4) at t₀ + c4 * h,
  //   xc₀ + a41 * h * k1 + a42 * h * k2 + a43 * h * k3.
  xc.SetFromVector(save_xc0_);
  xc.PlusEqScaled({{a41 * h, k1}, {a42 * h, k2}, {a43 * h, k3}});
  derivs4_->get_mutable_vector().SetFrom(
      this->EvalTimeDerivatives(context).get_vector());
  const VectorBase<T>& k4 = derivs4_->get_vector();

  // Compute the fourth intermediate state and derivative (i.e., Stage 5).
  const double c5 = 8.0 / 9;
  const double a51 = 19372.0 / 6561;
  const double a52 = -25360.0 / 2187;
  const double a53 = 64448.0 / 6561;
  const double a54 = -212.0 / 729;
  // This call marks t- and xc-dependent cache entries out of date, including
  // the derivative cache entry. (We already have the xc reference but must
  // issue the out-of-date notification here since we're about to change it.)
  context.SetTimeAndNoteContinuousStateChange(t0 + c5 * h);

  // Evaluate the derivative (denoted k5) at t₀ + c5 * h,
  //   xc₀ + a51 * h * k1 + a52 * h * k2 + a53 * h * k3 + a54 * h * k4.
  xc.SetFromVector(save_xc0_);  // Restore xc ← xc₀.
  xc.PlusEqScaled({{a51 * h, k1}, {a52 * h, k2}, {a53 * h, k3}, {a54 * h, k4}});
  derivs5_->get_mutable_vector().SetFrom(
      this->EvalTimeDerivatives(context).get_vector());
  const VectorBase<T>& k5 = derivs5_->get_vector();

  // Compute the fifth intermediate state and derivative (i.e., Stage 6).
  const double a61 = 9017.0 / 3168;
  const double a62 = -355.0 / 33;
  const double a63 = 46732.0 / 5247;
  const double a64 = 49.0 / 176;
  const double a65 = -5103.0 / 18656;
  // This call marks t- and xc-dependent cache entries out of date, including
  // the derivative cache entry. (We already have the xc reference but must
  // issue the out-of-date notification here since we're about to change it.)
  context.SetTimeAndNoteContinuousStateChange(t1);

  // Evaluate the derivative (denoted k6) at t₀ + c6 * h,
  //   xc₀ + a61 * h * k1 + a62 * h * k2 + a63 * h * k3 + a64 * h * k4 +
  //       a65 * h * k5.
  xc.SetFromVector(save_xc0_);
  xc.PlusEqScaled({{a61 * h, k1},
                   {a62 * h, k2},
                   {a63 * h, k3},
                   {a64 * h, k4},
                   {a65 * h, k5}});
  derivs6_->get_mutable_vector().SetFrom(
      this->EvalTimeDerivatives(context).get_vector());
  const VectorBase<T>& k6 = derivs6_->get_vector();

  // Cache: we're about to write through the xc reference again, so need to
  // mark xc-dependent cache entries out of date; time doesn't change here.
  context.NoteContinuousStateChange();

  // Compute the propagated solution (we're able to do this because b1 = a71,
  // b2 = a72, b3 = a73, b4 = a74, b5 = a75, and b6 = a76).
  // Note that a72 is 0.0, so we leave that term out below.
  const double a71 = 35.0 / 384;
  const double a73 = 500.0 / 1113;
  const double a74 = 125.0 / 192;
  const double a75 = -2187.0 / 6784;
  const double a76 = 11.0 / 84;
  // This call marks t- and xc-dependent cache entries out of date, including
  // the derivative cache entry. (We already have the xc reference but must
  // issue the out-of-date notification here since we're about to change it.)
  // Note that we use the simplification t1 = t0 + h * c7 = t0 + h * 1.
  context.SetTimeAndNoteContinuousStateChange(t1);

  // Evaluate the derivative (denoted k7) at t₀ + c7 * h,
  //   xc₀ + a71 * h * k1 + a72 * h * k2 + a73 * h * k3 + a74 * h * k4 +
  //       a75 * h * k5 + a76 * h * k6.
  xc.SetFromVector(save_xc0_);
  xc.PlusEqScaled({{a71 * h, k1},
                   {a73 * h, k3},
                   {a74 * h, k4},
                   {a75 * h, k5},
                   {a76 * h, k6}});
  const ContinuousState<T>& derivs7 = this->EvalTimeDerivatives(context);
  const VectorBase<T>& k7 = derivs7.get_vector();

  // WARNING: k7 is a live reference into the cache. Be careful of adding
  // code below that modifies the context until after k7 is used below. In fact,
  // it is best not to modify the context from here on out, as modifying the
  // context will effectively destroy the FSAL benefit that this integrator
  // provides.

  // Calculate the 4th-order solution that will be used for the error
  // estimate and then the error estimate itself. The part of this
  // formula that uses the "a" coefficients (re)-computes the fifth order
  // solution. The part of this formula that uses the "d" coefficients computes
  // the fourth order solution. The subtraction (and negation) operations
  // yield the error estimate.
  // Note: d2 is 0.0; it has been removed from the formula below.
  const double d1 = 5179.0 / 57600;
  const double d3 = 7571.0 / 16695;
  const double d4 = 393.0 / 640;
  const double d5 = -92097.0 / 339200;
  const double d6 = 187.0 / 2100;
  const double d7 = 1.0 / 40;
  err_est_vec_->SetZero();
  err_est_vec_->PlusEqScaled({{(a71 - d1) * h, k1},
                              {(a73 - d3) * h, k3},
                              {(a74 - d4) * h, k4},
                              {(a75 - d5) * h, k5},
                              {(a76 - d6) * h, k6},
                              {(-d7) * h, k7}});

  // If the size of the system has changed, the error estimate will no longer
  // be sized correctly. Verify that the error estimate is the correct size.
  DRAKE_DEMAND(this->get_error_estimate()->size() == xc.size());
  this->get_mutable_error_estimate()->SetFromVector(
      err_est_vec_->CopyToVector().cwiseAbs());

  // RK5 always succeeds in taking its desired step.
  return true;
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::systems::RungeKutta5Integrator)
