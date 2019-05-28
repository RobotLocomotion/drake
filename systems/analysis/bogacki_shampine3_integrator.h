#pragma once

#include <memory>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/common/unused.h"
#include "drake/systems/analysis/integrator_base.h"

namespace drake {
namespace systems {

/**
 A third-order, four-stage, first-same-as-last (FSAL) Runge Kutta integrator
 with a second order error estimate.

 @tparam T A double or autodiff type.

 Instantiated templates for the following kinds of T's are provided:
 - double
 - AutoDiffXd

 For a discussion of this Runge-Kutta method, see [Hairer, 1993].
 The Butcher tableaux for this integrator follows:
 <pre>
 0    |
 1/2  | 1/2
 3/4  | 0           3/4
 1    | 2/9         1/3      4/9
 -----------------------------------------------------------------------------
        2/9         1/3      4/9     0
        7/24        1/4      1/3     1/8
</pre>
 where the second to last row is the 3rd-order (propagated) solution and
 the last row gives a 2nd-order accurate solution which, when subtracted
 from the propagated solution, yields the 2nd-order error estimate of the
 error.
  - [Bogacki, 1989] P. Bogacki and L. Shampine. "A 3(2) pair of Runge–Kutta
    formulas", Appl. Math. Letters, 2 (4): 321–325, 1989.
 */
template <class T>
class BogackiShampine3Integrator final : public IntegratorBase<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BogackiShampine3Integrator)

  ~BogackiShampine3Integrator() override = default;

  explicit BogackiShampine3Integrator(const System<T>& system,
      Context<T>* context = nullptr) : IntegratorBase<T>(system, context) {
    derivs1_ = system.AllocateTimeDerivatives();
    derivs2_ = system.AllocateTimeDerivatives();
    derivs3_ = system.AllocateTimeDerivatives();
    derivs4_ = system.AllocateTimeDerivatives();
    err_est_vec_.resize(derivs1_->size());
    save_xc0_.resize(derivs1_->size());
  }

  /**
   * The integrator supports error estimation.
   */
  bool supports_error_estimation() const override { return true; }

  /// This integrator provides second order error estimates.
  int get_error_estimate_order() const override { return 2; }

 private:
  void DoInitialize() override;
  bool DoStep(const T& h) override;

  // Vector used in error estimate calculations.
  VectorX<T> err_est_vec_;

  // Vector used to save initial value of xc.
  VectorX<T> save_xc0_;

  // These are pre-allocated temporaries for use by integration. They store
  // the derivatives computed at various points within the integration
  // interval.
  std::unique_ptr<ContinuousState<T>> derivs1_, derivs2_, derivs3_, derivs4_;
};

/**
 * Bogacki-Shampine-specific initialization function.
 * @throws std::logic_error if *neither* the initial step size target nor the
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

  // TODO(sherm1) Consider moving this notation description to IntegratorBase
  //              when it is more widely adopted.
  // Notation: we're using numeric subscripts for times t₀ and t₁, and
  // lower-case letter superscripts like t⁽ᵃ⁾ and t⁽ᵇ⁾ to indicate values
  // for intermediate stages of which there are two here, a and b.
  // State x₀ = {xc₀, xd₀, xa₀}. We modify only t and xc here, but
  // derivative calculations depend on everything in the context, including t,
  // x and inputs u (which may depend on t and x).
  // Define x⁽ᵃ⁾ ≜ {xc⁽ᵃ⁾, xd₀, xa₀} and u⁽ᵃ⁾ ≜ u(t⁽ᵃ⁾, x⁽ᵃ⁾).

  // Notation: we use Butcher tableaux notation

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

  // Compute the propagated solution.
  const double c4 = 1.0;
  const double a41 = 2.0 / 9;
  const double a42 = 1.0 / 3;
  const double a43 = 4.0 / 9;
  // This call marks t- and xc-dependent cache entries out of date, including
  // the derivative cache entry. (We already have the xc reference but must
  // issue the out-of-date notification here since we're about to change it.)
  context.SetTimeAndNoteContinuousStateChange(t0 + c4 * h);

  // Evaluate the derivative (denoted k4) at t₀ + c4 * h,
  //   xc₀ + a41 * h * k1 + a42 * h * k2 + a43 * h * k3.
  xc.SetFromVector(save_xc0_);  // Restore xc ← xc₀.
  xc.PlusEqScaled({{a41 * h, k1}, {a42 * h, k2}, {a43 * h, k3}});
  const VectorX<T> xc_t1 = xc.CopyToVector();
  derivs4_->get_mutable_vector().SetFrom(
      this->EvalTimeDerivatives(context).get_vector());
  const VectorBase<T>& k4 =  derivs4_->get_vector();

  // Calculate the 2nd-order solution that will be used for the error estimate.
  const double d1 = 7.0 / 24;
  const double d2 = 1.0 / 4;
  const double d3 = 1.0 / 3;
  const double d4 = 1.0 / 8;
  xc.SetFromVector(save_xc0_);
  xc.PlusEqScaled({{d1 * h, k1}, {d2 * h, k2}, {d3 * h, k3} , {d4 * h, k4}});

  // Compute the error estimate.
  err_est_vec_ = xc_t1 - xc.CopyToVector();
  err_est_vec_ = err_est_vec_.cwiseAbs();

  // If the size of the system has changed, the error estimate will no longer
  // be sized correctly. Verify that the error estimate is the correct size.
  DRAKE_DEMAND(this->get_error_estimate()->size() == xc.size());

  // Set the final state (the propagated solution) at t₁.
  xc.SetFromVector(xc_t1);
  this->get_mutable_error_estimate()->SetFromVector(err_est_vec_);

  // Bogacki-Shampine always succeeds in taking its desired step.
  return true;
}

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::systems::BogackiShampine3Integrator)

