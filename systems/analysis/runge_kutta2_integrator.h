#pragma once

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/analysis/integrator_base.h"

namespace drake {
namespace systems {

/**
 * A second-order, explicit Runge Kutta integrator.
 * @tparam_default_scalar
 * @ingroup integrators
 */
template <class T>
class RungeKutta2Integrator final : public IntegratorBase<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RungeKutta2Integrator)

  ~RungeKutta2Integrator() override = default;

  /**
   * Constructs fixed-step integrator for a given system using the given
   * context for initial conditions.
   * @param system A reference to the system to be simulated
   * @param max_step_size The maximum (fixed) step size; the integrator will
   *                      not take larger step sizes than this.
   * @param context pointer to the context (nullptr is ok, but the caller
   *                must set a non-null context before Initialize()-ing the
   *                integrator).
   * @sa Initialize()
   */
  RungeKutta2Integrator(const System<T>& system, const T& max_step_size,
                        Context<T>* context = nullptr) :
      IntegratorBase<T>(system, context) {
    IntegratorBase<T>::set_maximum_step_size(max_step_size);
    derivs0_ = IntegratorBase<T>::get_system().AllocateTimeDerivatives();
  }

  /**
   * The RK2 integrator does not support error estimation.
   */
  bool supports_error_estimation() const override { return false; }

  /// Integrator does not provide an error estimate.
  int get_error_estimate_order() const override { return 0; }

 private:
  bool DoStep(const T& h) override;

  // A pre-allocated temporary for use by integration.
  std::unique_ptr<ContinuousState<T>> derivs0_;
};

/**
 * Integrates the system forward in time from the current time t₀ to
 * t₁ = t₀ + h. The value of h is determined by IntegratorBase::Step().
 *
 * The Butcher tableau for this integrator follows: <pre>
 *
 *     0  |
 *     1  | 1
 *     -----------------
 *          1/2     1/2
 * </pre>
 */
template <class T>
bool RungeKutta2Integrator<T>::DoStep(const T& h) {
  Context<T>* const context = IntegratorBase<T>::get_mutable_context();

  // CAUTION: This is performance-sensitive inner loop code that uses dangerous
  // long-lived references into state and cache to avoid unnecessary copying and
  // cache invalidation. Be careful not to insert calls to methods that could
  // invalidate any of these references before they are used.

  // TODO(sherm1) Consider moving this notation description to IntegratorBase
  //              when it is more widely adopted.
  // Notation: we're using numeric subscripts for times t₀ and t₁, and
  // lower-case letter superscripts like t⁽ᵃ⁾ and t⁽ᵇ⁾ to indicate values
  // for intermediate stages of which there is only one here, stage a.
  // State x₀ = {xc₀, xd₀, xa₀}. We modify only t and xc here, but
  // derivative calculations depend on everything in the context, including t,
  // x and inputs u (which may depend on t and x).
  // Define x⁽ᵃ⁾ ≜ {xc⁽ᵃ⁾, xd₀, xa₀} and u⁽ᵃ⁾ ≜ u(t⁽ᵃ⁾, x⁽ᵃ⁾).

  // Evaluate derivative xcdot₀ ← xcdot(t₀, x(t₀), u(t₀)). Copy the result
  // into a temporary since we'll be calculating another derivative below.
  derivs0_->get_mutable_vector().SetFrom(
      this->EvalTimeDerivatives(*context).get_vector());
  const VectorBase<T>& xcdot0 = derivs0_->get_vector();

  // Cache: xcdot0 references a *copy* of the derivative result so is immune
  // to subsequent evaluations.

  // First intermediate stage is an explicit Euler step. This call marks t-
  // and xc-dependent cache entries out of date, including the derivative
  // cache entry.
  VectorBase<T>& xc = context->SetTimeAndGetMutableContinuousStateVector(
      context->get_time() + h);  // t⁽ᵃ⁾ ← t₁ = t₀ + h
  xc.PlusEqScaled(h, xcdot0);    // xc⁽ᵃ⁾ ← xc₀ + h * xcdot₀

  // Evaluate derivative xcdot⁽ᵃ⁾ ← xcdot(t⁽ᵃ⁾, x⁽ᵃ⁾, u⁽ᵃ⁾).
  const VectorBase<T>& xcdot_a =
      this->EvalTimeDerivatives(*context).get_vector();

  // Cache: xcdot_a references the live derivative cache value, currently
  // up to date but about to be marked out of date. We do not want to make
  // an unnecessary copy of this data.

  // Cache: Because we captured a reference to xc above and now want to modify
  // it in place and recalculate, we must manually tell the caching system that
  // we've made that change since it is otherwise unobservable. There is an
  // advanced method available for this purpose.

  // Marks xc-dependent cache entries out of date, including xcdot_a; time
  // doesn't change here.
  context->NoteContinuousStateChange();

  // Cache: xcdot_a still references the derivative cache value, which is
  // unchanged, although it is marked out of date. xcdot0 is unaffected.

  // xc₁ = xc₀ + h * (xcdot₀ + xcdot⁽ᵃ⁾)/2
  //     = xc⁽ᵃ⁾ + h * (xcdot⁽ᵃ⁾ - xcdot₀)/2
  xc.PlusEqScaled({{h / 2, xcdot_a}, {-h / 2, xcdot0}});

  // RK2 always succeeds at taking the step.
  return true;
}
}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::systems::RungeKutta2Integrator)
