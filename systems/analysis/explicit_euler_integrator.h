#pragma once

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/analysis/integrator_base.h"

namespace drake {
namespace systems {

/**
 * A first-order, explicit Euler integrator. State is updated in the following
 * manner:
 * <pre>
 * x(t+h) = x(t) + dx/dt * h
 * </pre>
 *
 * @tparam_default_scalar
 * @ingroup integrators
 */
template <class T>
class ExplicitEulerIntegrator final : public IntegratorBase<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ExplicitEulerIntegrator)

  ~ExplicitEulerIntegrator() override = default;

  /**
   * Constructs a fixed-step integrator for a given system using the given
   * context for initial conditions.
   * @param system A reference to the system to be simulated
   * @param max_step_size The maximum (fixed) step size; the integrator will
   *                      not take larger step sizes than this.
   * @param context Pointer to the context (nullptr is ok, but the caller
   *                must set a non-null context before Initialize()-ing the
   *                integrator).
   * @sa Initialize()
   */
  ExplicitEulerIntegrator(const System<T>& system, const T& max_step_size,
                          Context<T>* context = nullptr)
      : IntegratorBase<T>(system, context) {
    IntegratorBase<T>::set_maximum_step_size(max_step_size);
  }

  /**
   * Explicit Euler integrator does not support error estimation.
   */
  bool supports_error_estimation() const override { return false; }

  /// Integrator does not provide an error estimate.
  int get_error_estimate_order() const override { return 0; }

 private:
  bool DoStep(const T& h) override;
};

/**
 * Integrates the system forward in time by h, starting at the current time t₀.
 * This value of h is determined by IntegratorBase::Step().
 */
template <class T>
bool ExplicitEulerIntegrator<T>::DoStep(const T& h) {
  Context<T>& context = *this->get_mutable_context();

  // CAUTION: This is performance-sensitive inner loop code that uses dangerous
  // long-lived references into state and cache to avoid unnecessary copying and
  // cache invalidation. Be careful not to insert calls to methods that could
  // invalidate any of these references before they are used.

  // Evaluate derivative xcdot₀ ← xcdot(t₀, x(t₀), u(t₀)).
  const ContinuousState<T>& xc_deriv = this->EvalTimeDerivatives(context);
  const VectorBase<T>& xcdot0 = xc_deriv.get_vector();

  // Cache: xcdot0 references the live derivative cache value, currently
  // up to date but about to be marked out of date. We do not want to make
  // an unnecessary copy of this data.

  // Update continuous state and time. This call marks t- and xc-dependent
  // cache entries out of date, including xcdot0.
  VectorBase<T>& xc = context.SetTimeAndGetMutableContinuousStateVector(
      context.get_time() + h);  // t ← t₀ + h

  // Cache: xcdot0 still references the derivative cache value, which is
  // unchanged, although it is marked out of date.

  xc.PlusEqScaled(h, xcdot0);   // xc(t₀ + h) ← xc(t₀) + h * xcdot₀

  // This integrator always succeeds at taking the step.
  return true;
}

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::systems::ExplicitEulerIntegrator)
