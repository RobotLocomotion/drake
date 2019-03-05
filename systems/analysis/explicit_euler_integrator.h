#pragma once

#include <memory>

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
  bool DoStep(const T& dt) override;
};

/**
 * Integrates the system forward in time by dt, starting at the current time t₀.
 * This value of dt is determined by IntegratorBase::Step().
 */
template <class T>
bool ExplicitEulerIntegrator<T>::DoStep(const T& dt) {
  Context<T>& context = *this->get_mutable_context();

  // Evaluate derivative xcdot₀ ← xcdot(t₀, x(t₀), u(t₀)).
  const ContinuousState<T>& xc_deriv = this->EvalTimeDerivatives(context);
  const VectorBase<T>& xcdot0 = xc_deriv.get_vector();

  // Update continuous state and time.
  // This call invalidates t- and xc-dependent cache entries.
  VectorBase<T>& xc = context.SetTimeAndGetMutableContinuousStateVector(
      context.get_time() + dt);  // t ← t₀ + h
  xc.PlusEqScaled(dt, xcdot0);   // xc(t₀ + h) ← xc(t₀) + dt * xcdot₀

  // This integrator always succeeds at taking the step.
  return true;
}

}  // namespace systems
}  // namespace drake

