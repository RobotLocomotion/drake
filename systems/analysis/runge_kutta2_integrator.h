#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/systems/analysis/integrator_base.h"

namespace drake {
namespace systems {

/**
 * A second-order, explicit Runge Kutta integrator.
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
 * @param pointer to the context (nullptr is ok, but the caller
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
  bool DoStep(const T& dt) override;

  // A pre-allocated temporary for use by integration.
  std::unique_ptr<ContinuousState<T>> derivs0_;
};

/**
 * Integrates the system forward in time from the current time t₀ to
 * t₁ = t₀ + dt. The value of dt is determined by IntegratorBase::Step().
 *
 * The Butcher tableaux for this integrator follows: <pre>
 *
 *     0  |
 *     1  | 1
 *     -----------------
 *          1/2     1/2
 * </pre>
 */
template <class T>
bool RungeKutta2Integrator<T>::DoStep(const T& dt) {
  Context<T>* const context = IntegratorBase<T>::get_mutable_context();

  // Evaluate derivative xcdot(t₀) ← xcdot(t₀, x(t₀), u(t₀)). Copy the result
  // into a temporary since we'll be calculating another derivative below.
  derivs0_->get_mutable_vector().SetFrom(
      this->EvalTimeDerivatives(*context).get_vector());
  const VectorBase<T>& xcdot0 = derivs0_->get_vector();

  // First stage is an explicit Euler step:
  // xc(t+h) = xc(t) + dt * xcdot(t, xc(t), u(t))
  // This call invalidates t- and xc-dependent cache entries.
  VectorBase<T>& xc = context->SetTimeAndGetMutableContinuousStateVector(
      context->get_time() + dt);  // t ← t₁ = t₀ + h
  xc.PlusEqScaled(dt, xcdot0);    // xc' = xc₀ + dt * xcdot0

  // Evaluate derivative xcdot₁ ← xcdot(t₁, x', u').
  const VectorBase<T>& xcdot1 =
      this->EvalTimeDerivatives(*context).get_vector();

  // Invalidates xc-dependent context entries; time doesn't change here.
  context->NoteContinuousStateChange();
  // xc₁ = xc₀ + dt * (xcdot0 + xcdot1)/2 = xc' + dt * (xcdot1 - xcdot0)/2
  xc.PlusEqScaled({{dt / 2, xcdot1}, {-dt / 2, xcdot0}});

  // RK2 always succeeds at taking the step.
  return true;
}
}  // namespace systems
}  // namespace drake

