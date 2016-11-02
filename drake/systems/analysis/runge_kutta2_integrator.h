#pragma once

#include "drake/systems/analysis/integrator_base.h"

namespace drake {
namespace systems {

/**
 * A second-order, explicit Runge Kutta integrator.
 */
template <class T>
class RungeKutta2Integrator : public IntegratorBase<T> {
 public:
  virtual ~RungeKutta2Integrator() {}

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
    derivs1_ = IntegratorBase<T>::get_system().AllocateTimeDerivatives();
  }

  /**
   * The RK2 integrator does not support accuracy estimation.
   */
  bool supports_accuracy_estimation() const override { return false; }

  /**
   * The RK2 integrator does not support error control.
   */
  bool supports_error_control() const override { return false; }

 private:
  bool DoStep(const T& dt) override;

  // These are pre-allocated temporaries for use by integration
  std::unique_ptr<ContinuousState<T>> derivs0_, derivs1_;
};

/**
 * Integrates the system forward in time by dt. This value is determined
 * by IntegratorBase::Step().
 */
template <class T>
bool RungeKutta2Integrator<T>::DoStep(const T& dt) {
  // Find the continuous state xc within the Context, just once.
  auto context = IntegratorBase<T>::get_mutable_context();
  VectorBase<T>* xc = context->get_mutable_continuous_state_vector();

  // TODO(sherm1) This should be calculating into the cache so that
  // Publish() doesn't have to recalculate if it wants to output derivatives.
  IntegratorBase<T>::get_system().EvalTimeDerivatives(
      IntegratorBase<T>::get_context(), derivs0_.get());

  // First stage is an explicit Euler step:
  // xc(t+h) = xc(t) + dt * xcdot(t, xc(t), u(t))
  const auto& xcdot0 = derivs0_->get_vector();
  xc->PlusEqScaled(dt, xcdot0);  // xc += dt * xcdot0
  T t = IntegratorBase<T>::get_context().get_time() + dt;
  IntegratorBase<T>::get_mutable_context()->set_time(t);

  // use derivative at t+dt
  IntegratorBase<T>::get_system().EvalTimeDerivatives(
      *IntegratorBase<T>::get_mutable_context(), derivs1_.get());
  const auto& xcdot1 = derivs1_->get_vector();

  // TODO(sherm1) Use better operators when available.
  xc->PlusEqScaled(dt * 0.5, xcdot1);
  xc->PlusEqScaled(-dt * 0.5, xcdot0);

  IntegratorBase<T>::UpdateStatistics(dt);

  // Fixed step integrator always returns true
  return true;
}
}  // namespace systems
}  // namespace drake

