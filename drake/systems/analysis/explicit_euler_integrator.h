#pragma once

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
class ExplicitEulerIntegrator : public IntegratorBase<T> {
 public:
  virtual ~ExplicitEulerIntegrator() {}

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
    derivs_ = system.AllocateTimeDerivatives();
  }


  /**
   * Explicit Euler integrator does not support accuracy estimation.
   */
  bool supports_accuracy_estimation() const override { return false; }

  /**
   * Explicit Euler ntegrator does not support error control.
   */
  bool supports_error_control() const override { return false; }

 private:
  bool DoStep(const T& dt) override;

  // These are pre-allocated temporaries for use by integration
  std::unique_ptr<ContinuousState<T>> derivs_;
};

/**
 * Integrates the system forward in time by dt. This value is determined
 * by IntegratorBase::Step().
 */
template <class T>
bool ExplicitEulerIntegrator<T>::DoStep(const T& dt) {
  // Find the continuous state xc within the Context, just once.
  auto context = IntegratorBase<T>::get_mutable_context();
  VectorBase<T>* xc = context->get_mutable_continuous_state_vector();

  // TODO(sherm1) This should be calculating into the cache so that
  // Publish() doesn't have to recalculate if it wants to output derivatives.
  IntegratorBase<T>::get_system().EvalTimeDerivatives(
      IntegratorBase<T>::get_context(), derivs_.get());

  // Compute derivative and update configuration and velocity.
  // xc(t+h) = xc(t) + dt * xcdot(t, xc(t), u(t))
  const auto& xcdot = derivs_->get_vector();
  xc->PlusEqScaled(dt, xcdot);  // xc += dt * xcdot
  context->set_time(context->get_time() + dt);

  IntegratorBase<T>::UpdateStatistics(dt);

  // Fixed step integrator always returns true
  return true;
}
}  // namespace systems
}  // namespace drake

