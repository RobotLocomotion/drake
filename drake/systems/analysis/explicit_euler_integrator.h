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
   * Constructs fixed-step integrator for a given system using the given
   * context for initial conditions.
   * @param system A reference to the system to be simulated
   * @param max_step_size The maximum (fixed) step size; the integrator will
   *                      not take larger step sizes than this.
   * @param context Pointer to the context (nullptr is ok).
   */
  ExplicitEulerIntegrator(const System<T>& system, const T& max_step_size,
                          Context<T>* context = nullptr)
      : IntegratorBase<T>(system, context) {
    IntegratorBase<T>::set_maximum_step_size(max_step_size);
    derivs_ = system.AllocateTimeDerivatives();
  }

  typename IntegratorBase<T>::StepResult Step(const T& publish_dt,
                                              const T& update_dt) override;

  /**
   * Integrator does not support accuracy estimation.
   */
  bool supports_accuracy_estimation() const override { return false; }

  /**
   * Integrator does not support error control.
   */
  bool supports_error_control() const override { return false; }

 private:
  // These are pre-allocated temporaries for use by integration
  std::unique_ptr<ContinuousState<T>> derivs_;
};  // ExplictEulerIntegrator

/**
 * Integrates the system forward in time. Integrator must already have
 * been initialized or a std::logic_error exception will be thrown. The
 * Step(.) method for the explicit Euler integrator will integrate forward
 * by the maximum of { `publish_dt`, `update_dt`, `get_maximum_step_size()` }.
 * @param publish_dt the step size, >= 0.0 (exception will be thrown
 *        if this is not the case) at which the next publish is desired
 * @param update_dt the step size, >= 0.0 (exception will be thrown
 *        if this is not the case) at which the next update is scheduled
 * @returns The reason for the integration step ending.
 */
template <class T>
typename IntegratorBase<T>::StepResult ExplicitEulerIntegrator<T>::Step(
    const T& publish_dt, const T& update_dt) {

  // TODO(edrumwri): Separate code into reusable chunks once variable
  // step integration introduced

  // Sort the times for stopping- sort is stable to preserve preferences for
  // stopping. In decreasing order of preference for equal values, we want
  // the update step, then the publish step, then the maximum step size.
  const int64_t kDTs = 3;  // number of dt values to evaluate
  const T& max_step_size = IntegratorBase<T>::get_maximum_step_size();
  const T* stop_dts[kDTs] = { &update_dt, &publish_dt,  &max_step_size};
  std::stable_sort(stop_dts, stop_dts+kDTs,
                   [](const T* t1, const T* t2) { return *t1 < *t2; });

  // set dt
  const T& dt = *stop_dts[0];

  if (dt < 0.0)
    throw std::logic_error("Negative dt.");
  if (!IntegratorBase<T>::is_initialized())
    throw std::logic_error("Integrator not initialized.");

  // Find the continuous state xc within the Context, just once.
  auto context = IntegratorBase<T>::get_mutable_context();
  VectorBase<T>* xc = context->get_mutable_continuous_state()->
      get_mutable_state();

  // TODO(sherm1) This should be calculating into the cache so that
  // Publish() doesn't have to recalculate if it wants to output derivatives.
  IntegratorBase<T>::get_system().EvalTimeDerivatives(
      IntegratorBase<T>::get_context(), derivs_.get());

  // Compute derivative and update configuration and velocity.
  // xc(t+h) = xc(t) + dt * xcdot(t, xc(t), u(t))
  const auto& xcdot = derivs_->get_state();
  xc->PlusEqScaled(dt, xcdot);  // xc += dt * xcdot
  context->set_time(context->get_time() + dt);

  IntegratorBase<T>::UpdateStatistics(dt);

  // Return depending on the step taken.
  if (stop_dts[0] == &max_step_size)
    return IntegratorBase<T>::kTimeHasAdvanced;
  if (stop_dts[0] == &publish_dt)
    return IntegratorBase<T>::kReachedPublishTime;
  if (stop_dts[0] == &update_dt)
    return IntegratorBase<T>::kReachedUpdateTime;
  DRAKE_ABORT_MSG("Never should have reached here.");
}  // Step(.)
}  // systems
}  // drake

