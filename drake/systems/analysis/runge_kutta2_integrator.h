#pragma once

#include "drake/systems/analysis/integrator_base.h"
#include "integrator_base.h"

namespace drake {
namespace systems {

/**
 * A second-order, explicit Runge Kutta integrator. Configuration (q) and
 * velocity (v) are both updated using the following formula:
 * q(t+h) = q(t) + v(t)*h
 * v(t+h) = v(t) + dv/dt(q(t), v(t)) * h
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
 * @param pointer to the context (nullptr is ok)
 */
  RungeKutta2Integrator(const System<T>& system, const T& max_step_size,
                        Context<T>* context = nullptr) :
      IntegratorBase<T>(system, context) {
    IntegratorBase<T>::set_maximum_step_size(max_step_size);
    derivs0_ = IntegratorBase<T>::get_system().AllocateTimeDerivatives();
    derivs1_ = IntegratorBase<T>::get_system().AllocateTimeDerivatives();
  }

  typename IntegratorBase<T>::StepResult Step(const T& publish_dt,
                                              const T& update_dt) override;

  /**
   * Integrator does not support accuracy estimation.
   */
  bool does_support_accuracy_estimation() const override { return false; }

  /**
   * Integrator does not support error control.
   */
  bool does_support_error_control() const override { return false; }

  /**
 * Integrator does not support an initial step size target.
 */
  void request_initial_step_size_target(const T& step_size) override {
    drake::log()->warn("RungeKutta2Integrator does not support step size"
                           " targets");
  }

  /**
   * No accuracy setting for RK2 integrator.
   * @param accuracy unused
   */
  void set_target_accuracy(const T& accuracy) override {
    throw std::runtime_error("Accuracy setting not available"
                                 " for RungeKutta2"
                                 " integrator");
  }

  /**
   * No accuracy setting for RK2 integrator.
   */
  const T& get_target_accuracy() const override {
    throw std::logic_error("Accuracy setting not available"
                                 " for RungeKutta2"
                                 " integrator");
  }

 private:
  // These are pre-allocated temporaries for use by integration
  std::unique_ptr<ContinuousState<T>> derivs0_, derivs1_;
};  // ExplictEulerIntegrator

template <class T>
typename IntegratorBase<T>::StepResult RungeKutta2Integrator<T>::Step(
    const T& publish_dt, const T& update_dt) {

  // sort the times for stopping- sort is stable to preserve preferences for
  // stopping
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
  VectorBase<T>* xc =
      context->get_mutable_state()->get_mutable_continuous_state()->
          get_mutable_state();

  // TODO(sherm1) This should be calculating into the cache so that
  // Publish() doesn't have to recalculate if it wants to output derivatives.
  IntegratorBase<T>::get_system().EvalTimeDerivatives(
      *IntegratorBase<T>::get_mutable_context(), derivs0_.get());

  // First stage is an explicit Euler step:
  // xc(t+h) = xc(t) + dt * xcdot(t, xc(t), xd(t+), u(t))
  const auto& xcdot0 = derivs0_->get_state();
  xc->PlusEqScaled(dt, xcdot0);  // xc += dt * xcdot0
  T t = IntegratorBase<T>::get_context().get_time() + dt;
  IntegratorBase<T>::get_mutable_context()->set_time(t);

  // use derivative at t+dt
  IntegratorBase<T>::get_system().EvalTimeDerivatives(
      *IntegratorBase<T>::get_mutable_context(), derivs1_.get());
  const auto& xcdot1 = derivs1_->get_state();

  // TODO(sherm1) Use better operators when available.
  xc->PlusEqScaled(dt * 0.5, xcdot1);
  xc->PlusEqScaled(-dt * 0.5, xcdot0);

  // We successfully took a step -- collect statistics.
  IntegratorBase<T>::UpdateStatistics(dt);

  // return appropriately
  if (stop_dts[0] == &max_step_size) {
    return IntegratorBase<T>::kTimeHasAdvanced;
  } else {
    if (stop_dts[0] == &publish_dt) {
      return IntegratorBase<T>::kReachedPublishTime;
    } else {
      DRAKE_ASSERT(stop_dts[0] == &update_dt);
      return IntegratorBase<T>::kReachedUpdateTime;
    }
  }
}  // Step(.)
}  // systems
}  // drake

