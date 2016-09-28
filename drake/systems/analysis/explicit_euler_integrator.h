#pragma once

#include "drake/systems/analysis/integrator_base.h"

namespace drake {
namespace systems {

/**
 * A first-order, explicit Euler integrator. Configuration (q) and velocity (v)
 * are both updated using last configuration and velocity:
 * q(t+h) = q(t) + v(t)*h
 * v(t+h) = v(t) + dv/dt(q(t), v(t)) * h
 */
template <class T>
class ExplicitEulerIntegrator : public IntegratorBase<T> {
 public:
  virtual ~ExplicitEulerIntegrator() {}


  ExplicitEulerIntegrator(const System<T>& system, double step_size,
                          Context<T>* context = nullptr)
      : IntegratorBase<T>(system, context) {
    step_size_ = step_size;
    derivs_ = system.AllocateTimeDerivatives();
  }

  typename IntegratorBase<T>::StepResult Step(const T& dt) override;

  /**
   * No accuracy setting for Euler integrator.
   * @param accuracy unused
   */
  void set_target_accuracy(double accuracy) override {
    throw std::logic_error(
        "Accuracy setting not available"
            " for explicit Euler"
            " integrator");
  }

  /**
   * No accuracy setting for Euler integrator.
   */
  double get_target_accuracy() const override {
    throw std::logic_error(
        "Accuracy setting not available"
            " for explicit Euler"
            " integrator");
  }

  /**
   *   Gets the fixed step size.
   */
  const T& get_ideal_next_step_size() const override { return step_size_; }

  /**
   *   Sets the fixed step size.
   *   @param step_size the fixed step size
   */
  void set_fixed_step_size(const T& step_size) { step_size_ = step_size; }

  /**
   *   Gets the fixed step size.
   */
  const T& get_fixed_step_size() const { return step_size_; }

  /** Request that the first attempted integration step have a particular size.
   * All steps will be taken at this step size.
   **/
  void request_initial_step_size_target(const T& step_size) override {
    step_size_ = step_size;
  }

  /** Get the first integration step target size. Equal to the fixed step size.
   **/
  const T& get_initial_step_size_target() const override { return step_size_; }

 private:
  // The step size
  T step_size_{0};

  // These are pre-allocated temporaries for use by integration
  std::unique_ptr<ContinuousState<T>> derivs_;
};  // ExplictEulerIntegrator

/**
 * Integrates the system forward in time by dt. Integrator must already have
 * been initialized or exception will be thrown.
 * @param dt the integration step size, dt >= 0.0 (exception will be thrown
 *        if this is not the case).
 * @return the reason for the integration step ending
 */
template <class T>
typename IntegratorBase<T>::StepResult ExplicitEulerIntegrator<T>::Step(
    const T& dt) {

  if (dt < 0.0)
    throw std::logic_error("Negative dt.");
  if (!IntegratorBase<T>::is_initialized())
    throw std::logic_error("Integrator not initialized.");

  // Find the continuous state xc within the Context, just once.
  auto context = IntegratorBase<T>::get_mutable_context();
  VectorBase<T>* xc = context->get_mutable_state()->
      get_mutable_continuous_state()->get_mutable_state();

  // TODO(sherm1) This should be calculating into the cache so that
  // Publish() doesn't have to recalculate if it wants to output derivatives.
  IntegratorBase<T>::get_system().EvalTimeDerivatives(
      *IntegratorBase<T>::get_mutable_context(), derivs_.get());

  // compute derivative and update configuration and velocity
  // xc(t+h) = xc(t) + dt * xcdot(t, xc(t), xd(t+), u(t))
  const auto& xcdot = derivs_->get_state();
  xc->PlusEqScaled(dt, xcdot);  // xc += dt * xcdot
  context->set_time(context->get_time() + dt);

  // We successfully took a step -- collect statistics.
  IntegratorBase<T>::set_num_steps_taken(
      IntegratorBase<T>::get_num_steps_taken()+1);
  if (IntegratorBase<T>::get_num_steps_taken() == 1) {  // The first step.
    IntegratorBase<T>::set_actual_initial_step_size_taken(dt);
    IntegratorBase<T>::set_smallest_step_size_taken(dt);
    IntegratorBase<T>::set_largest_step_size_taken(dt);
  } else {  // Not the first step.
    if (dt < IntegratorBase<T>::get_smallest_step_size_taken())
      IntegratorBase<T>::set_smallest_step_size_taken(dt);
    if (dt > IntegratorBase<T>::get_largest_step_size_taken())
      IntegratorBase<T>::set_largest_step_size_taken(dt);
  }

  return IntegratorBase<T>::kTimeHasAdvanced;
}
}  // systems
}  // drake

