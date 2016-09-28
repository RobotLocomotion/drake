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

  RungeKutta2Integrator(const System<T>& system, double step_size,
                        Context<T>* context = nullptr) :
      IntegratorBase<T>(system, context) {
    step_size_ = step_size;
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
   * No accuracy setting for RK2 integrator.
   * @param accuracy unused
   */
  void set_target_accuracy(double accuracy) override {
    throw std::runtime_error("Accuracy setting not available"
                                 " for RungeKutta2"
                                 " integrator");
  }

  /**
   * No accuracy setting for RK2 integrator.
   */
  double get_target_accuracy() const override {
    throw std::logic_error("Accuracy setting not available"
                                 " for RungeKutta2"
                                 " integrator");
  }

  /**
   *   Gets the fixed step size.
   */
  const T& get_ideal_next_step_size() const override { return step_size_; }

  /**
   *   Sets the fixed step size.
   */
  void set_fixed_step_size(const T& step_size) { step_size_ = step_size; }

  /**
   *   Gets the fixed step size
   */
  const T& get_fixed_step_size() const { return step_size_; }

  /** Request that the first attempted integration step have a particular size.
     All steps will be taken at this step size.
   **/
  void request_initial_step_size_target(const T& step_size) override {
    step_size_ = step_size;
  }

  /** Get the first integration step target size. Equal to the fixed step size.
   **/
  const T& get_initial_step_size_target() const override {
    return step_size_;
  }

 private:
  // These are pre-allocated temporaries for use by integration
  std::unique_ptr<ContinuousState<T>> derivs0_, derivs1_;

  // The integration step size
  T step_size_{(T) 0.0};
};  // ExplictEulerIntegrator

template <class T>
typename IntegratorBase<T>::StepResult RungeKutta2Integrator<T>::Step(
    const T& publish_dt, const T& update_dt) {

  // set dt
  bool publish_stop = (publish_dt < update_dt);
  const T& dt = (publish_stop) ? publish_dt : update_dt;

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

  return (publish_stop) ? IntegratorBase<T>::kReachedPublishTime :
         IntegratorBase<T>::kReachedUpdateTime;
}
}  // systems
}  // drake

