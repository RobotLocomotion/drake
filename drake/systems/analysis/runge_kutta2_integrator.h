//
// Created by drum on 9/14/16.
//

#ifndef DRAKE_SUPERBUILD_RK2_INTEGRATOR_H
#define DRAKE_SUPERBUILD_RK2_INTEGRATOR_H

#include "drake/systems/analysis/integrator_base.h"
#include "integrator_base.h"

namespace drake {
namespace systems {

template <class T>
class RungeKutta2Integrator : public IntegratorBase<T> {
 public:
  virtual ~RungeKutta2Integrator() {}

  RungeKutta2Integrator(const System<T>& system, double step_size,
                        Context<T>* context = NULL) :
      IntegratorBase<T>(system, context) {
    step_size_ = step_size;
    derivs0_ = IntegratorBase<T>::system_.AllocateTimeDerivatives();
    derivs1_ = IntegratorBase<T>::system_.AllocateTimeDerivatives();
  }

  virtual typename IntegratorBase<T>::StepResult Step(const T& dt);

  /// No accuracy setting for RK2 integrator
  virtual void set_target_accuracy(double accuracy) {
    throw std::runtime_error("Accuracy setting not available"
                                 " for RungeKutta2"
                                 " integrator");
  }

  /// No accuracy setting for RK2 integrator
  virtual double get_target_accuracy() const {
    throw std::runtime_error("Accuracy setting not available"
                                 " for RungeKutta2"
                                 " integrator");
  }

  /// Gets the fixed step size
  virtual const T& get_ideal_next_step_size() const { return step_size_; }

  /// Sets the fixed step size
  void set_fixed_step_size(const T& step_size) { step_size_ = step_size; }

  /// Gets the fixed step size
  const T& get_fixed_step_size() const { return step_size_; }

  /** Request that the first attempted integration step have a particular size.
     All steps will be taken at this step size.
   **/
  virtual void request_initial_step_size_target(const T& step_size) {
    step_size_ = step_size;
  }

  /** Get the first integration step target size. Equal to the fixed step size.
   **/
  virtual const T& get_initial_step_size_target() const {
    return step_size_;
  }

 private:
  // These are pre-allocated temporaries for use by integration
  std::unique_ptr<ContinuousState<T>> derivs0_, derivs1_;

  /// The integration step size
  T step_size_{(T) 0.0};
};  // ExplictEulerIntegrator

template <class T>
typename IntegratorBase<T>::StepResult RungeKutta2Integrator<T>::Step(
    const T& dt) {
  DRAKE_THROW_UNLESS(dt >= 0.0);
  DRAKE_THROW_UNLESS(IntegratorBase<T>::initialization_done_);

  // Find the continuous state xc within the Context, just once.
  auto& context = IntegratorBase<T>::context_;
  VectorBase<T>* xc =
      context->get_mutable_state()->continuous_state->get_mutable_state();

  // TODO(sherm1) This should be calculating into the cache so that
  // Publish() doesn't have to recalculate if it wants to output derivatives.
  IntegratorBase<T>::system_.EvalTimeDerivatives(*IntegratorBase<T>::context_,
                                                 derivs0_.get());

  // First stage is an explicit Euler step:
  // xc(t+h) = xc(t) + dt * xcdot(t, xc(t), xd(t+), u(t))
  const auto& xcdot0 = derivs0_->get_state();
  xc->PlusEqScaled(dt, xcdot0);  // xc += dt * xcdot0
  T t = IntegratorBase<T>::context_->get_time() + dt;
  IntegratorBase<T>::context_->set_time(t);

  // use derivative at t+dt
  IntegratorBase<T>::system_.EvalTimeDerivatives(*IntegratorBase<T>::context_,
                                                 derivs1_.get());
  const auto& xcdot1 = derivs1_->get_state();

  // TODO(sherm1) Use better operators when available.
  xc->PlusEqScaled(dt * 0.5, xcdot1);
  xc->PlusEqScaled(-dt * 0.5, xcdot0);

  // We successfully took a step -- collect statistics.
  if (++IntegratorBase<T>::num_steps_taken_ == 1) {  // The first step.
    IntegratorBase<T>::actual_initial_step_size_taken_ = dt;
    IntegratorBase<T>::smallest_step_size_taken_ = dt;
    IntegratorBase<T>::largest_step_size_taken_ = dt;
  } else {  // Not the first step.
    if (dt < IntegratorBase<T>::smallest_step_size_taken_)
      IntegratorBase<T>::smallest_step_size_taken_ = dt;
    if (dt > IntegratorBase<T>::largest_step_size_taken_)
      IntegratorBase<T>::largest_step_size_taken_ = dt;
  }

  return IntegratorBase<T>::kTimeHasAdvanced;
}
}  // systems
}  // drake

#endif  // DRAKE_SUPERBUILD_EXPLICIT_EULER_INTEGRATOR_H
