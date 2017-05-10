#pragma once

#include <cmath>

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/witness_function.h"

namespace drake {
namespace systems {
namespace analysis_test {

template <class T>
class LogisticSystem;

/// Witness function for determining when the state of the logistic system
/// crosses zero.
template <class T>
class LogisticWitness : public systems::WitnessFunction<T>
{
 public:
  LogisticWitness(const LogisticSystem<T>* system) :
    systems::WitnessFunction<T>(system), system_(system) {
  }

  // The witness function is simply the state value itself.
  T Evaluate(const Context<T>& context) override {
    return (*context.get_continuous_state())[0];
  }

  // Triggering this witness function will only yield a publish event.
  typename systems::DiscreteEvent<T>::ActionType get_action_type()
      const override {
    return systems::DiscreteEvent<T>::kPublishAction;
  }

  // Trigger the witness function whenever the state crosses zero.
  typename systems::WitnessFunction<T>::TriggerType get_trigger_type()
      const override {
    return systems::WitnessFunction<T>::TriggerType::kCrossesZero; }

  // Tight time isolation.
  T get_time_isolation_tolerance() const override { return 1e-8; }

  // Relatively thick dead band.
  T get_positive_dead_band() const override { return 1e-4; }
  T get_negative_dead_band() const override { return -1e-4; }

  // Naive trigger time bisects the two times.
  T do_get_trigger_time(const std::pair<T, T>& time_and_witness_value0,
                        const std::pair<T, T>& time_and_witness_valuef)
                        const override {
    return (time_and_witness_value0.first + time_and_witness_valuef.first)/2;
  }

  //.Pointer to the system
  const LogisticSystem<T>* system_;
};

/// System with state evolution yielding a logistic function, for purposes of
/// witness function testing using the differential equation
/// dx/dt = α⋅(1 - (x/k)^ν)⋅t, where ν > 0 (affects the shape of the curve),
/// α > 0 (growth rate), and k is the upper asymptote.
template <class T>
class LogisticSystem : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LogisticSystem)

  LogisticSystem(double k, double alpha, double nu) : k_(k), alpha_(alpha),
      nu_(nu) {
    this->DeclareDiscreteUpdatePeriodSec(1);
    this->DeclareContinuousState(1);
    witness_ = std::make_unique<LogisticWitness<T>>(this);
  }

  void DoCalcTimeDerivatives(const systems::Context<T>& context,
    systems::ContinuousState<T>* continuous_state) const override {
    // Get the current time.
    const T& t = context.get_time();

    // Get state.
    const T& x = (*context.get_continuous_state())[0];

    // Compute the derivative.
    (*continuous_state)[0] = alpha_ * (1 - pow(x/k_,nu_)) * t;
  }

  void DoCalcOutput(const Context<T>& context,
                    SystemOutput<T>* output) const override {
  }

  std::vector<systems::WitnessFunction<T>*> get_witness_functions(
      const systems::Context<T>& context) const override {
    std::vector<systems::WitnessFunction<T>*> witness_vec = { witness_.get() };
    return witness_vec;
  }

  void DoPublish(
      const drake::systems::Context<double>& context) const override {
    if (publish_callback_ != nullptr) publish_callback_(context);
  }

  void set_publish_callback(
      std::function<void(const Context<double>&)> callback) {
    publish_callback_ = callback;
  }

 private:
  std::unique_ptr<LogisticWitness<T>> witness_;
  std::function<void(const Context<double>&)> publish_callback_{nullptr};

  // The upper asymptote on the logistic function.
  double k_{1.0};

  // The rate (> 0) at which the logistic function approaches the asymptote.
  double alpha_{1.0};

  // Parameter (> 0) that affects near which asymptote maximum growth occurs.
  double nu_{1.0};
};

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake
