#pragma once

#include <cmath>

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/witness_function.h"

namespace drake {
namespace systems {
namespace analysis_test {

template <class T>
class SquareWaveSystem;

template <class T>
class SquareWaveWitness : public systems::WitnessFunction<T>
{
 public:
  SquareWaveWitness(const SquareWaveSystem<T>* system) :
    systems::WitnessFunction<T>(system), system_(system) {
  }

  // The witness function is simply the state value itself.
  T Evaluate(const Context<T>& context) {
    return context.get_discrete_state(0)->get_value()(0);
  }

  typename systems::DiscreteEvent<T>::ActionType get_action_type()
      const override {
    return systems::DiscreteEvent<T>::kPublishAction;
  }

  typename systems::WitnessFunction<T>::TriggerType get_trigger_type()
      const override {
    return systems::WitnessFunction<T>::TriggerType::kCrossesZero; }

  T get_time_isolation_tolerance() const override { return 1e-8; }
  T get_positive_dead_band() const override { return 1e-8; }
  T get_negative_dead_band() const override { return -1e-8; }


  // Naive trigger time bisects the two times.
  T do_get_trigger_time(const std::pair<T, T>& time_and_witness_value0,
                        const std::pair<T, T>& time_and_witness_valuef)
                        const override {
    return (time_and_witness_value0.first + time_and_witness_valuef.first)/2;
  }

  //.Pointer to the system
  SquareWaveSystem<T>* system_;
};

/// System with state that represents a square wave for witness function
/// testing. The square wave has a period of one second.
template <class T>
class SquareWaveSystem : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SquareWaveSystem)

  SquareWaveSystem(const T& vertical_offset) {
    this->DeclareDiscreteUpdatePeriodSec(1);
    this->DeclareDiscreteState(1);
    vertical_offset_ = vertical_offset;
    witness_ = std::make_unique<SquareWaveWitness<T>>(this);
  }

  void DoCalcDiscreteVariableUpdates(const systems::Context<T>& context,
    systems::DiscreteValues<T>* discrete_state) const {
    // Get state.
    auto& x = discrete_state->get_value()(0);
    x = (fmod(context.get_time(), 2) > 1) ? vertical_offset_ :
                                            vertical_offset_ + 1;
  }

  void DoCalcOutput(const Context<T>& context,
                    SystemOutput<T>* output) const override {
  }

  std::vector<systems::WitnessFunction<T>*> get_witness_functions(
      const systems::Context<T>& context) const {
    std::vector<systems::WitnessFunction<T>*> witness_vec = { witness_.get() };
  }

 private:
   std::unique_ptr<SquareWaveWitness<T>> witness_;
   int vertical_offset_{1};
};

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake
