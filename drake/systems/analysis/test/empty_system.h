#pragma once

#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/witness_function.h"

namespace drake {
namespace systems {
namespace analysis_test {

template <class T>
class EmptySystem;

/// Witness function for determining when the time of the empty system
/// crosses zero. The witness function is just the time in the context.
template <class T>
class ClockWitness : public systems::WitnessFunction<T> {
 public:
  explicit ClockWitness(const EmptySystem<T>* system,
      const typename systems::WitnessFunction<T>::TriggerType& ttype) :
      systems::WitnessFunction<T>(ttype,
          systems::DiscreteEvent<T>::kPublishAction),
    system_(*system) {
  }

  // The witness function is simply the time value itself.
  T DoEvaluate(const Context<T>& context) override {
    return context.get_time();
  }

  // Pointer to the system.
  const EmptySystem<T>& system_;
};

/// System with no state evolution for testing a simplistic witness function.
template <class T>
class EmptySystem : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(EmptySystem)

  explicit EmptySystem(const typename systems::WitnessFunction<T>::TriggerType&
                         ttype) {
    witness_ = std::make_unique<ClockWitness<T>>(this, ttype);
  }

  void DoCalcOutput(const Context<T>& context,
                    SystemOutput<T>* output) const override {}

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
  std::unique_ptr<ClockWitness<T>> witness_;
  std::function<void(const Context<double>&)> publish_callback_{nullptr};
};

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake
