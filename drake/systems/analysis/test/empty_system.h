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
  explicit ClockWitness(
      const T& trigger_time,
      const System<T>& system,
      const typename systems::WitnessFunction<T>::DirectionType& dir_type) :
        systems::WitnessFunction<T>(system, dir_type,
          systems::DiscreteEvent<T>::kPublishAction),
        trigger_time_(trigger_time) {
  }

 protected:
  // The witness function is the time value itself plus the offset value.
  T DoEvaluate(const Context<T>& context) const override {
    return context.get_time() - trigger_time_;
  }

 private:
  // The time at which the witness function is to trigger.
  T trigger_time_{0};
};

/// System with no state evolution for testing a simplistic witness function.
template <class T>
class EmptySystem : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(EmptySystem)

  explicit EmptySystem(const T& offset,
      const typename systems::WitnessFunction<T>::DirectionType& dir_type) {
    witness_ = std::make_unique<ClockWitness<T>>(offset, *this, dir_type);
  }

  void set_publish_callback(
      std::function<void(const Context<double>&)> callback) {
    publish_callback_ = callback;
  }

 protected:
  void DoGetWitnessFunctions(
      const systems::Context<T>&,
      std::vector<const systems::WitnessFunction<T>*>* w) const override {
    w->push_back(witness_.get());
  }

  void DoPublish(
      const drake::systems::Context<double>& context) const override {
    if (publish_callback_ != nullptr) publish_callback_(context);
  }

 private:
  std::unique_ptr<ClockWitness<T>> witness_;
  std::function<void(const Context<double>&)> publish_callback_{nullptr};
};

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake
