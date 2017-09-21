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
class StatelessSystem;

/// Witness function for determining when the time of the empty system
/// crosses zero. The witness function is just the time in the context.
template <class T>
class ClockWitness : public WitnessFunction<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ClockWitness)

  explicit ClockWitness(
      double trigger_time,
      const System<T>& system,
      const WitnessFunctionDirection& dir_type) :
        WitnessFunction<T>(system, dir_type),
        trigger_time_(trigger_time) {
  }

  /// Get the time at which this witness triggers.
  double get_trigger_time() const { return trigger_time_; }

 protected:
  // The witness function is the time value itself plus the offset value.
  T DoEvaluate(const Context<T>& context) const override {
    return context.get_time() - trigger_time_;
  }

  void DoAddEvent(CompositeEventCollection<T>* events) const override {
    events->add_publish_event(std::make_unique<PublishEvent<T>>(
        Event<T>::TriggerType::kWitness));
  }

 private:
  // The time at which the witness function is to trigger.
  const double trigger_time_;
};

/// System with no state for testing a simplistic witness function.
template <class T>
class StatelessSystem final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StatelessSystem)

  StatelessSystem(double offset, const WitnessFunctionDirection& dir_type)
      : LeafSystem<T>(SystemTypeTag<analysis_test::StatelessSystem>{}) {
    witness_ = std::make_unique<ClockWitness<T>>(offset, *this, dir_type);
  }

  /// Scalar-converting copy constructor. See @ref system_scalar_conversion.
  /// @note This function does not preserve the publish callback because
  ///       this is test code for which it is expected that no one will care
  ///       whether the publish callback survives transmogrification.
  template <typename U>
  explicit StatelessSystem(const StatelessSystem<U>& other)
      : StatelessSystem<T>(other.witness_->get_trigger_time(),
                           other.witness_->get_dir_type()) {}

  void set_publish_callback(
      std::function<void(const Context<T>&)> callback) {
    publish_callback_ = callback;
  }

 protected:
  void DoGetWitnessFunctions(
      const Context<T>&,
      std::vector<const WitnessFunction<T>*>* w) const override {
    w->push_back(witness_.get());
  }

  void DoPublish(
      const Context<T>& context,
      const std::vector<const PublishEvent<T>*>&) const override {
    if (publish_callback_ != nullptr) publish_callback_(context);
  }

 private:
  // Allow different specializations to access each other's private data.
  template <typename> friend class StatelessSystem;

  std::unique_ptr<ClockWitness<T>> witness_;
  std::function<void(const Context<T>&)> publish_callback_{nullptr};
};

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake
