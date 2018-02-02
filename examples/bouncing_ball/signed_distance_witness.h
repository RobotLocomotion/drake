#pragma once

#include <memory>

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/event.h"
#include "drake/systems/framework/witness_function.h"

namespace drake {
namespace bouncing_ball {

template <class T>
class SignedDistanceWitnessFunction : public systems::WitnessFunction<T> {
 public:
  explicit SignedDistanceWitnessFunction(const systems::System<T>& system) :
      systems::WitnessFunction<T>(
          system, systems::WitnessFunctionDirection::kPositiveThenNonPositive) {
    event_ = std::make_unique<systems::UnrestrictedUpdateEvent<T>>(
      systems::Event<T>::TriggerType::kWitness);
  }
  ~SignedDistanceWitnessFunction() override {}

 private:
  T DoEvaluate(const systems::Context<T>& context) const override {
    const systems::VectorBase<T>& xc = context.get_continuous_state_vector();
    return xc.GetAtIndex(0);
  }

  void DoAddEvent(systems::CompositeEventCollection<T>* events) const override {
    event_->add_to_composite(events);
  }

  /// Unique pointer to the event.
  std::unique_ptr<systems::UnrestrictedUpdateEvent<T>> event_;
};

}  // namespace bouncing_ball
}  // namespace drake

