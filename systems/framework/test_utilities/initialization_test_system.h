#pragma once

#include <vector>

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/** A LeafSystem which declares all possible initialization events, and records
 information (in mutable variables and in the context) about whether the events
 have been processed. This can be used to confirm that initialization events
 are properly handled. */
class InitializationTestSystem : public LeafSystem<double> {
 public:
  InitializationTestSystem() {
    // N.B. In the code below, we call the DeclareInitializationEvent() internal
    // API to declare events, so that we can carefully inspect the contents
    // during unit testing. Users should NOT use this event API, but instead the
    // DeclareInitializationPublishEvent(), etc.

    // Publish event.
    PublishEvent<double> pub_event(
        TriggerType::kInitialization,
        [this](const System<double>&, const Context<double>&,
               const PublishEvent<double>& event) {
          EXPECT_EQ(event.get_trigger_type(), TriggerType::kInitialization);
          this->InitPublish();
          return EventStatus::Succeeded();
        });
    DeclareInitializationEvent(pub_event);

    // Discrete state and update event.
    DeclareDiscreteState(1);
    DiscreteUpdateEvent<double> discrete_update_event(
        TriggerType::kInitialization,
        [this](const System<double>&, const Context<double>&,
               const DiscreteUpdateEvent<double>& event,
               DiscreteValues<double>* discrete_state) {
          EXPECT_EQ(event.get_trigger_type(), TriggerType::kInitialization);
          this->InitDiscreteUpdate(discrete_state);
          return EventStatus::Succeeded();
        });
    DeclareInitializationEvent(discrete_update_event);

    // Abstract state and update event.
    DeclareAbstractState(Value<bool>(false));
    UnrestrictedUpdateEvent<double> unrestricted_update_event(
        TriggerType::kInitialization,
        [this](const System<double>&, const Context<double>&,
               const UnrestrictedUpdateEvent<double>& event,
               State<double>* state) {
          EXPECT_EQ(event.get_trigger_type(), TriggerType::kInitialization);
          this->InitUnrestrictedUpdate(state);
          return EventStatus::Succeeded();
        });
    DeclareInitializationEvent(unrestricted_update_event);
  }

  bool get_pub_init() const { return pub_init_; }
  bool get_dis_update_init() const { return dis_update_init_; }
  bool get_unres_update_init() const { return unres_update_init_; }

 private:
  void InitPublish() const { pub_init_ = true; }

  void InitDiscreteUpdate(DiscreteValues<double>* discrete_state) const {
    dis_update_init_ = true;
    discrete_state->set_value(Vector1d(1.23));
  }

  void InitUnrestrictedUpdate(State<double>* state) const {
    unres_update_init_ = true;
    state->get_mutable_abstract_state<bool>(0) = true;
  }

  mutable bool pub_init_{false};
  mutable bool dis_update_init_{false};
  mutable bool unres_update_init_{false};
};

}  // namespace systems
}  // namespace drake
