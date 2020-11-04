#pragma once

#include <limits>
#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/systems/analysis/test_utilities/spring_mass_system.h"

namespace drake {
namespace systems {
namespace analysis_test {

template <class T>
class MySpringMassSystem : public SpringMassSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MySpringMassSystem)

  // Pass through to SpringMassSystem, except add events and handlers.
  MySpringMassSystem(double stiffness, double mass, double update_rate)
      : SpringMassSystem<T>(stiffness, mass, false /*no input force*/) {
    // This forced-publish event is necessary for any simulator_test case that
    // needs to verify that the publish_every_time_step feature works.
    this->DeclareForcedPublishEvent(&MySpringMassSystem::CountPublishes);

    if (update_rate > 0.0) {
      this->DeclarePeriodicDiscreteUpdateEvent(1.0 / update_rate, 0.0,
          &MySpringMassSystem::CountDiscreteUpdates);
    }
  }

  int get_publish_count() const { return publish_count_; }

  int get_update_count() const { return update_count_; }

 private:
  EventStatus CountPublishes(const Context<T>&) const {
    ++publish_count_;
    return EventStatus::Succeeded();
  }

  // The discrete equation update here is for the special case of zero
  // discrete variables- in other words, this is just a counter.
  EventStatus CountDiscreteUpdates(const Context<T>&,
                                   DiscreteValues<T>*) const {
    ++update_count_;
    return EventStatus::Succeeded();
  }

  mutable int publish_count_{0};
  mutable int update_count_{0};
};  // MySpringMassSystem

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake
