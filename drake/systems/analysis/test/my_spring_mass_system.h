#pragma once

#include <limits>
#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/systems/plants/spring_mass_system/spring_mass_system.h"

namespace drake {
namespace systems {
namespace analysis_test {

template <class T>
class MySpringMassSystem : public SpringMassSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MySpringMassSystem)

  // Pass through to SpringMassSystem, except add update rate
  MySpringMassSystem(double stiffness, double mass, double update_rate)
      : SpringMassSystem<T>(stiffness, mass, false /*no input force*/) {
    if (update_rate > 0.0) {
      this->DeclarePeriodicDiscreteUpdate(1.0 / update_rate);
    }
  }

  int get_publish_count() const { return publish_count_; }

  int get_update_count() const { return update_count_; }

 private:
  // Publish t q u to standard output.
  void DoPublish(const Context<T>&,
                 const std::vector<const systems::PublishEvent<T>*>&)
      const override {
    ++publish_count_;
  }

  // The discrete equation update here is for the special case of zero
  // discrete variables- in other words, this is just a counter.
  void DoCalcDiscreteVariableUpdates(
      const Context<T>&,
      const std::vector<const systems::DiscreteUpdateEvent<T>*>&,
      DiscreteValues<T>*) const override {
    ++update_count_;
  }

  mutable int publish_count_{0};
  mutable int update_count_{0};
};  // MySpringMassSystem

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake
