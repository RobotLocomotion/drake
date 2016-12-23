#pragma once

#include <limits>

#include "drake/systems/plants/spring_mass_system/spring_mass_system.h"

namespace drake {
namespace systems {
namespace analysis_test {

template <class T>
class MySpringMassSystem : public SpringMassSystem<T> {
 public:
  // Pass through to SpringMassSystem, except add update rate
  MySpringMassSystem(double stiffness, double mass, double update_rate)
      : SpringMassSystem<T>(stiffness, mass, false /*no input force*/) {
    if (update_rate > 0.0) {
      this->DeclareUpdatePeriodSec(1.0 / update_rate);
    }
  }

  int get_publish_count() const { return publish_count_; }

  int get_update_count() const { return update_count_; }

  /** There are no discrete variables in this system, but it does use
   * a discrete update with zero variables. In other words, this function
   * is a kludge until discrete variables are automatically allocated in
   * a system.
   */
  std::unique_ptr<DiscreteState<T>> AllocateDiscreteVariables()
    const override {
    return std::make_unique<DiscreteState<T>>();
  }

 private:
  // Publish t q u to standard output.
  void DoPublish(const Context<T>& context) const override {
    ++publish_count_;
  }

  // The discrete equation update here is for the special case of zero
  // discrete variables- in other words, this is just a counter.
  void DoCalcDiscreteVariableUpdates(const Context<T>& context,
                                     DiscreteState<T>* discrete_state)
    const override {
    ++update_count_;
  }

  mutable int publish_count_{0};
  mutable int update_count_{0};
};  // MySpringMassSystem

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake

