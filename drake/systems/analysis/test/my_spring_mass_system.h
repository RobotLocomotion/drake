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
      : SpringMassSystem<T>(stiffness, mass, false /*no input force*/),
        update_rate_(update_rate) {}

  int get_publish_count() const { return publish_count_; }

  int get_update_count() const { return update_count_; }

  /** There are no difference variables in this system, but it does use
   * a difference update with zero variables. In other words, this function
   * is a kludge until difference variables are automatically allocated in
   * a system.
   */
  std::unique_ptr<DifferenceState<T>> AllocateDifferenceVariables()
    const override {
    return std::make_unique<DifferenceState<T>>();
  }

 private:
  // Publish t q u to standard output.
  void DoPublish(const Context<double> &context) const override {
    ++publish_count_;
  }

  // The difference equation update here is for the special case of zero
  // difference variables- in other words, this is just a counter.
  void DoEvalDifferenceUpdates(const Context<T>& context,
                               DifferenceState<T>* difference_state)
    const override {
    ++update_count_;
  }

  // Force a update at the next multiple of the sample rate. If current
  // time is exactly at a update time, we assume the update has been
  // done and return the following update time. That means we don't get a
  // sample at 0 but will get one at the end.
  void DoCalcNextUpdateTime(const Context<double> &context,
                            UpdateActions<T> *actions) const override {
    if (update_rate_ <= 0.) {
      actions->time = std::numeric_limits<double>::infinity();
      return;
    }

    // For reliable behavior, convert floating point times into integer
    // sample counts. We want the ceiling unless same as the floor.
    const int prev =
        static_cast<int>(std::floor(context.get_time() * update_rate_));
    const int next =
        static_cast<int>(std::ceil(context.get_time() * update_rate_));
    const int which = next == prev ? next + 1 : next;

    // Convert the next update count back to a time to return.
    const double next_update = which / update_rate_;
    actions->time = next_update;
    actions->events.clear();
    actions->events.push_back(DiscreteEvent<T>());
    actions->events.back().action = DiscreteEvent<T>::kUpdateAction;
  }

  double update_rate_{0.};  // Default is "don't update".

  mutable int publish_count_{0};
  mutable int update_count_{0};
};  // MySpringMassSystem

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake

