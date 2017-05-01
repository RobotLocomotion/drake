#pragma once

#include <iostream>
#include <memory>
#include <vector>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

/**
 * A generic class that captures the semantics of an individual event. This
 * class is agnostic to EventType. Any Trigger can be associated any EventType.
 * Trigger also owns an optional AbstractValue to facilitate passing additional
 * information from event triggering to event handlers. This is especially
 * useful for systems with external inputs (e.g. a LCM message subscriber).
 * The underlying data in AbstractValue needs to be copy-constructible and
 * assignable.
 */
class Trigger {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Trigger)

  /**
   * Predefined types of triggers. Used at run time to distinguish derived
   * Trigger type. See the corresponding class documentation for more details.
   * Any user introduced triggers need to have their unique type ids defined
   * here as well.
   */
  enum class TriggerType {
    kUnknown = 0,

    /**
     * This trigger means that any associated event is triggered forcefully in
     * contrast to the normal event trigger mechanisms in System such as
     * System::CalcNextUpdateTime() or System::GetPerStepEvents(). An example
     * usage is to directly invoke the event handlers. Calling
     * System::Publish(context) as opposed to System::Publish(context, events)
     * is a common and useful use case.
     */
    kForced = 1,

    /**
     * This trigger means that any associated event is triggered by a timer. The
     * most common use case is to represent periodic reoccurring events. However,
     * one-shot events triggered by a timer can also be represented with a 0 period.
     */
    kTimed = 2,
    kPeriodic = 3,

    /**
     * This trigger means that any associated event is triggered whenever the
     * `solver` takes a `step`. A `solver` can be any code that controls the time
     * and state evolution of a System. The Simulator is an instance of a `solver`,
     * and a `step` in the Simulator case corresponds to its underlying Integrator
     * taking a major time step. Per step events are most commonly triggered in
     * System::GetPerStepEvents(). A very common use of this is to update a
     * discrete or abstract state variable that changes whenever the trajectory
     * advances, such as the "min" or "max" of some quantity, recording of a signal
     * in a delay buffer, or publishing. Another example is to implement feedback
     * controllers interfaced with physical devices. The controller can be
     * implemented in the event handler, and the `step` corresponds to receiving
     * sensor data from the hardware.
     */
    kPerStep = 4,

    kWitness = 5,
  };

  explicit Trigger(TriggerType type) : type_(type) {}

  Trigger(TriggerType type, std::unique_ptr<AbstractValue> data)
      : type_(type), data_(std::move(data)) {}

  /**
   * Returns the trigger type.
   */
  TriggerType get_type() const { return type_; }

  /**
   * Returns a const pointer to the AbstractValue.
   */
  const AbstractValue* get_data() const { return data_.get(); }

  /**
   * Returns a const reference to the underlying data.
   *
   * @tparam DataType Needs to be copy-constructible and assignable.
   */
  template <typename DataType>
  const DataType& get_data() const {
    return data_->GetValue<DataType>();
  }

  /**
   * Sets and transfers the ownership of @p data.
   */
  void set_data(std::unique_ptr<AbstractValue> data) {
    data_ = std::move(data);
  }

  /**
   * Clones this instance.
   */
  std::unique_ptr<Trigger> Clone() const {
    auto clone = std::make_unique<Trigger>(type_);
    if (data_ != nullptr) {
      clone->set_data(data_->Clone());
    }
    return clone;
  }

 private:
  const TriggerType type_{TriggerType::kUnknown};

  // To Evan, Sherm and Eric: this is actually pretty useful for systems that
  // take external inputs, like a LcmSubscriberSystem. The current
  // implementation is not atomic in the sense that it first checks for message
  // arrival, if one arrives, it schedules an unrestricted update event. But,
  // by the time the handler is called, the message in the buffer could be
  // different from the one that triggers the event in the first place. With
  // this, I just can pass the lcm message content in the trigger.

  // Owned AbstractData.
  std::unique_ptr<AbstractValue> data_{nullptr};
};

std::ostream& operator<<(std::ostream& out, const Trigger::TriggerType& type);

}  // namespace systems
}  // namespace drake
