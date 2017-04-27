#pragma once

#include <iostream>
#include <memory>
#include <vector>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

/**
 * A generic class that captures the semantics of an individual event. This
 * class is agnostic to EventType. Any Trigger can be associated any EventType.
 * Trigger also owns an optional AbstractValue to facilitate passing additional
 * information from event triggering to event handlers. This is especially
 * useful for systems with external inputs (e.g. a LCM message subscriber).
 */
class Trigger {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Trigger)

  /**
   * Predefined types of triggers. Used at run time to distinguish derived
   * Trigger type. See the corresponding class documentation for more details.
   */
  enum class TriggerType {
    kUnknown = 0,
    kForced = 1,
    kPeriodic = 2,
    kPerStep = 3,
  };

  virtual ~Trigger() {}

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
    std::unique_ptr<Trigger> clone = DoClone();
    if (data_ != nullptr) {
      clone->set_data(data_->Clone());
    }
    return clone;
  }

 protected:
  explicit Trigger(TriggerType type) : type_(type) {}

  virtual std::unique_ptr<Trigger> DoClone() const = 0;

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

/**
 * This trigger means that any associated event is triggered forcefully in
 * contrast to the normal event trigger mechanisms in System such as
 * System::CalcNextUpdateTime() or System::GetPerStepEvents(). An example
 * usage is to directly invoke the event handlers. Calling
 * System::Publish(context) as opposed to System::Publish(context, triggers)
 * is a very common and useful typical use case.
 */
class ForcedTrigger final : public Trigger {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ForcedTrigger)

  ForcedTrigger() : Trigger(TriggerType::kForced) {}

  static const std::vector<const Trigger*>& OneForcedTrigger();

 private:
  std::unique_ptr<Trigger> DoClone() const override {
    return std::unique_ptr<Trigger>(new ForcedTrigger());
  }
};

/**
 * This trigger means that any associated event is triggered whenever the
 * `evaluator` takes a `step`. An `evaluator` can be any code that controls
 * the time and state evolution of a System. The Simulator is an instance of
 * an `evaluator`, and a `step` in the Simulator case corresponds to its
 * underlying Integrator taking a major time step. Per step events are most
 * commonly triggered in System::GetPerStepEvents(). A common use case is to
 * trigger a publish event whenever the Simulator takes a major time step.
 * Another example is to implement feedback controllers interfaced with real
 * hardware. The controller can be implemented in the event handler, and
 * the `step` corresponds to receiving sensor data from the hardware.
 */
class PerStepTrigger final : public Trigger {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PerStepTrigger)

  PerStepTrigger() : Trigger(TriggerType::kPerStep) {}

 private:
  std::unique_ptr<Trigger> DoClone() const override {
    return std::unique_ptr<Trigger>(new PerStepTrigger());
  }
};

/**
 * This trigger means that any associated event is triggered by a timer. The
 * most common use case is to represent periodic reoccurring events. However,
 * one-shot events triggered by a timer can also be represented with a 0 period.
 */
class PeriodicTrigger final : public Trigger {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PeriodicTrigger)

  PeriodicTrigger(double period_sec, double offset_sec)
      : Trigger(TriggerType::kPeriodic),
        period_sec_(period_sec), offset_sec_(offset_sec) {}

  double get_period_sec() const { return period_sec_; }
  double get_offset_sec() const { return offset_sec_; }

 private:
  std::unique_ptr<Trigger> DoClone() const override {
    return std::unique_ptr<Trigger>(
        new PeriodicTrigger(period_sec_, offset_sec_));
  }

  double period_sec_{0};
  double offset_sec_{0};
};

}  // namespace systems
}  // namespace drake
