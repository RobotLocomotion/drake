#pragma once

#include <memory>
#include <vector>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

/**
 * A generic class that captures the semantics of an individual event.
 */
class Trigger {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Trigger)

  /**
   * Predefined types of triggers. Used at run time to distinguish derived
   * Trigger type.
   */
  enum TriggerType {
    kUnknownTrigger = 0,
    /// Corresponds to ForcedTrigger.
    kForced = 1,
    /// Corresponds to PeriodicTrigger.
    kPeriodic = 2,
    /// Corresponds to PerStepTrigger.
    kPerStep = 3,
    /// Corresponds to AbstractTrigger.
    kAbstract = 4,
  };

  virtual ~Trigger() {}

  /**
   * Returns the trigger type.
   */
  TriggerType get_type() const { return type_; }

  /**
   * Clones this instance.
   */
  std::unique_ptr<Trigger> Clone() const { return DoClone(); }

 protected:
  explicit Trigger(TriggerType type) : type_(type) {}

  virtual std::unique_ptr<Trigger> DoClone() const = 0;

 private:
  TriggerType type_{TriggerType::kUnknownTrigger};
};

/**
 * This trigger means that any associated event is triggered forcefully.
 * One use case is for the user to directly invoke the event handlers.
 */
class ForcedTrigger final : public Trigger {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ForcedTrigger)

  ForcedTrigger() : Trigger(kForced) {}

  static const std::vector<const Trigger*>& OneForcedTrigger();

 private:
  std::unique_ptr<Trigger> DoClone() const override {
    return std::unique_ptr<Trigger>(new ForcedTrigger());
  }
};

/**
 * This trigger means that any associated event is triggered by any per
 * evaluation step (i.e. a major time step in Simulator)
 */
class PerStepTrigger final : public Trigger {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PerStepTrigger)

  PerStepTrigger() : Trigger(kPerStep) {}

 private:
  std::unique_ptr<Trigger> DoClone() const override {
    return std::unique_ptr<Trigger>(new PerStepTrigger());
  }
};

/**
 * This trigger means that any associated event is triggered by a periodic
 * timer.
 */
class PeriodicTrigger final : public Trigger {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PeriodicTrigger)

  PeriodicTrigger() : Trigger(kPeriodic) {}

 private:
  std::unique_ptr<Trigger> DoClone() const override {
    return std::unique_ptr<Trigger>(new PeriodicTrigger());
  }

 private:
  // should probably save offset and period.
};


// To Evan, Sherm and Eric: this is actually pretty useful for systems that
// take external inputs, like a LcmSubscriberSystem. The current implementation
// is not atomic in the sense that it first checks for message arrival, if one
// arrives, it schedules an unrestricted update event. But, by the time the
// handler is called, the message in the buffer could be different from the one
// that triggers the event in the first place. With this, I just can pass the
// lcm message content in the trigger.
/**
 * Base class for abstract triggers. This class owns an AbstractValue, and can
 * be used to pass additional information from event triggering mechanisms to
 * event handlers.
 */
class AbstractTrigger : public Trigger {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AbstractTrigger)

  /**
   * Constructor.
   * @param data Data to be owned.
   */
  explicit AbstractTrigger(std::unique_ptr<AbstractValue> data)
      : Trigger(kAbstract), data_(std::move(data)) {}

  /**
   * Factory function that makes a AbstractTrigger given @p data.
   *
   * @tparam DataType Needs to be copy-constructible and assignable.
   */
  template <typename DataType>
  static std::unique_ptr<AbstractTrigger> Make(const DataType& data) {
    return std::make_unique<AbstractTrigger>(
        AbstractValue::Make<DataType>(data));
  }

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

 private:
  std::unique_ptr<Trigger> DoClone() const override;

  // Owned AbstractData.
  std::unique_ptr<AbstractValue> data_;
};

}  // namespace systems
}  // namespace drake
