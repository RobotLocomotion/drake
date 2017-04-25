#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

class Trigger {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Trigger)

  enum TriggerType {
    kUnknownTrigger = 0,
    kForced = (1 << 0),
    kPeriodic = (1 << 1),
    kPerStep = (1 << 2),
    kAbstract = (1 << 4),
  };

  virtual ~Trigger() {}

  TriggerType get_type() const { return type_; }

  virtual std::unique_ptr<Trigger> Clone() const = 0;

 protected:
  explicit Trigger(TriggerType type) : type_(type) {}

 private:
  TriggerType type_;
};

class ForcedTrigger : public Trigger {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ForcedTrigger)

  ForcedTrigger() : Trigger(kForced) {}

  static const std::vector<const Trigger*>& OneForcedTrigger();

  std::unique_ptr<Trigger> Clone() const override {
    return std::unique_ptr<Trigger>(new ForcedTrigger());
  }
};

class PerStepTrigger : public Trigger {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PerStepTrigger)

  PerStepTrigger() : Trigger(kPerStep) {}

  std::unique_ptr<Trigger> Clone() const override {
    return std::unique_ptr<Trigger>(new PerStepTrigger());
  }
};

class PeriodicTrigger : public Trigger {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PeriodicTrigger)

  PeriodicTrigger() : Trigger(kPeriodic) {}

  std::unique_ptr<Trigger> Clone() const override {
    return std::unique_ptr<Trigger>(new PeriodicTrigger());
  }

 private:
  // should probably save offset and period.
};

class AbstractTrigger : public Trigger {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AbstractTrigger)

  AbstractTrigger(std::unique_ptr<AbstractValue> data)
      : Trigger(kAbstract), data_(std::move(data)) {}

  template <typename DataType>
  static std::unique_ptr<AbstractTrigger> Make(const DataType& data) {
    return std::make_unique<AbstractTrigger>(
        AbstractValue::Make<DataType>(data));
  }

  const AbstractValue* get_data() const {
    return data_.get();
  }

  template <typename DataType>
  const DataType& get_data() const {
    return data_->GetValue<DataType>();
  }

  std::unique_ptr<Trigger> Clone() const override;

 private:
  std::unique_ptr<AbstractValue> data_;
};

}  // namespace systems
}  // namespace drake
