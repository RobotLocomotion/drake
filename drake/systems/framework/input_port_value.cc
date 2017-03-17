#include "drake/systems/framework/input_port_value.h"

namespace drake {
namespace systems {

InputPortValue::~InputPortValue() {}

void InputPortValue::Invalidate() {
  if (invalidation_callback_ != nullptr) {
    invalidation_callback_();
  }
}

DependentInputPortValue::DependentInputPortValue(
    OutputPortValue* output_port_value)
    : output_port_value_(output_port_value) {
  DRAKE_DEMAND(output_port_value_ != nullptr);
  output_port_value_->add_dependent(this);
}

DependentInputPortValue::~DependentInputPortValue() {
  if (output_port_value_ != nullptr) {
    output_port_value_->remove_dependent(this);
  }
}

void DependentInputPortValue::Disconnect() {
  output_port_value_ = nullptr;
}
FreestandingInputPortValue::FreestandingInputPortValue(
    std::unique_ptr<AbstractValue> data)
    : output_port_value_(std::move(data)) {
  output_port_value_.add_dependent(this);
}

FreestandingInputPortValue::~FreestandingInputPortValue() {
  output_port_value_.remove_dependent(this);
}

}  // namespace systems
}  // namespace drake
