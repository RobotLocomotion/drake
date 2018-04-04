#include "drake/systems/framework/input_port_value.h"

namespace drake {
namespace systems {

InputPortValue::~InputPortValue() {}

DependentInputPortValue::DependentInputPortValue(
    OutputPortValue* output_port_value)
    : output_port_value_(output_port_value) {
  DRAKE_DEMAND(output_port_value_ != nullptr);
}

FreestandingInputPortValue::FreestandingInputPortValue(
    std::unique_ptr<AbstractValue> data)
    : output_port_value_(std::move(data)) {}

}  // namespace systems
}  // namespace drake
