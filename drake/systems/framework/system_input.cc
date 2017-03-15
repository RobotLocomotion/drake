#include "drake/systems/framework/system_input.h"

namespace drake {
namespace systems {

InputPort::~InputPort() {}

void InputPort::Invalidate() {
  if (invalidation_callback_ != nullptr) {
    invalidation_callback_();
  }
}

DependentInputPort::DependentInputPort(OutputPortValue* output_port_value)
    : output_port_value_(output_port_value) {
  DRAKE_DEMAND(output_port_value_ != nullptr);
  output_port_value_->add_dependent(this);
}

DependentInputPort::~DependentInputPort() {
  if (output_port_value_ != nullptr) {
    output_port_value_->remove_dependent(this);
  }
}

void DependentInputPort::Disconnect() {
  output_port_value_ = nullptr;
}
FreestandingInputPort::FreestandingInputPort(
    std::unique_ptr<AbstractValue> data)
    : output_port_value_(std::move(data)) {
  output_port_value_.add_dependent(this);
}

FreestandingInputPort::~FreestandingInputPort() {
  output_port_value_.remove_dependent(this);
}

}  // namespace systems
}  // namespace drake
