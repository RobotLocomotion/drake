#include "drake/systems/framework/system_input.h"

namespace drake {
namespace systems {

InputPort::~InputPort() {}

void InputPort::Invalidate() {
  if (invalidation_callback_ != nullptr) {
    invalidation_callback_();
  }
}

DependentInputPort::DependentInputPort(OutputPort* output_port)
    : output_port_(output_port) {
  DRAKE_DEMAND(output_port_ != nullptr);
  output_port_->add_dependent(this);
}

DependentInputPort::~DependentInputPort() {
  if (output_port_ != nullptr) {
    output_port_->remove_dependent(this);
  }
}

void DependentInputPort::Disconnect() {
  output_port_ = nullptr;
}
FreestandingInputPort::FreestandingInputPort(
    std::unique_ptr<AbstractValue> data)
    : output_port_(std::move(data)) {
  output_port_.add_dependent(this);
}

FreestandingInputPort::~FreestandingInputPort() {
  output_port_.remove_dependent(this);
}

}  // namespace systems
}  // namespace drake
