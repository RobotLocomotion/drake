#include "drake/systems/framework/system_input.h"

namespace drake {
namespace systems {

InputPort::~InputPort() {}

void InputPort::Invalidate() {
  if (invalidation_callback_ != nullptr) {
    invalidation_callback_();
  }
}

DependentInputPort::~DependentInputPort() {
  output_port_->remove_dependent(this);
}

FreestandingInputPort::~FreestandingInputPort() {
  output_port_.remove_dependent(this);
}

}  // systems
}  // drake
