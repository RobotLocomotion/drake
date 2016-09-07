#include "drake/systems/framework/system_output.h"

namespace drake {
namespace systems {

OutputPortListenerInterface::~OutputPortListenerInterface() {}

OutputPort::~OutputPort() {
  // Notify any input ports that are still connected to this output port that
  // this output port no longer exists.
  for (OutputPortListenerInterface* dependent : dependents_) {
    dependent->Disconnect();
  }
}


std::unique_ptr<OutputPort> OutputPort::Clone() const {
  if (data_ != nullptr) {
    return std::make_unique<OutputPort>(data_->Clone());
  }
  return nullptr;
}


void OutputPort::InvalidateAndIncrement() {
  ++version_;
  for (OutputPortListenerInterface* dependent : dependents_) {
    dependent->Invalidate();
  }
}

}  // systems
}  // drake
