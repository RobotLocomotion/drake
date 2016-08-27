#include "drake/systems/framework/system_output.h"

namespace drake {
namespace systems {

OutputPortListenerInterface::~OutputPortListenerInterface() {}

OutputPort::~OutputPort() {}


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
