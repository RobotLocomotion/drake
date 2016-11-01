#include "drake/systems/framework/system_output.h"

namespace drake {
namespace systems {

OutputPort::~OutputPort() {
  // Notify any input ports that are still connected to this output port that
  // this output port no longer exists.
  for (detail::OutputPortListenerInterface* dependent : dependents_) {
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
  for (detail::OutputPortListenerInterface* dependent : dependents_) {
    dependent->Invalidate();
  }
}

}  // namespace systems
}  // namespace drake
