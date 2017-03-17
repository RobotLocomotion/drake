#include "drake/systems/framework/output_port_value.h"

namespace drake {
namespace systems {

OutputPortValue::~OutputPortValue() {
  // Notify any input ports that are still connected to this output port that
  // this output port no longer exists.
  for (detail::OutputPortListenerInterface* dependent : dependents_) {
    dependent->Disconnect();
  }
}


std::unique_ptr<OutputPortValue> OutputPortValue::Clone() const {
  if (data_ != nullptr) {
    return std::make_unique<OutputPortValue>(data_->Clone());
  }
  return nullptr;
}


void OutputPortValue::InvalidateAndIncrement() {
  ++version_;
  for (detail::OutputPortListenerInterface* dependent : dependents_) {
    dependent->Invalidate();
  }
}

template class SystemOutput<double>;

template class LeafSystemOutput<double>;

}  // namespace systems
}  // namespace drake
