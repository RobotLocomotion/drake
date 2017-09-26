#include "drake/systems/framework/output_port_value.h"

#include "drake/common/default_scalars.h"

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

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::SystemOutput)

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::LeafSystemOutput)
