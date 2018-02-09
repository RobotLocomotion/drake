#include "drake/systems/framework/output_port_value.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace systems {

OutputPortValue::~OutputPortValue() {}

std::unique_ptr<OutputPortValue> OutputPortValue::Clone() const {
  if (data_ != nullptr) {
    return std::make_unique<OutputPortValue>(data_->Clone());
  }
  return nullptr;
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::SystemOutput)

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::LeafSystemOutput)
