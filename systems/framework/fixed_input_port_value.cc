#include "drake/systems/framework/fixed_input_port_value.h"

namespace drake {
namespace systems {

AbstractValue* FixedInputPortValue::GetMutableData() {
  // TODO(sherm1) Change event tracking goes here.
  ++serial_number_;
  return value_.get_mutable();
}

}  // namespace systems
}  // namespace drake
