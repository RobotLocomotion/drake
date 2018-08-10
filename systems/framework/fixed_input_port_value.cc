#include "drake/systems/framework/fixed_input_port_value.h"

#include "drake/systems/framework/context_base.h"

namespace drake {
namespace systems {

AbstractValue* FixedInputPortValue::GetMutableData() {
  DRAKE_DEMAND(owning_subcontext_ != nullptr);
  ContextBase& context = *owning_subcontext_;
  const DependencyTracker& tracker = context.get_tracker(ticket_);
  const int64_t change_event = context.start_new_change_event();
  tracker.NoteValueChange(change_event);
  ++serial_number_;
  return value_.get_mutable();
}

}  // namespace systems
}  // namespace drake
