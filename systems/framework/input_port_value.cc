#include "drake/systems/framework/input_port_value.h"

#include "drake/systems/framework/context_base.h"

namespace drake {
namespace systems {

AbstractValue* FreestandingInputPortValue::GetMutableData() {
  ContextBase& context = get_mutable_owning_context();
  const DependencyTracker& tracker = context.get_tracker(ticket_);
  const int64_t change_event = context.start_new_change_event();
  tracker.NoteValueChange(change_event);
  ++serial_number_;
  return value_.get();
}

}  // namespace systems
}  // namespace drake
