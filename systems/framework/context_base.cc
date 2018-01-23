#include "drake/systems/framework/context_base.h"

#include "drake/common/unused.h"
#include "drake/systems/framework/system_base.h"

namespace drake {
namespace systems {

std::string ContextBase::GetSystemPathname() const {
  std::vector<const ContextBase*> path_to_root{this};
  while (const ContextBase* parent = path_to_root.back()->get_parent_base())
    path_to_root.push_back(parent);
  std::string path;
  std::for_each(path_to_root.rbegin(), path_to_root.rend(),
                [&path](const ContextBase* node) {
                  path += "/" + node->GetSystemName();
                });
  return path;
}

void ContextBase::SetFixedInputPortValue(
    InputPortIndex index,
    std::unique_ptr<FreestandingInputPortValue> port_value) {
  DRAKE_DEMAND(0 <= index && index < get_num_input_ports());
  DRAKE_DEMAND(port_value != nullptr);

  DependencyTracker& port_tracker =
      get_mutable_tracker(input_port_tickets()[index]);
  FreestandingInputPortValue* old_value = input_port_values_[index].get();

  if (old_value != nullptr) {
    // All the dependency wiring is already in place.
    port_value->set_ticket(old_value->ticket());
  } else {
    // Create a new tracker and subscribe to it.
    DependencyTracker& value_tracker = graph_.CreateNewDependencyTracker(
        "Value for fixed input port " + std::to_string(index));
    port_value->set_ticket(value_tracker.ticket());
    port_tracker.SubscribeToPrerequisite(&value_tracker);
  }

  // Fill in the FreestandingInputPortValue object and install it.
  port_value->set_owning_context(this, index);
  input_port_values_[index] = std::move(port_value);

  // Invalidate anyone who cares about this input port.
  port_tracker.NoteValueChange(start_new_change_event());
}

// Set up trackers for independent sources: time, accuracy, state, parameters,
// and input ports.
void ContextBase::CreateWellKnownTrackers() {
  DependencyGraph& trackers = graph_;
  // This is the dummy "tracker" used for constants and anything else that has
  // no dependencies on any Context source.
  (void)trackers.CreateNewDependencyTracker(SystemBase::nothing_ticket(),
                                            "nothing");

  // Allocate trackers for time, accuracy, q, v, z.
  auto& time_tracker =
      trackers.CreateNewDependencyTracker(SystemBase::time_ticket(), "t");
  auto& accuracy_tracker = trackers.CreateNewDependencyTracker(
      SystemBase::accuracy_ticket(), "accuracy");
  auto& q_tracker =
      trackers.CreateNewDependencyTracker(SystemBase::q_ticket(), "q");
  auto& v_tracker =
      trackers.CreateNewDependencyTracker(SystemBase::v_ticket(), "v");
  auto& z_tracker =
      trackers.CreateNewDependencyTracker(SystemBase::z_ticket(), "z");

  // Continuous state xc depends on q, v, and z.
  auto& xc_tracker =
      trackers.CreateNewDependencyTracker(SystemBase::xc_ticket(), "xc");
  xc_tracker.SubscribeToPrerequisite(&q_tracker);
  xc_tracker.SubscribeToPrerequisite(&v_tracker);
  xc_tracker.SubscribeToPrerequisite(&z_tracker);

  // Allocate the "all discrete variables" xd tracker. The associated System is
  // responsible for allocating the individual discrete variable group xdᵢ
  // trackers and subscribing this one to each of those.
  auto& xd_tracker =
      trackers.CreateNewDependencyTracker(SystemBase::xd_ticket(), "xd");

  // Allocate the "all abstract variables" xa tracker. The associated System is
  // responsible for allocating the individual abstract variable xaᵢ
  // trackers and subscribing this one to each of those.
  auto& xa_tracker =
      trackers.CreateNewDependencyTracker(SystemBase::xa_ticket(), "xa");

  // The complete state x={xc,xd,xa}.
  auto& x_tracker =
      trackers.CreateNewDependencyTracker(SystemBase::all_state_ticket(), "x");
  x_tracker.SubscribeToPrerequisite(&xc_tracker);
  x_tracker.SubscribeToPrerequisite(&xd_tracker);
  x_tracker.SubscribeToPrerequisite(&xa_tracker);

  // Allocate the "all parameters" p tracker. The associated System is
  // responsible for allocating the individual numeric parameter pnᵢ and
  // abstract paraemter paᵢ trackers and subscribing this one to each of those.
  auto& p_tracker = trackers.CreateNewDependencyTracker(
      SystemBase::all_parameters_ticket(), "p");

  // Allocate the "all input ports" u tracker. The associated System is
  // responsible for allocating the individual input port uᵢ
  // trackers and subscribing this one to each of those.
  auto& u_tracker = trackers.CreateNewDependencyTracker(
      SystemBase::all_input_ports_ticket(), "u");

  // Allocate the "all sources" tracker. The complete list of known sources
  // is t,a,x,p, and u. Cache entries are not included separately because they
  // must ultimately depend on these same sources.
  auto& everything_tracker = trackers.CreateNewDependencyTracker(
      SystemBase::all_sources_ticket(), "all sources");
  everything_tracker.SubscribeToPrerequisite(&time_tracker);
  everything_tracker.SubscribeToPrerequisite(&accuracy_tracker);
  everything_tracker.SubscribeToPrerequisite(&x_tracker);
  everything_tracker.SubscribeToPrerequisite(&p_tracker);
  everything_tracker.SubscribeToPrerequisite(&u_tracker);

  auto& configuration_tracker = trackers.CreateNewDependencyTracker(
      SystemBase::configuration_ticket(), "configuration");
  // This default subscription must be changed if configuration is not
  // represented by q in this System.
  configuration_tracker.SubscribeToPrerequisite(&q_tracker);

  auto& velocity_tracker = trackers.CreateNewDependencyTracker(
      SystemBase::velocity_ticket(), "velocity");
  // This default subscription must be changed if velocity is not
  // represented by v in this System.
  velocity_tracker.SubscribeToPrerequisite(&v_tracker);

  // This tracks configuration & velocity regardless of their source.
  auto& kinematics_tracker = trackers.CreateNewDependencyTracker(
      SystemBase::kinematics_ticket(), "kinematics");
  kinematics_tracker.SubscribeToPrerequisite(&configuration_tracker);
  kinematics_tracker.SubscribeToPrerequisite(&velocity_tracker);

  auto& xcdot_tracker = trackers.CreateNewDependencyTracker(
      SystemBase::xcdot_ticket(), "xcdot");
  // TODO(sherm1) Connect to cache entry.
  unused(xcdot_tracker);

  auto& xdhat_tracker = trackers.CreateNewDependencyTracker(
      SystemBase::xdhat_ticket(), "xdhat");
  // TODO(sherm1) Connect to cache entry.
  unused(xdhat_tracker);
}

void ContextBase::NoteTimeChanged(int64_t change_event) {
  get_tracker(SystemBase::time_ticket()).NoteValueChange(change_event);
}

void ContextBase::NoteAccuracyChanged(int64_t change_event) {
  get_tracker(SystemBase::accuracy_ticket()).NoteValueChange(change_event);
}

void ContextBase::NoteAllQChanged(int64_t change_event) {
  get_tracker(SystemBase::q_ticket()).NoteValueChange(change_event);
}

void ContextBase::NoteAllVChanged(int64_t change_event) {
  get_tracker(SystemBase::v_ticket()).NoteValueChange(change_event);
}

void ContextBase::NoteAllZChanged(int64_t change_event) {
  get_tracker(SystemBase::z_ticket()).NoteValueChange(change_event);
}

void ContextBase::AddInputPort(InputPortIndex expected_index,
                               DependencyTicket ticket) {
  DRAKE_DEMAND(expected_index.is_valid() && ticket.is_valid());
  DRAKE_DEMAND(expected_index == get_num_input_ports());
  DRAKE_DEMAND(input_port_tickets_.size() == input_port_values_.size());
  auto& ui_tracker = graph_.CreateNewDependencyTracker(
      ticket, "u_" + std::to_string(expected_index));
  input_port_values_.emplace_back(nullptr);
  input_port_tickets_.emplace_back(ticket);
  auto& u_tracker =
      graph_.get_mutable_tracker(SystemBase::all_input_ports_ticket());
  u_tracker.SubscribeToPrerequisite(&ui_tracker);
}

}  // namespace systems
}  // namespace drake
