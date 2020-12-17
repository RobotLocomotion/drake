#include "drake/systems/framework/context_base.h"

#include <string>
#include <typeinfo>

#include "drake/common/unused.h"

namespace drake {
namespace systems {

std::unique_ptr<ContextBase> ContextBase::Clone() const {
  if (!is_root_context()) {
      throw std::logic_error(fmt::format(
          "Context::Clone(): Cannot clone a non-root Context; "
          "this Context was created by '{}'.", system_name_));
  }

  std::unique_ptr<ContextBase> clone_ptr(CloneWithoutPointers(*this));
  ContextBase& clone = *clone_ptr;

  // Create a complete mapping of tracker pointers.
  DependencyTracker::PointerMap tracker_map;
  BuildTrackerPointerMap(*this, clone, &tracker_map);

  // Then do a pointer fixup pass.
  FixContextPointers(*this, tracker_map, &clone);
  return clone_ptr;
}

ContextBase::~ContextBase() {}

std::string ContextBase::GetSystemPathname() const {
  const std::string parent_path = get_parent_base()
                                      ? get_parent_base()->GetSystemPathname()
                                      : std::string();
  return parent_path + internal::SystemMessageInterface::path_separator() +
         GetSystemName();
}

FixedInputPortValue& ContextBase::FixInputPort(
    int index, const AbstractValue& value) {
  std::unique_ptr<FixedInputPortValue> fixed =
      internal::ContextBaseFixedInputAttorney::CreateFixedInputPortValue(
          value.Clone());
  FixedInputPortValue& fixed_ref = *fixed;
  SetFixedInputPortValue(InputPortIndex(index), std::move(fixed));
  return fixed_ref;
}

void ContextBase::AddInputPort(
    InputPortIndex expected_index, DependencyTicket ticket,
    std::function<void(const AbstractValue&)> fixed_input_type_checker) {
  DRAKE_DEMAND(expected_index.is_valid() && ticket.is_valid());
  DRAKE_DEMAND(expected_index == num_input_ports());
  DRAKE_DEMAND(input_port_tickets_.size() == input_port_values_.size());
  DRAKE_DEMAND(input_port_tickets_.size() == input_port_type_checkers_.size());
  if (!fixed_input_type_checker) {
    fixed_input_type_checker = [](const AbstractValue&) {};
  }
  auto& ui_tracker = graph_.CreateNewDependencyTracker(
      ticket, "u_" + std::to_string(expected_index));
  input_port_values_.emplace_back(nullptr);
  input_port_tickets_.emplace_back(ticket);
  input_port_type_checkers_.emplace_back(std::move(fixed_input_type_checker));
  auto& u_tracker = graph_.get_mutable_tracker(
      DependencyTicket(internal::kAllInputPortsTicket));
  u_tracker.SubscribeToPrerequisite(&ui_tracker);
}

void ContextBase::AddOutputPort(
    OutputPortIndex expected_index, DependencyTicket ticket,
    const internal::OutputPortPrerequisite& prerequisite) {
  DRAKE_DEMAND(expected_index.is_valid() && ticket.is_valid());
  DRAKE_DEMAND(expected_index == num_output_ports());
  auto& yi_tracker = graph_.CreateNewDependencyTracker(
      ticket, "y_" + std::to_string(expected_index));
  output_port_tickets_.push_back(ticket);
  // If no child subsystem was specified then this output port's dependency is
  // resolvable within this subcontext so we can subscribe now. Inter-subcontext
  // dependencies are set up by Diagram after all child intra-subcontext
  // dependency trackers have been allocated.
  if (!prerequisite.child_subsystem) {
    yi_tracker.SubscribeToPrerequisite(
        &get_mutable_tracker(prerequisite.dependency));
  }
}

void ContextBase::SetFixedInputPortValue(
    InputPortIndex index,
    std::unique_ptr<FixedInputPortValue> port_value) {
  DRAKE_DEMAND(0 <= index && index < num_input_ports());
  DRAKE_DEMAND(port_value != nullptr);

  // Fail-fast if the user supplied the wrong type or size.
  input_port_type_checkers_[index](port_value->get_value());

  DependencyTracker& port_tracker =
      get_mutable_tracker(input_port_tickets_[index]);
  FixedInputPortValue* old_value =
      input_port_values_[index].get_mutable();

  DependencyTicket ticket_to_use;
  if (old_value != nullptr) {
    // All the dependency wiring should be in place already.
    ticket_to_use = old_value->ticket();
    DRAKE_DEMAND(graph_.has_tracker(ticket_to_use));
    DRAKE_ASSERT(graph_.get_tracker(ticket_to_use).HasSubscriber(port_tracker));
    DRAKE_ASSERT(
        port_tracker.HasPrerequisite(graph_.get_tracker(ticket_to_use)));
  } else {
    // Create a new tracker and subscribe to it.
    DependencyTracker& value_tracker = graph_.CreateNewDependencyTracker(
        "Value for fixed input port " + std::to_string(index));
    ticket_to_use = value_tracker.ticket();
    port_tracker.SubscribeToPrerequisite(&value_tracker);
  }

  // Fill in the FixedInputPortValue object and install it.
  internal::ContextBaseFixedInputAttorney::set_ticket(port_value.get(),
                                                      ticket_to_use);
  internal::ContextBaseFixedInputAttorney::set_owning_subcontext(
      port_value.get(), this);
  input_port_values_[index] = std::move(port_value);

  // Invalidate anyone who cares about this input port.
  graph_.get_tracker(ticket_to_use).NoteValueChange(start_new_change_event());
}

// Set up trackers for independent sources: time, accuracy, state, parameters,
// and input ports, and the predefined computations for derivatives, energy,
// and power. This code should set up everything listed in the
// internal::BuiltInTicketNumbers enum, and do so in the same order. The code
// for individual trackers below must be kept up to date with the API contracts
// for the corresponding tickets in SystemBase.
void ContextBase::CreateBuiltInTrackers() {
  DependencyGraph& graph = graph_;
  // This is the dummy "tracker" used for constants and anything else that has
  // no dependencies on any Context source. Ignoring return value.
  graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kNothingTicket), "nothing");

  // Allocate trackers for time and accuracy.
  auto& time_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kTimeTicket), "t");
  auto& accuracy_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kAccuracyTicket), "accuracy");

  // Allocate trackers for continuous state variables. These are independent
  // in leaf systems but diagrams must add dependencies on their children's
  // q, v, and z trackers respectively.
  auto& q_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kQTicket), "q");
  auto& v_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kVTicket), "v");
  auto& z_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kZTicket), "z");

  // Continuous state xc depends on q, v, and z.
  auto& xc_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kXcTicket), "xc");
  xc_tracker.SubscribeToPrerequisite(&q_tracker);
  xc_tracker.SubscribeToPrerequisite(&v_tracker);
  xc_tracker.SubscribeToPrerequisite(&z_tracker);

  // Allocate the "all discrete variables" xd tracker. The associated System is
  // responsible for allocating the individual discrete variable group xdᵢ
  // trackers and subscribing this one to each of those. Diagrams must add
  // dependencies on each child subcontext's xd tracker.
  auto& xd_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kXdTicket), "xd");

  // Allocate the "all abstract variables" xa tracker. The associated System is
  // responsible for allocating the individual abstract variable xaᵢ
  // trackers and subscribing this one to each of those. Diagrams must add
  // dependencies on each child subcontext's xa tracker.
  auto& xa_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kXaTicket), "xa");

  // The complete state x={xc,xd,xa}.
  auto& x_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kXTicket), "x");
  x_tracker.SubscribeToPrerequisite(&xc_tracker);
  x_tracker.SubscribeToPrerequisite(&xd_tracker);
  x_tracker.SubscribeToPrerequisite(&xa_tracker);

  // Allocate a tracker representing all the numeric parameters. The associated
  // System is responsible for allocating the individual numeric parameters pnᵢ
  // and subscribing the pn tracker to each one. Diagrams must add dependencies
  // on each child subcontext's pn tracker.
  auto& pn_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kPnTicket), "pn");

  // Allocate a tracker representing all the abstract parameters. The associated
  // System is responsible for allocating the individual numeric parameters paᵢ
  // and subscribing the pa tracker to each one. Diagrams must add dependencies
  // on each child subcontext's pa tracker.
  auto& pa_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kPaTicket), "pa");

  // Allocate a tracker representing all the parameters, p={pn,pa}.
  auto& p_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kAllParametersTicket), "p");
  p_tracker.SubscribeToPrerequisite(&pn_tracker);
  p_tracker.SubscribeToPrerequisite(&pa_tracker);

  // Allocate the "all input ports" u tracker. The associated System is
  // responsible for allocating the individual input port uᵢ
  // trackers and subscribing this one to each of those.
  auto& u_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kAllInputPortsTicket), "u");

  // Allocate the "all sources except input ports" tracker. The complete list of
  // known sources is t,a,x,p. Note that cache entries are not included.
  // Usually that won't matter since cache entries typically depend on at least
  // one of t,a,x, or p so will be invalided for the same reason the current
  // computation is. However, any dependency on a cache entry that depends only
  // on input ports would have to be declared explicitly. (In practice specific
  // input port trackers should be active with this one and logically that
  // should take care of cache entries also -- otherwise there is a contributing
  // input port that wasn't listed.)
  auto& all_sources_except_input_ports_tracker =
      graph.CreateNewDependencyTracker(
          DependencyTicket(internal::kAllSourcesExceptInputPortsTicket),
          "all sources except input ports");
  all_sources_except_input_ports_tracker.SubscribeToPrerequisite(&time_tracker);
  all_sources_except_input_ports_tracker.SubscribeToPrerequisite(
      &accuracy_tracker);
  all_sources_except_input_ports_tracker.SubscribeToPrerequisite(&x_tracker);
  all_sources_except_input_ports_tracker.SubscribeToPrerequisite(&p_tracker);

  // Allocate the "all sources" tracker. The complete list of known sources
  // is t,a,x,p,u. Note that cache entries are not included. Under normal
  // operation that doesn't matter because cache entries are invalidated only
  // when one of these source values changes. Any computation that has
  // declared "all sources" dependence will also have been invalidated for the
  // same reason so doesn't need to explicitly list cache entries.
  auto& all_sources_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kAllSourcesTicket), "all sources");
  all_sources_tracker.SubscribeToPrerequisite(
      &all_sources_except_input_ports_tracker);
  all_sources_tracker.SubscribeToPrerequisite(&u_tracker);

  // Allocate kinematics trackers to provide a level of abstraction from the
  // specific state variables that are used to represent configuration and
  // rate of change of configuration. For example, a kinematics cache entry
  // should depend on configuration regardless of whether we use continuous or
  // discrete variables. And it should be possible to switch between continuous
  // and discrete representations without having to change the specified
  // dependency, which remains "configuration" either way.
  //
  // See SystemBase::configuration_ticket() and kinematics_ticket() for the API
  // contract that must be implemented here and make sure this code is kept
  // up to date with that contract.

  // TODO(sherm1) Should track changes to configuration and velocity regardless
  // of how represented. See issue #9171. Until that is resolved, we must
  // assume that "configuration" results (like end effector location and PE)
  // can be affected by anything *except* time, v, and u; and "kinematics"
  // results (like end effector velocity and KE) can be affected by anything
  // except time and u.
  auto& configuration_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kConfigurationTicket), "configuration");
  // Compare with "all sources" above.
  configuration_tracker.SubscribeToPrerequisite(&accuracy_tracker);
  configuration_tracker.SubscribeToPrerequisite(&q_tracker);  // Not v.
  configuration_tracker.SubscribeToPrerequisite(&z_tracker);
  configuration_tracker.SubscribeToPrerequisite(&xd_tracker);
  configuration_tracker.SubscribeToPrerequisite(&xa_tracker);
  configuration_tracker.SubscribeToPrerequisite(&p_tracker);

  // This tracks configuration & velocity regardless of how represented.
  auto& kinematics_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kKinematicsTicket), "kinematics");
  kinematics_tracker.SubscribeToPrerequisite(&configuration_tracker);
  kinematics_tracker.SubscribeToPrerequisite(&v_tracker);

  // The following trackers are for well-known cache entries which don't
  // exist yet at this point in Context creation. When the corresponding cache
  // entry values are created (by SystemBase), these trackers will be subscribed
  // to the Calc() method's prerequisites, and set to invalidate the cache entry
  // value when the prerequisites change. Diagrams must add dependencies on
  // their constituent subcontexts' corresponding trackers.

  auto& xcdot_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kXcdotTicket), "xcdot");
  unused(xcdot_tracker);

  auto& pe_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kPeTicket), "PE");
  unused(pe_tracker);

  auto& ke_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kKeTicket), "KE");
  unused(ke_tracker);

  auto& pc_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kPcTicket), "pc");
  unused(pc_tracker);

  auto& pnc_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kPncTicket), "pnc");
  unused(pnc_tracker);
}

void ContextBase::BuildTrackerPointerMap(
    const ContextBase& source, const ContextBase& clone,
    DependencyTracker::PointerMap* tracker_map) {
  // First map the pointers local to this context.
  source.graph_.AppendToTrackerPointerMap(clone.get_dependency_graph(),
                                          &*tracker_map);

  // Then recursively ask our descendants to add their information to the map.
  source.DoPropagateBuildTrackerPointerMap(clone, &*tracker_map);
}

void ContextBase::FixContextPointers(
    const ContextBase& source, const DependencyTracker::PointerMap& tracker_map,
    ContextBase* clone) {
  // First repair pointers local to this context.
  clone->graph_.RepairTrackerPointers(source.get_dependency_graph(),
                                      tracker_map, clone, &clone->cache_);
  // Cache and FixedInputs only need their back pointers set to `this`.
  clone->cache_.RepairCachePointers(clone);
  for (auto& fixed_input : clone->input_port_values_) {
    if (fixed_input != nullptr) {
      internal::ContextBaseFixedInputAttorney::set_owning_subcontext(
          fixed_input.get_mutable(), clone);
    }
  }

  // Then recursively ask our descendants to repair their pointers.
  clone->DoPropagateFixContextPointers(source, tracker_map);
}

}  // namespace systems
}  // namespace drake
