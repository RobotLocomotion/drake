#include "drake/systems/framework/context_base.h"

#include <string>
#include <typeinfo>

#include "drake/common/unused.h"

namespace drake {
namespace systems {

std::unique_ptr<ContextBase> ContextBase::Clone() const {
  std::unique_ptr<ContextBase> clone_ptr(CloneWithoutPointers(*this));

  // Verify that the most-derived Context didn't forget to override
  // DoCloneWithoutPointers().
  const ContextBase& source = *this;  // Deref here to avoid typeid warning.
  ContextBase& clone = *clone_ptr;
  DRAKE_ASSERT(typeid(source) == typeid(clone));

  // Create a complete mapping of tracker pointers.
  DependencyTracker::PointerMap tracker_map;
  BuildTrackerPointerMap(clone, &tracker_map);

  // Then do a pointer fixup pass.
  clone.FixContextPointers(source, tracker_map);
  return clone_ptr;
}

ContextBase::~ContextBase() {}

void ContextBase::DisableCaching() const {
  cache_.DisableCaching();
  // TODO(sherm1) Recursive disabling of descendents goes here.
}

void ContextBase::EnableCaching() const {
  cache_.EnableCaching();
  // TODO(sherm1) Recursive enabling of descendents goes here.
}

void ContextBase::SetAllCacheEntriesOutOfDate() const {
  cache_.SetAllEntriesOutOfDate();
  // TODO(sherm1) Recursive update of descendents goes here.
}

std::string ContextBase::GetSystemPathname() const {
  const std::string parent_path = get_parent_base()
                                      ? get_parent_base()->GetSystemPathname()
                                      : std::string();
  return parent_path + internal::SystemMessageInterface::path_separator() +
         GetSystemName();
}

FixedInputPortValue& ContextBase::FixInputPort(
    int index, std::unique_ptr<AbstractValue> value) {
  auto fixed = std::make_unique<FixedInputPortValue>(std::move(value));
  FixedInputPortValue& fixed_ref = *fixed;
  SetFixedInputPortValue(InputPortIndex(index), std::move(fixed));
  return fixed_ref;
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
  auto& u_tracker = graph_.get_mutable_tracker(
      DependencyTicket(internal::kAllInputPortsTicket));
  u_tracker.SubscribeToPrerequisite(&ui_tracker);
}

void ContextBase::SetFixedInputPortValue(
    InputPortIndex index,
    std::unique_ptr<FixedInputPortValue> port_value) {
  DRAKE_DEMAND(0 <= index && index < get_num_input_ports());
  DRAKE_DEMAND(port_value != nullptr);

  // Fill in the FixedInputPortValue object and install it.
  detail::ContextBaseFixedInputAttorney::set_owning_subcontext(
      port_value.get(), this);
  input_port_values_[index] = std::move(port_value);
}

// Set up trackers for independent sources: time, accuracy, state, parameters,
// and input ports.
void ContextBase::CreateBuiltInTrackers() {
  DependencyGraph& graph = graph_;
  // This is the dummy "tracker" used for constants and anything else that has
  // no dependencies on any Context source. Ignoring return value.
  graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kNothingTicket), "nothing");

  // Allocate trackers for time, accuracy, q, v, z.
  auto& time_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kTimeTicket), "t");
  auto& accuracy_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kAccuracyTicket), "accuracy");
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
  // trackers and subscribing this one to each of those.
  auto& xd_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kXdTicket), "xd");

  // Allocate the "all abstract variables" xa tracker. The associated System is
  // responsible for allocating the individual abstract variable xaᵢ
  // trackers and subscribing this one to each of those.
  auto& xa_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kXaTicket), "xa");

  // The complete state x={xc,xd,xa}.
  auto& x_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kXTicket), "x");
  x_tracker.SubscribeToPrerequisite(&xc_tracker);
  x_tracker.SubscribeToPrerequisite(&xd_tracker);
  x_tracker.SubscribeToPrerequisite(&xa_tracker);

  // Allocate the "all parameters" p tracker. The associated System is
  // responsible for allocating the individual numeric parameter pnᵢ and
  // abstract paraemter paᵢ trackers and subscribing this one to each of those.
  auto& p_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kAllParametersTicket), "p");

  // Allocate the "all input ports" u tracker. The associated System is
  // responsible for allocating the individual input port uᵢ
  // trackers and subscribing this one to each of those.
  auto& u_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kAllInputPortsTicket), "u");

  // Allocate the "all sources" tracker. The complete list of known sources
  // is t,a,x,p,u. Note that cache entries are not included. Under normal
  // operation that doesn't matter because cache entries are invalidated only
  // when one of these source values changes. Any computation that has
  // declared "all sources" dependence will also have been invalidated for the
  // same reason so doesn't need to explicitly list cache entries.
  auto& all_sources_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kAllSourcesTicket), "all sources");
  all_sources_tracker.SubscribeToPrerequisite(&time_tracker);
  all_sources_tracker.SubscribeToPrerequisite(&accuracy_tracker);
  all_sources_tracker.SubscribeToPrerequisite(&x_tracker);
  all_sources_tracker.SubscribeToPrerequisite(&p_tracker);
  all_sources_tracker.SubscribeToPrerequisite(&u_tracker);

  // Allocate kinematics trackers to provide a level of abstraction from the
  // specific state variables that are used to represent configuration and
  // rate of change of configuration. For example, a kinematics cache entry
  // should depend on configuration regardless of whether we use continuous or
  // discrete variables. And it should be possible to switch between continuous
  // and discrete representations without having to change the specified
  // dependency, which remains "configuration" either way.

  // Should track changes to configuration regardless of how represented. The
  // default is that the continuous "q" variables represent the configuration.
  auto& configuration_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kConfigurationTicket), "configuration");
  // This default subscription must be changed if configuration is not
  // represented by q in this System.
  configuration_tracker.SubscribeToPrerequisite(&q_tracker);

  // Should track changes to configuration time rate of change (i.e., velocity)
  // regardless of how represented. The default is that the continuous "v"
  // variables represent the configuration rate of change.
  auto& velocity_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kVelocityTicket), "velocity");
  // This default subscription must be changed if velocity is not
  // represented by v in this System.
  velocity_tracker.SubscribeToPrerequisite(&v_tracker);

  // This tracks configuration & velocity regardless of how represented.
  auto& kinematics_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kKinematicsTicket), "kinematics");
  kinematics_tracker.SubscribeToPrerequisite(&configuration_tracker);
  kinematics_tracker.SubscribeToPrerequisite(&velocity_tracker);

  auto& xcdot_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kXcdotTicket), "xcdot");
  // TODO(sherm1) Connect to cache entry.
  unused(xcdot_tracker);

  auto& xdhat_tracker = graph.CreateNewDependencyTracker(
      DependencyTicket(internal::kXdhatTicket), "xdhat");
  // TODO(sherm1) Connect to cache entry.
  unused(xdhat_tracker);
}

void ContextBase::BuildTrackerPointerMap(
    const ContextBase& clone,
    DependencyTracker::PointerMap* tracker_map) const {
  // First map the pointers local to this context.
  graph_.AppendToTrackerPointerMap(clone.get_dependency_graph(),
                                   &(*tracker_map));
  // TODO(sherm1) Recursive update of descendents goes here.
}

void ContextBase::FixContextPointers(
    const ContextBase& source,
    const DependencyTracker::PointerMap& tracker_map) {
  // First repair pointers local to this context.
  graph_.RepairTrackerPointers(source.get_dependency_graph(), tracker_map, this,
                               &cache_);
  // Cache and FixedInputs only need their back pointers set to `this`.
  cache_.RepairCachePointers(this);
  for (auto& fixed_input : input_port_values_) {
    if (fixed_input != nullptr) {
      detail::ContextBaseFixedInputAttorney::set_owning_subcontext(
          fixed_input.get_mutable(), this);
    }
  }

  // TODO(sherm1) Recursive update of descendents goes here.
}

}  // namespace systems
}  // namespace drake
