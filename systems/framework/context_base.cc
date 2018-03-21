#include "drake/systems/framework/context_base.h"

#include <string>
#include <typeinfo>

namespace drake {
namespace systems {

std::unique_ptr<ContextBase> ContextBase::Clone() const {
  std::unique_ptr<ContextBase> clone_ptr(CloneWithoutPointers());

  // Verify that the most-derived Context didn't forget to override
  // CloneWithoutPointers().
  const ContextBase& source = *this;  // Deref here to avoid typeid warning.
  ContextBase& clone = *clone_ptr;
  DRAKE_ASSERT(typeid(source) == typeid(clone));

  // Create a complete mapping of tracker pointers.
  DependencyTracker::PointerMap tracker_map;
  BuildTrackerPointerMap(clone, &tracker_map);

  // Then do a pointer fixup pass.
  clone.FixTrackerPointers(source, tracker_map);
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
  // TODO(sherm1) Replace with the real pathname.
  return "/dummy/system/pathname";
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

  // TODO(sherm1) Add the rest of the built-in trackers here.
}

void ContextBase::BuildTrackerPointerMap(
    const ContextBase& clone,
    DependencyTracker::PointerMap* tracker_map) const {
  // First map the pointers local to this context.
  graph_.AppendToTrackerPointerMap(clone.get_dependency_graph(),
                                   &(*tracker_map));
  // TODO(sherm1) Recursive update of descendents goes here.
}

void ContextBase::FixTrackerPointers(
    const ContextBase& source,
    const DependencyTracker::PointerMap& tracker_map) {
  // First repair pointers local to this context.
  graph_.RepairTrackerPointers(source.get_dependency_graph(), tracker_map, this,
                               &cache_);
  // Cache and only needs its back pointers set to this.
  cache_.RepairCachePointers(this);
  // TODO(sherm1) Recursive update of descendents goes here.
}

}  // namespace systems
}  // namespace drake
