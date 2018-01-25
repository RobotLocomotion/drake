#include "drake/systems/framework/system_base.h"

namespace drake {
namespace systems {

std::unique_ptr<ContextBase> SystemBase::MakeContext() const {
  // Derived class creates the concrete Context object, which already contains
  // all the well-known trackers (the ones with fixed tickets).
  std::unique_ptr<ContextBase> context_ptr = DoMakeContext();
  DRAKE_DEMAND(context_ptr != nullptr);
  ContextBase& context = *context_ptr;

  context.set_system_name(get_name());

  // Add the independent-source trackers and wire them up appropriately. That
  // includes input ports since their dependencies are external.
  CreateSourceTrackers(&context);

  DependencyGraph& trackers = context.get_mutable_dependency_graph();

  // Create the Context cache containing a CacheEntryValue corresponding to
  // each CacheEntry, add a DependencyTracker and subscribe it to its
  // prerequisites as specified in the CacheEntry. Cache entries are
  // necessarily ordered such that the first cache entry can depend only
  // on the known source trackers created above, the second may depend on
  // those plus the first, and so on. Circular dependencies are not permitted.
  Cache& cache = context.get_mutable_cache();
  for (CacheIndex index(0); index < num_cache_entries(); ++index) {
    const CacheEntry& entry = get_cache_entry(index);
    cache.CreateNewCacheEntryValue(entry.cache_index(), entry.ticket(),
        entry.description(), entry.prerequisites(), &trackers);
  }

  // Create the output port trackers yᵢ here. Nothing in this System may
  // depend on them; subscribers will be input ports from peer Systems or
  // an exported output port in the parent Diagram. The associated cache entries
  // were just created above. If the output port's prerequisite is just a
  // cache entry in this subsystem, set up that dependency here. For output
  // ports that have been exported from a child subsystem, defer setting up the
  // dependency until we're doing inter-subsystem dependencies later.
  context.output_port_tickets().clear();
  for (const auto& oport : output_ports_) {
    auto& yi_tracker = trackers.CreateNewDependencyTracker(
        oport->ticket(), "y_" + std::to_string(oport->get_index()));
    context.output_port_tickets().push_back(oport->ticket());
    std::pair<optional<SubsystemIndex>, DependencyTicket> prerequisite =
        oport->GetPrerequisite();
    if (prerequisite.first)
      continue;  // Depends on some other subsystem; defer until later.
    yi_tracker.SubscribeToPrerequisite(
        &context.get_mutable_tracker(prerequisite.second));
  }

  return context_ptr;
}

void SystemBase::AcquireContextResources(ContextBase* context) const {
  DRAKE_DEMAND(context != nullptr);
  // Let the derived class acquire its needed resources and validate
  // that it can handle a System with this structure.
  DoAcquireContextResources(&*context);

  // We now have a complete Context. We can allocate space for cache entry
  // values using the allocators, which require a context.
  Cache& cache = context->get_mutable_cache();
  for (CacheIndex index(0); index < num_cache_entries(); ++index) {
    const CacheEntry& entry = get_cache_entry(index);
    CacheEntryValue& cache_value = cache.get_mutable_cache_entry_value(index);
    cache_value.SetInitialValue(entry.Allocate(*context));
  }
}

// Set up trackers for variable-numbered independent sources: discrete and
// abstract state, numerical and abstract parameters, and input ports.
// The generic trackers like "all parameters" are already present in the
// supplied Context, but we have to subscribe them to the individual
// elements now.
void SystemBase::CreateSourceTrackers(ContextBase* context_ptr) const {
  ContextBase& context = *context_ptr;
  DependencyGraph& trackers = context.get_mutable_dependency_graph();

  // Define a lambda to do the repeated work below: create trackers for
  // individual entities and subscribe the group tracker to each of them.
  auto MakeTrackers = [&trackers](
      DependencyTicket subscriber_ticket,
      const std::vector<TrackerInfo>& system_ticket_info,
      std::vector<DependencyTicket>* context_tickets) {
    DependencyTracker& subscriber =
        trackers.get_mutable_tracker(subscriber_ticket);
    context_tickets->clear();
    for (const auto& info : system_ticket_info) {
      auto& source_tracker =
          trackers.CreateNewDependencyTracker(info.ticket, info.description);
      context_tickets->push_back(info.ticket);
      subscriber.SubscribeToPrerequisite(&source_tracker);
    }
  };

  // Allocate trackers for each discrete variable group xdᵢ, and subscribe
  // the "all discrete variables" tracker xd to those.
  MakeTrackers(xd_ticket(), discrete_state_tickets_,
               &context.discrete_state_tickets());

  // Allocate trackers for each abstract state variable xaᵢ, and subscribe
  // the "all abstract variables" tracker xa to those.
  MakeTrackers(xa_ticket(), abstract_state_tickets_,
               &context.abstract_state_tickets());

  // Allocate trackers for each numeric parameter pnᵢ and each abstract
  // parameter paᵢ, and subscribe the "all parameters" tracker p to those.
  MakeTrackers(all_parameters_ticket(), numeric_parameter_tickets_,
               &context.numeric_parameter_tickets());
  MakeTrackers(all_parameters_ticket(), abstract_parameter_tickets_,
               &context.abstract_parameter_tickets());

  // Allocate trackers for each input port uᵢ, and subscribe the "all input
  // ports" tracker u to those. (Doesn't use TrackerInfo so can't use lambda.)
  for (const auto& iport : input_ports_) {
    context.AddInputPort(iport->get_index(), iport->ticket());
  }
}

}  // namespace systems
}  // namespace drake
