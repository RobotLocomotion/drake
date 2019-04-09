#include "drake/systems/framework/system_base.h"

#include <algorithm>
#include <deque>

#include <fmt/format.h>

#include "drake/systems/framework/fixed_input_port_value.h"

namespace {

// Output a string like "System::EvalInput()".
std::string FmtFunc(const char* func) {
  return fmt::format("System::{}()", func);
}

}

namespace drake {
namespace systems {

SystemBase::~SystemBase() {}

std::string SystemBase::GetSystemPathname() const {
  const std::string parent_path =
      get_parent_service() ? get_parent_service()->GetParentPathname()
                           : std::string();
  return parent_path + internal::SystemMessageInterface::path_separator() +
         GetSystemName();
}

const CacheEntry& SystemBase::DeclareCacheEntry(
    std::string description, CacheEntry::AllocCallback alloc_function,
    CacheEntry::CalcCallback calc_function,
    std::set<DependencyTicket> prerequisites_of_calc) {
  return DeclareCacheEntryWithKnownTicket(
      assign_next_dependency_ticket(), std::move(description),
      std::move(alloc_function), std::move(calc_function),
      std::move(prerequisites_of_calc));
}

const CacheEntry& SystemBase::DeclareCacheEntryWithKnownTicket(
    DependencyTicket known_ticket, std::string description,
    CacheEntry::AllocCallback alloc_function,
    CacheEntry::CalcCallback calc_function,
    std::set<DependencyTicket> prerequisites_of_calc) {
  // If the prerequisite list is empty the CacheEntry constructor will throw
  // a logic error.
  const CacheIndex index(num_cache_entries());
  cache_entries_.emplace_back(std::make_unique<CacheEntry>(
      this, index, known_ticket, std::move(description),
      std::move(alloc_function), std::move(calc_function),
      std::move(prerequisites_of_calc)));
  const CacheEntry& new_entry = *cache_entries_.back();
  return new_entry;
}

void SystemBase::InitializeContextBase(ContextBase* context_ptr) const {
  DRAKE_DEMAND(context_ptr != nullptr);
  ContextBase& context = *context_ptr;

  // Initialization should happen only once per Context.
  DRAKE_DEMAND(
      !detail::SystemBaseContextBaseAttorney::is_context_base_initialized(
          context));

  detail::SystemBaseContextBaseAttorney::set_system_name(&context, get_name());

  // Add the independent-source trackers and wire them up appropriately. That
  // includes input ports since their dependencies are external.
  CreateSourceTrackers(&context);

  DependencyGraph& graph = context.get_mutable_dependency_graph();

  // Create the Context cache containing a CacheEntryValue corresponding to
  // each CacheEntry, add a DependencyTracker and subscribe it to its
  // prerequisites as specified in the CacheEntry. Cache entries are
  // necessarily ordered such that the first cache entry can depend only
  // on the known source trackers created above, the second may depend on
  // those plus the first, and so on. Circular dependencies are not permitted.
  Cache& cache = context.get_mutable_cache();
  for (CacheIndex index(0); index < num_cache_entries(); ++index) {
    const CacheEntry& entry = get_cache_entry(index);
    CacheEntryValue& cache_value = cache.CreateNewCacheEntryValue(
        entry.cache_index(), entry.ticket(), entry.description(),
        entry.prerequisites(), &graph);
    // TODO(sherm1) Supply initial value on creation instead and get rid of
    // this separate call.
    cache_value.SetInitialValue(entry.Allocate());
  }

  // Create the output port trackers yᵢ here. Nothing in this System may
  // depend on them; subscribers will be input ports from peer subsystems or
  // an exported output port in the parent Diagram. The associated cache entries
  // were just created above. Any intra-system prerequisites are set up now.
  for (const auto& oport : output_ports_) {
    detail::SystemBaseContextBaseAttorney::AddOutputPort(
        &context, oport->get_index(), oport->ticket(),
        oport->GetPrerequisite());
  }

  detail::SystemBaseContextBaseAttorney::mark_context_base_initialized(
      &context);
}

OutputPortBase::OptionalInputPortIndices SystemBase::InputsFromTickets(
    const std::set<DependencyTicket>& prerequisites) const {
  OutputPortBase::OptionalInputPortIndices result;

  // Fast-path handling for the default case.  If all_sources is used, then we
  // should let other mechanisms (symbolic, DoHasDirectFeedthrough) respond.
  if (prerequisites.count(all_sources_ticket()) > 0) {
    DRAKE_DEMAND(!result.has_value());
    return result;
  }

  // If the output depends on all inputs, then we can't enumerate all of their
  // indices here because they might not have been declared yet.
  if (prerequisites.count(all_input_ports_ticket()) > 0) {
    DRAKE_DEMAND(!result.has_value());
    return result;
  }

  // For now, we can't check the dependencies of cache entries.
  for (const auto& cache_entry_ptr : cache_entries_) {
    if (prerequisites.count(cache_entry_ptr->ticket()) > 0) {
      DRAKE_DEMAND(!result.has_value());
      return result;
    }
  }

  // Collect the list of tickets known to never depend on the inputs.
  std::vector<DependencyTicket> non_inputs{
      nothing_ticket(),
      time_ticket(),
      accuracy_ticket(),
      q_ticket(),
      v_ticket(),
      z_ticket(),
      xc_ticket(),
      xd_ticket(),
      xa_ticket(),
      all_state_ticket(),
      pn_ticket(),
      pa_ticket(),
      all_parameters_ticket(),
      configuration_ticket(),
      kinematics_ticket(),
      xcdot_ticket(),
      pe_ticket(),
      ke_ticket(),
      pc_ticket(),
      pnc_ticket(),
  };
  for (const auto& ticket_vector : {
          discrete_state_tickets_, abstract_state_tickets_,
          numeric_parameter_tickets_, abstract_parameter_tickets_ }) {
    for (const auto& tracker_info : ticket_vector) {
      non_inputs.push_back(tracker_info.ticket);
    }
  }
  for (const auto& output_port_base_ptr : output_ports_) {
    non_inputs.push_back(output_port_base_ptr->ticket());
  }
  std::sort(non_inputs.begin(), non_inputs.end());

  // Compute (prerequisites - non_inputs) which should be only input tickets.
  std::deque<DependencyTicket> inputs;
  std::set_difference(
      prerequisites.begin(), prerequisites.end(),
      non_inputs.begin(), non_inputs.end(),
      std::back_inserter(inputs));

  // Convert input tickets to input indices.
  result = std::vector<InputPortIndex>();
  for (const auto& input_port_base_ptr : input_ports_) {
    if (!inputs.empty() && (inputs.front() == input_port_base_ptr->ticket())) {
      result->push_back(input_port_base_ptr->get_index());
      inputs.pop_front();
    }
  }

  // At this point we've covered *all* tickets.
  DRAKE_DEMAND(inputs.empty());

  return result;
}

// Set up trackers for variable-numbered independent sources: discrete and
// abstract state, numerical and abstract parameters, and input ports.
// The generic trackers like "all parameters" are already present in the
// supplied Context, but we have to subscribe them to the individual
// elements now.
void SystemBase::CreateSourceTrackers(ContextBase* context_ptr) const {
  ContextBase& context = *context_ptr;

  // Define a lambda to do the repeated work below: create trackers for
  // individual entities and subscribe the group tracker to each of them.
  auto make_trackers = [&context](
      DependencyTicket subscriber_ticket,
      const std::vector<TrackerInfo>& system_ticket_info,
      void (*add_ticket_to_context)(ContextBase*, DependencyTicket)) {
    DependencyGraph& graph = context.get_mutable_dependency_graph();
    DependencyTracker& subscriber =
        graph.get_mutable_tracker(subscriber_ticket);

    for (const auto& info : system_ticket_info) {
      auto& source_tracker =
          graph.CreateNewDependencyTracker(info.ticket, info.description);
      add_ticket_to_context(&context, info.ticket);
      subscriber.SubscribeToPrerequisite(&source_tracker);
    }
  };

  // Allocate trackers for each discrete variable group xdᵢ, and subscribe
  // the "all discrete variables" tracker xd to those.
  make_trackers(
      xd_ticket(), discrete_state_tickets_,
      &detail::SystemBaseContextBaseAttorney::AddDiscreteStateTicket);

  // Allocate trackers for each abstract state variable xaᵢ, and subscribe
  // the "all abstract variables" tracker xa to those.
  make_trackers(
      xa_ticket(), abstract_state_tickets_,
      &detail::SystemBaseContextBaseAttorney::AddAbstractStateTicket);

  // Allocate trackers for each numeric parameter pnᵢ and each abstract
  // parameter paᵢ, and subscribe the pn and pa trackers to them.
  make_trackers(
      pn_ticket(), numeric_parameter_tickets_,
      &detail::SystemBaseContextBaseAttorney::AddNumericParameterTicket);
  make_trackers(
      pa_ticket(), abstract_parameter_tickets_,
      &detail::SystemBaseContextBaseAttorney::AddAbstractParameterTicket);

  // Allocate trackers for each input port uᵢ, and subscribe the "all input
  // ports" tracker u to them. Doesn't use TrackerInfo so can't use the lambda.
  for (const auto& iport : input_ports_) {
    detail::SystemBaseContextBaseAttorney::AddInputPort(
        &context, iport->get_index(), iport->ticket(),
        MakeFixInputPortTypeChecker(iport->get_index()));
  }
}

// The only way for a system to evaluate its own input port is if that
// port is fixed. In that case the port's value is in the corresponding
// subcontext and we can just return it. Otherwise, the port obtains its value
// from some other system and we need our parent's help to get access to
// that system.
const AbstractValue* SystemBase::EvalAbstractInputImpl(
    const char* func, const ContextBase& context,
    InputPortIndex port_index) const {
  if (port_index >= num_input_ports())
    ThrowInputPortIndexOutOfRange(func, port_index);

  const FixedInputPortValue* const free_port_value =
      context.MaybeGetFixedInputPortValue(port_index);

  if (free_port_value != nullptr)
    return &free_port_value->get_value();  // A fixed input port.

  // The only way to satisfy an input port of a root System is to make it fixed.
  // Since it wasn't fixed, it is unconnected.
  if (get_parent_service() == nullptr) return nullptr;

  // This is not the root System, and the port isn't fixed, so ask our parent to
  // evaluate it.
  return get_parent_service()->EvalConnectedSubsystemInputPort(
      *detail::SystemBaseContextBaseAttorney::get_parent_base(context),
      get_input_port_base(port_index));
}

void SystemBase::ThrowNegativePortIndex(const char* func,
                                        int port_index) const {
  DRAKE_DEMAND(port_index < 0);
  throw std::out_of_range(
      fmt::format("{}: negative port index {} is illegal. (System {})",
                  FmtFunc(func), port_index, GetSystemPathname()));
}

void SystemBase::ThrowInputPortIndexOutOfRange(const char* func,
                                               InputPortIndex port) const {
  throw std::out_of_range(fmt::format(
      "{}: there is no input port with index {} because there "
      "are only {} input ports in system {}.",
      FmtFunc(func),  port, num_input_ports(), GetSystemPathname()));
}

void SystemBase::ThrowOutputPortIndexOutOfRange(const char* func,
                                                OutputPortIndex port) const {
  throw std::out_of_range(fmt::format(
      "{}: there is no output port with index {} because there "
      "are only {} output ports in system {}.",
      FmtFunc(func), port,
      num_output_ports(), GetSystemPathname()));
}

void SystemBase::ThrowNotAVectorInputPort(const char* func,
                                          InputPortIndex port) const {
  throw std::logic_error(fmt::format(
      "{}: vector port required, but input port '{}' (index {}) was declared "
          "abstract. Even if the actual value is a vector, use "
          "EvalInputValue<V> instead for an abstract port containing a vector "
          "of type V. (System {})",
      FmtFunc(func),  get_input_port_base(port).get_name(), port,
      GetSystemPathname()));
}

void SystemBase::ThrowInputPortHasWrongType(
    const char* func, InputPortIndex port, const std::string& expected_type,
    const std::string& actual_type) const {
  ThrowInputPortHasWrongType(
      func, GetSystemPathname(), port, get_input_port_base(port).get_name(),
      expected_type, actual_type);
}

void SystemBase::ThrowInputPortHasWrongType(
    const char* func, const std::string& system_pathname, InputPortIndex port,
    const std::string& port_name, const std::string& expected_type,
    const std::string& actual_type) {
  throw std::logic_error(fmt::format(
      "{}: expected value of type {} for input port '{}' (index {}) "
          "but the actual type was {}. (System {})",
      FmtFunc(func), expected_type, port_name, port, actual_type,
      system_pathname));
}

void SystemBase::ThrowCantEvaluateInputPort(const char* func,
                                            InputPortIndex port) const {
  throw std::logic_error(
      fmt::format("{}: input port '{}' (index {}) is neither connected nor "
                      "fixed so cannot be evaluated. (System {})",
                  FmtFunc(func), get_input_port_base(port).get_name(), port,
                  GetSystemPathname()));
}

}  // namespace systems
}  // namespace drake
