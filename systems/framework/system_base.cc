#include "drake/systems/framework/system_base.h"

#include <atomic>
#include <stdexcept>

#include <fmt/format.h>

#include "drake/common/unused.h"
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

internal::SystemId SystemBase::get_next_id() {
  return internal::SystemId::get_new_id();
}

std::string SystemBase::GetSystemPathname() const {
  const std::string parent_path =
      get_parent_service() ? get_parent_service()->GetParentPathname()
                           : std::string();
  return parent_path + internal::SystemMessageInterface::path_separator() +
         GetSystemName();
}

CacheEntry& SystemBase::DeclareCacheEntry(
    std::string description, ValueProducer value_producer,
    std::set<DependencyTicket> prerequisites_of_calc) {
  return DeclareCacheEntryWithKnownTicket(
      assign_next_dependency_ticket(), std::move(description),
      std::move(value_producer), std::move(prerequisites_of_calc));
}

CacheEntry& SystemBase::DeclareCacheEntryWithKnownTicket(
    DependencyTicket known_ticket, std::string description,
    ValueProducer value_producer,
    std::set<DependencyTicket> prerequisites_of_calc) {
  // If the prerequisite list is empty the CacheEntry constructor will throw
  // a logic error.
  const CacheIndex index(num_cache_entries());
  cache_entries_.emplace_back(std::make_unique<CacheEntry>(
      this, index, known_ticket, std::move(description),
      std::move(value_producer), std::move(prerequisites_of_calc)));
  CacheEntry& new_entry = *cache_entries_.back();
  return new_entry;
}

void SystemBase::InitializeContextBase(ContextBase* context_ptr) const {
  DRAKE_DEMAND(context_ptr != nullptr);
  ContextBase& context = *context_ptr;

  // Initialization should happen only once per Context.
  DRAKE_DEMAND(
      !internal::SystemBaseContextBaseAttorney::is_context_base_initialized(
          context));

  internal::SystemBaseContextBaseAttorney::set_system_name(
      &context, get_name());
  internal::SystemBaseContextBaseAttorney::set_system_id(
      &context, system_id_);

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

    if (entry.is_disabled_by_default())
      cache_value.disable_caching();
  }

  // Create the output port trackers yᵢ here. Nothing in this System may
  // depend on them; subscribers will be input ports from peer subsystems or
  // an exported output port in the parent Diagram. The associated cache entries
  // were just created above. Any intra-system prerequisites are set up now.
  for (const auto& oport : output_ports_) {
    internal::SystemBaseContextBaseAttorney::AddOutputPort(
        &context, oport->get_index(), oport->ticket(),
        oport->GetPrerequisite());
  }

  internal::SystemBaseContextBaseAttorney::mark_context_base_initialized(
      &context);
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
      &internal::SystemBaseContextBaseAttorney::AddDiscreteStateTicket);

  // Allocate trackers for each abstract state variable xaᵢ, and subscribe
  // the "all abstract variables" tracker xa to those.
  make_trackers(
      xa_ticket(), abstract_state_tickets_,
      &internal::SystemBaseContextBaseAttorney::AddAbstractStateTicket);

  // Allocate trackers for each numeric parameter pnᵢ and each abstract
  // parameter paᵢ, and subscribe the pn and pa trackers to them.
  make_trackers(
      pn_ticket(), numeric_parameter_tickets_,
      &internal::SystemBaseContextBaseAttorney::AddNumericParameterTicket);
  make_trackers(
      pa_ticket(), abstract_parameter_tickets_,
      &internal::SystemBaseContextBaseAttorney::AddAbstractParameterTicket);

  // Allocate trackers for each input port uᵢ, and subscribe the "all input
  // ports" tracker u to them. Doesn't use TrackerInfo so can't use the lambda.
  for (const auto& iport : input_ports_) {
    internal::SystemBaseContextBaseAttorney::AddInputPort(
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

  // If this is a root Context, our parent can't evaluate it.
  if (context.is_root_context()) return nullptr;

  // This is not the root System, and the port isn't fixed, so ask our parent to
  // evaluate it.
  return get_parent_service()->EvalConnectedSubsystemInputPort(
      *internal::SystemBaseContextBaseAttorney::get_parent_base(context),
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

void SystemBase::ThrowValidateContextMismatch(
    const ContextBase& context) const {
  const char* const info_link =
      "For more information about Context-System mismatches, see "
      "https://drake.mit.edu/"
      "troubleshooting.html#framework-context-system-mismatch";

  // N.B. This is subtle logic that relies on what can/can't be true by the
  // time we evaluate each test. Here's the explanation of the logic.
  //
  // We're given the context C and `this` system S. We define R and D as the
  // root ancestor context of C and the root diagram D of system S,
  // respectively. It is perfectly possible for C = R and S = D. We define X.id
  // as the system id associated with X (either context or system). Finally, we
  // use the notation X.id ∈ D to indicate the system named by X.id is a
  // *descendant* of the diagram rooted at D.
  //
  // The default disposition is to throw the "generic" message: "some system was
  // called with some other system's context". We throw special messages iff
  // one of two conditions is satisfied.
  //
  //    1. C.id = D.id and S ≠ D: Root context passed to subsystem.
  //    2. C.id ∈ D and S = D: Subsystem context passed to root diagram.
  //
  // So, we have the following possible relationships and results between C, S,
  // R, and D.
  //
  //     State of C  |  State of S |  Outcome
  //    -------------|-------------|-----------
  //  1  C.id = D.id |  S = D      | ValidateContext() already approved this.
  //  2  C.id = D.id |  S ≠ D      | Root context passed to subsystem message.
  //  3  C.id ∈ D    |  S = D      | C != R, R.id = D.id. Leaf context passed to
  //                 |             | root diagram.
  //  4  C.id ∈ D    |  S ≠ D      | C != R, R.id != S.id. Present "generic"
  //                 |             | contact message.
  //  5  C.id ∉ D    |  all cases  | C.id and R.id cannot match D.id or S.id.
  //                 |             | Present "generic" context message.

  // The row 2 condition. get_parent_service() is only non-null for subsystems
  // in Diagrams. I.e., S ≠ D. We then throw iff C.id = D.id.
  if (get_parent_service() != nullptr) {
    const internal::SystemId root_id =  // D.id
        get_parent_service()->GetRootSystemBase().get_system_id();
    if (context.get_system_id() == root_id) {
      throw std::logic_error(fmt::format(
          "A function call on a {} system named '{}' was passed the root "
          "Diagram's Context instead of the appropriate subsystem Context. "
          "Use GetMyContextFromRoot() or similar to acquire the appropriate "
          "subsystem Context.\n{}",
          this->GetSystemType(), this->GetSystemPathname(), info_link));
    }
  }

  // Given C, find R. Note: R = C is a possible return value.
  const ContextBase& root_context = [&context]() -> const ContextBase& {
    const ContextBase* iterator = &context;
    while (true)  {
      const ContextBase* parent =
          internal::SystemBaseContextBaseAttorney::get_parent_base(*iterator);
      if (parent == nullptr) {
        return *iterator;
      }
      iterator = parent;
    }
  }();
  // The row 3 condition. It's not a literal translation of the condition. If
  // C.id ∈ D, then R.id = D.id *must* be true. Therefore, if R.id = S.id is
  // true, then, by transitivity, D.id = S.id → S = D. The condition is met.
  //
  // There is no risk of false positives. Consider the following cases:
  //
  //   - C = R (C has no ancestor context). Then C.id ∈ D cannot be true (as
  //     C.id doesn't refer to a subsystem). (See below for C.id ∉ D.)
  //   - S ≠ D. C.id ∈ D → R.id = D.id. Therefore S.id ≠ R.id. (Row 4)
  //   - C.id ∉ D, neither C.id or R.id can match S.id. (row 5)
  if (root_context.get_system_id() == get_system_id()) {
    throw std::logic_error(fmt::format(
        "A function call on the root Diagram was passed a subcontext "
        "associated with its subsystem named '{}' instead of the root "
        "context. When calling a function on a the root Digram, you must "
        "pass a reference to the root Context, not a subcontext.\n{}",
        context.GetSystemPathname(), info_link));
  }

  throw std::logic_error(fmt::format(
      "A function call on a {} system named '{}' was passed the Context of "
      "a system named '{}' instead of the appropriate subsystem Context.\n{}",
      this->GetSystemType(), this->GetSystemPathname(),
      context.GetSystemPathname(), info_link));
}

void SystemBase::ThrowNotCreatedForThisSystemImpl(
    const std::string& nice_type_name, internal::SystemId id) const {
  if (!id.is_valid()) {
    throw std::logic_error(fmt::format(
        "{} was not associated with any System but should have been "
        "created for {} System {}",
        nice_type_name, GetSystemType(), GetSystemPathname()));
  } else {
    throw std::logic_error(fmt::format("{} was not created for {} System {}",
                                       nice_type_name, GetSystemType(),
                                       GetSystemPathname()));
  }
}

std::string SystemBase::GetUnsupportedScalarConversionMessage(
    const std::type_info& source_type,
    const std::type_info& destination_type) const {
  unused(source_type);
  return fmt::format(
      "System {} of type {} does not support scalar conversion to type {}",
      GetSystemPathname(), GetSystemType(),
      NiceTypeName::Get(destination_type));
}

namespace internal {

std::string DiagramSystemBaseAttorney::GetUnsupportedScalarConversionMessage(
    const SystemBase& system, const std::type_info& source_type,
    const std::type_info& destination_type) {
  return system.GetUnsupportedScalarConversionMessage(
      source_type, destination_type);
}

}  // namespace internal

}  // namespace systems
}  // namespace drake
