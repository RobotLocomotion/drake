#include "drake/systems/framework/system_base.h"

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
    std::vector<DependencyTicket> prerequisites_of_calc) {
  // If the prerequisite list is empty the CacheEntry constructor will throw
  // a logic error.
  const CacheIndex index(num_cache_entries());
  const DependencyTicket ticket(assign_next_dependency_ticket());
  cache_entries_.emplace_back(std::make_unique<CacheEntry>(
      this, index, ticket, std::move(description), std::move(alloc_function),
      std::move(calc_function), std::move(prerequisites_of_calc)));
  const CacheEntry& new_entry = *cache_entries_.back();
  return new_entry;
}

std::unique_ptr<ContextBase> SystemBase::MakeContext() const {
  // Derived class creates the concrete Context object, which already contains
  // all the well-known trackers (the ones with fixed tickets).
  std::unique_ptr<ContextBase> context_ptr = DoMakeContext();
  DRAKE_DEMAND(context_ptr != nullptr);
  ContextBase& context = *context_ptr;

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
    cache.CreateNewCacheEntryValue(entry.cache_index(), entry.ticket(),
                                   entry.description(), entry.prerequisites(),
                                   &graph);
  }

  // TODO(sherm1) Create the output port trackers yᵢ here.

  // TODO(sherm1) Move this to the AcquireContextResources phase.
  // We now have a complete Context. We can allocate space for cache entry
  // values using the allocators, which require a context.
  for (CacheIndex index(0); index < num_cache_entries(); ++index) {
    const CacheEntry& entry = get_cache_entry(index);
    CacheEntryValue& cache_value = cache.get_mutable_cache_entry_value(index);
    cache_value.SetInitialValue(entry.Allocate());
  }

  return context_ptr;
}

// Set up trackers for variable-numbered independent sources: discrete and
// abstract state, numerical and abstract parameters, and input ports.
// The generic trackers like "all parameters" are already present in the
// supplied Context, but we have to subscribe them to the individual
// elements now.
void SystemBase::CreateSourceTrackers(ContextBase* context_ptr) const {
  ContextBase& context = *context_ptr;

  // TODO(sherm1) Add state and parameter trackers here.

  // Allocate trackers for each input port uᵢ. Note that this also takes care of
  // subscribing the "all input ports" tracker u to them.
  for (const auto& iport : input_ports_) {
    context.AddInputPort(iport->get_index(), iport->ticket());
  }
}

// The only way for a subsystem to evaluate its own input port is if that
// port is fixed. In that case the port's value is in the corresponding
// subcontext and we can just return it. Otherwise, the port obtains its value
// from some other subsystem and we need our parent's help to get access to
// that subsystem.
const AbstractValue* SystemBase::EvalAbstractInputImpl(
    const char* func, const ContextBase& context,
    InputPortIndex port_index) const {
  if (port_index >= get_num_input_ports())
    ThrowInputPortIndexOutOfRange(func, port_index, get_num_input_ports());

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

void SystemBase::ThrowNegativeInputPortIndex(const char* func,
                                             int port_index) const {
  DRAKE_DEMAND(port_index < 0);
  throw std::out_of_range(
      fmt::format("{}: negative port index {} is illegal. (Subsystem {})",
                  FmtFunc(func), port_index, GetSystemPathname()));
}

void SystemBase::ThrowInputPortIndexOutOfRange(const char* func,
                                               InputPortIndex port,
                                               int num_input_ports) const {
  DRAKE_DEMAND(num_input_ports >= 0);
  throw std::out_of_range(
      fmt::format("{}: there is no input port with index {} because there "
                      "are only {} input ports in subsystem {}.",
                  FmtFunc(func), port, num_input_ports, GetSystemPathname()));
}

void SystemBase::ThrowNotAVectorInputPort(const char* func,
                                          InputPortIndex port) const {
  throw std::logic_error(fmt::format(
      "{}: vector port required, but input port[{}] was declared abstract. "
          "Even if the actual value is a vector, use EvalInputValue<V> "
          "instead for an abstract port containing a vector of type V. "
          "(Subsystem {})",
      FmtFunc(func), port, GetSystemPathname()));
}

void SystemBase::ThrowInputPortHasWrongType(
    const char* func, InputPortIndex port, const std::string& expected_type,
    const std::string& actual_type) const {
  throw std::logic_error(fmt::format(
      "{}: expected value of type {} for input port[{}] "
          "but the actual type was {}. (Subsystem {})",
      FmtFunc(func), expected_type, port, actual_type, GetSystemPathname()));
}

void SystemBase::ThrowCantEvaluateInputPort(const char* func,
                                           InputPortIndex port) const {
  throw std::logic_error(
      fmt::format("{}: input port[{}] is neither connected nor fixed so "
                      "cannot be evaluated. (Subsystem {})",
                  FmtFunc(func), port, GetSystemPathname()));
}

}  // namespace systems
}  // namespace drake
