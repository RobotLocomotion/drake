#include "drake/systems/framework/system_base.h"

namespace drake {
namespace systems {

SystemBase::~SystemBase() {}

std::string SystemBase::GetSystemPathname() const {
  // TODO(sherm1) Replace with the real pathname.
  return "/dummy/system/pathname/" + GetSystemName();
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

  // TODO(sherm1) Set context system name from GetSystemName().

  // TODO(sherm1) Add the independent-source trackers and wire them up
  // appropriately. That includes input ports since their dependencies are
  // external.

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

}  // namespace systems
}  // namespace drake
