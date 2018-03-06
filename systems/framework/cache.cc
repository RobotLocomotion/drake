// TODO(sherm1) Re-review this file in its entirety when the cache stubs are
// replaced with real code in a subsequent PR.

#include "drake/systems/framework/cache.h"

#include "drake/systems/framework/dependency_tracker.h"

namespace drake {
namespace systems {

CacheEntryValue& Cache::CreateNewCacheEntryValue(
    CacheIndex index, DependencyTicket ticket,
    const std::string& description,
    const std::vector<DependencyTicket>& prerequisites,
    DependencyGraph* trackers) {
  DRAKE_DEMAND(trackers != nullptr);
  DRAKE_DEMAND(index.is_valid() && ticket.is_valid());

  // Make sure there is a place for this cache entry in the cache.
  if (index >= num_entries())
    store_.resize(index + 1);

  // Create the new cache entry value and install it into this Cache. Note that
  // indirection here means the CacheEntryValue object's address is stable
  // even when store_ is resized.
  DRAKE_DEMAND(store_[index] == nullptr);
  store_[index] = std::make_unique<CacheEntryValue>(
      index, ticket, description, nullptr /* no value yet */);
  CacheEntryValue& value = *store_[index];

  // Allocate a DependencyTracker for this cache entry. Note that a pointer
  // to the new CacheEntryValue is retained so must have a lifetime matching
  // the tracker. That requires that the Cache and DependencyGraph are contained
  // in the same Context.
  DependencyTracker& tracker = trackers->CreateNewDependencyTracker(
      ticket,
      "cache " + description,
      &value);

  // Subscribe to prerequisites (trackers must already exist).
  for (auto prereq : prerequisites) {
    auto& prereq_tracker = trackers->get_mutable_tracker(prereq);
    tracker.SubscribeToPrerequisite(&prereq_tracker);
  }
  return value;
}

}  // namespace systems
}  // namespace drake
