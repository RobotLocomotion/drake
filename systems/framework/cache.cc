#include "drake/systems/framework/cache.h"

#include <memory>
#include <utility>

#include "drake/common/text_logging.h"
#include "drake/systems/framework/cache_entry.h"
#include "drake/systems/framework/context_base.h"
#include "drake/systems/framework/dependency_tracker.h"

namespace drake {
namespace systems {

CacheEntryValue& Cache::CreateNewCacheEntryValue(
    const CacheEntry& entry, DependencyGraph* trackers) {
  DRAKE_DEMAND(trackers != nullptr);
  const CacheIndex index = entry.cache_index();
  const DependencyTicket ticket = entry.ticket();
  DRAKE_DEMAND(index.is_valid() && ticket.is_valid());

  // Make sure there is a place for this cache entry in the cache.
  if (index >= num_entries())
    store_.resize(index + 1);

  store_[index] = std::make_unique<CacheEntryValue>(
      index, ticket, entry.description(), nullptr /* no value yet */);
  CacheEntryValue& value = *store_[index];

  // Allocate a DependencyTracker for this cache entry.
  DependencyTracker& tracker = trackers->CreateNewDependencyTracker(
      ticket,
      "cache " + entry.description(),
      &value);

  // Subscribe to prerequisites (trackers must already exist).
  for (auto prereq : entry.prerequisites()) {
    auto& prereq_tracker = trackers->get_mutable_tracker(prereq);
    tracker.SubscribeToPrerequisite(&prereq_tracker);
  }
  return value;
}

}  // namespace systems
}  // namespace drake
