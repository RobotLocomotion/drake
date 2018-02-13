#include "drake/systems/framework/cache.h"

#include <memory>
#include <utility>

#include "drake/common/text_logging.h"
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

  store_[index] = std::make_unique<CacheEntryValue>(
      index, ticket, description, nullptr /* no value yet */);
  CacheEntryValue& value = *store_[index];

  // Allocate a DependencyTracker for this cache entry.
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
