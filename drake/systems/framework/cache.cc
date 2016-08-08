#include "drake/systems/framework/cache.h"

#include <memory>
#include <limits>

#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {

Cache::Cache() {}

Cache::~Cache() {}

CacheTicket Cache::MakeCacheTicket(std::set<CacheTicket> dependencies) {
  // Create a new ticket.
  CacheTicket ticket = static_cast<int>(store_.size());

  // Add the ticket to the dependency map. Because dependencies may not be
  // added after the fact, the dependency map is guaranteed acyclic.
  for (const CacheTicket& dependency : dependencies) {
    cache_dependencies_[dependency].insert(ticket);
  }

  // Reserve a null unique_ptr<AbstractValue> for this ticket, which is
  // initially not valid.
  store_.push_back(CacheEntry{false, nullptr});
  return ticket;
}

void Cache::Invalidate(CacheTicket ticket) {
  const std::set<CacheTicket>& to_invalidate{ticket};
  InvalidateRecursively(to_invalidate);
}

void Cache::InvalidateRecursively(const std::set<CacheTicket>& to_invalidate) {
  for (CacheTicket ticket : to_invalidate) {
    // Delete the ticket's cache value.
    store_[ticket].value.reset(nullptr);
    // Visit all the dependents of this ticket.
    InvalidateRecursively(cache_dependencies_[ticket]);
  }
}

AbstractValue* Cache::Set(CacheTicket ticket,
                          std::unique_ptr<AbstractValue> value) {
  DRAKE_ABORT_UNLESS(ticket < static_cast<int>(store_.size()));
  store_[ticket].is_valid = true;
  store_[ticket].value = std::move(value);
  return store_[ticket].value.get();
}

AbstractValue* Cache::Get(CacheTicket ticket) const {
  DRAKE_ABORT_UNLESS(ticket < static_cast<int>(store_.size()));
  if (store_[ticket].is_valid) {
    return store_[ticket].value.get();
  } else {
    return nullptr;
  }
}

std::unique_ptr<Cache> Cache::Clone() const {
  auto clone = std::make_unique<Cache>();
  clone->cache_dependencies_ = cache_dependencies_;
  for (const auto& element : store_) {
    clone->store_.push_back(
        CacheEntry{element.is_valid, element.value->Clone()});
  }
  return clone;
}

}  // namespace systems
}  // namespace drake
