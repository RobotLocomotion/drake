#include "drake/systems/framework/cache.h"

#include <memory>
#include <limits>

#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {

namespace internal {

CacheEntry::CacheEntry(const CacheEntry& other) {
  *this = other;
}

CacheEntry& CacheEntry::operator=(const CacheEntry& other) {
  is_valid_ = other.is_valid();
  if (other.value() != nullptr) {
    value_ = other.value()->Clone();
  }
  dependents_ = other.dependents();
  return *this;
}

}  // namespace internal

using internal::CacheEntry;

Cache::Cache() {}

Cache::~Cache() {}

CacheTicket Cache::MakeCacheTicket(std::set<CacheTicket> prerequisites) {
  // Create a new ticket.
  CacheTicket ticket = static_cast<int>(store_.size());

  // Add the ticket to the dependency map. Because prerequisites may not be
  // added after the fact, the dependency map is guaranteed acyclic.
  for (const CacheTicket& prerequisite : prerequisites) {
    store_[prerequisite].add_dependent(ticket);
  }

  // Reserve a null, invalid CacheEntry for this ticket.
  store_.emplace_back();
  return ticket;
}

void Cache::Invalidate(CacheTicket ticket) {
  const std::set<CacheTicket>& to_invalidate{ticket};
  InvalidateRecursively(to_invalidate);
}

void Cache::InvalidateRecursively(const std::set<CacheTicket>& to_invalidate) {
  for (CacheTicket ticket : to_invalidate) {
    // Invalidate the ticket.
    store_[ticket].set_is_valid(false);
    // Visit all the tickets that depend on this one.
    InvalidateRecursively(store_[ticket].dependents());
  }
}

AbstractValue* Cache::Set(CacheTicket ticket,
                          std::unique_ptr<AbstractValue> value) {
  DRAKE_ABORT_UNLESS(ticket < static_cast<int>(store_.size()));
  store_[ticket].set_is_valid(true);
  store_[ticket].set_value(std::move(value));
  return store_[ticket].value();
}

AbstractValue* Cache::Get(CacheTicket ticket) const {
  DRAKE_ABORT_UNLESS(ticket < static_cast<int>(store_.size()));
  if (store_[ticket].is_valid()) {
    return store_[ticket].value();
  } else {
    return nullptr;
  }
}

std::unique_ptr<AbstractValue> Cache::Swap(
    CacheTicket ticket, std::unique_ptr<AbstractValue> value) {
  std::unique_ptr<AbstractValue> old_value = store_[ticket].release_value();
  Set(ticket, std::move(value));
  return old_value;
}

std::unique_ptr<Cache> Cache::Clone() const {
  auto clone = std::make_unique<Cache>();
  clone->store_ = store_;
  return clone;
}

}  // namespace systems
}  // namespace drake
