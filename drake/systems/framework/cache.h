#pragma once

#include <cstddef>
#include <map>
#include <memory>
#include <set>
#include <tuple>
#include <vector>

#include "drake/drakeSystemFramework_export.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

typedef int CacheTicket;

namespace internal {

/// A single cached piece of data, its validity bit, and the set of other cache
/// entries that depend on it.
class CacheEntry {
 public:
  CacheEntry() {}
  virtual ~CacheEntry() {}

  // CacheEntry is copyable.
  explicit CacheEntry(const CacheEntry& other);
  CacheEntry& operator=(const CacheEntry& other);

  bool is_valid() const { return is_valid_; }
  void set_is_valid(bool valid) { is_valid_ = valid; }

  AbstractValue* value() const { return value_.get(); }
  void set_value(std::unique_ptr<AbstractValue> value) {
    value_ = std::move(value);
  }
  std::unique_ptr<AbstractValue> release_value() {
    set_is_valid(false);
    return std::move(value_);
  }

  const std::set<CacheTicket>& dependents() const { return dependents_; }
  void add_dependent(CacheTicket ticket) { dependents_.insert(ticket); }

 private:
  bool is_valid_{false};
  std::unique_ptr<AbstractValue> value_;

  // The set of cache tickets that are invalidated when the ticket corresponding
  // to this entry is invalidated. This graph is directed and acyclic.
  std::set<CacheTicket> dependents_;
};

}  // namespace internal

/// A key-value cache that can be invalidated based on changes to fields in the
/// Context.
///
/// The cache is not thread-safe.
class DRAKESYSTEMFRAMEWORK_EXPORT Cache {
 public:
  Cache();
  virtual ~Cache();

  /// Creates a new cache ticket, which will be invalidated whenever any of
  /// the dependencies are invalidated.
  CacheTicket MakeCacheTicket(std::set<CacheTicket> prerequisites);

  /// Invalidates and deletes the value for @p ticket, and all lines that
  /// depend on it.
  void Invalidate(CacheTicket ticket);

  /// Takes ownership of a cached item, and returns a bare pointer to the item.
  AbstractValue* Set(CacheTicket ticket, std::unique_ptr<AbstractValue> value);

  /// Returns the cached item for the given ticket, or nullptr if the item
  /// has been invalidated.
  AbstractValue* Get(CacheTicket ticket) const;

  /// Returns the cached item for the given ticket, or nullptr if the item
  /// has been invalidated. Replaces it with @p value.
  std::unique_ptr<AbstractValue> Swap(CacheTicket ticket,
                                      std::unique_ptr<AbstractValue> value);

  std::unique_ptr<Cache> Clone() const;

 private:
  // Invalidates all tickets that depend on the tickets in @p to_invalidate.
  void InvalidateRecursively(const std::set<CacheTicket>& to_invalidate);

  // For each cache ticket, the stored value, and its validity bit.
  std::vector<internal::CacheEntry> store_;
};

}  // namespace systems
}  // namespace drake
