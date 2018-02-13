#pragma once

/** @file
Declares CacheEntryValue and Cache, which is the container for cache entry
values. */

// TODO(sherm1) Re-review this file in its entirety when the cache stubs are
// replaced with real code in a subsequent PR.

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/never_destroyed.h"
#include "drake/systems/framework/framework_common.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

class DependencyGraph;

// TODO(sherm1) Stubbed for testing DependencyTracker; do not review.

// These are stubs for the two classes that comprise the cache:
// - CacheEntryValue representing a single cache entry and storing its
//                   abtract value and "up-to-date" indicator.
// - Cache the container for CacheEntryValues
//
// Note that a Cache is local to a particular Context; that is, when there is
// a diagram, each Context within it has its own Cache object.
#ifndef DRAKE_DOXYGEN_CXX  // Hide from Doxygen for now.
class CacheEntryValue {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CacheEntryValue)

  CacheEntryValue(CacheIndex index, DependencyTicket ticket,
                  std::string description,
                  std::unique_ptr<AbstractValue> initial_value)
      : description_(std::move(description)),
        cache_index_(index),
        value_(std::move(initial_value)),
        ticket_(ticket) {
    DRAKE_DEMAND(index.is_valid() && ticket.is_valid());
    // OK if value is null here.
  }

  void SetInitialValue(std::unique_ptr<AbstractValue> init_value) {
    value_ = std::move(init_value);
    set_is_up_to_date(false);
  }

  template <typename V>
  void set_value(const V& new_value) {
    value_->SetValue<V>(new_value);
    set_is_up_to_date(true);
  }

  bool is_up_to_date() const { return is_up_to_date_flag_; }

  CacheIndex cache_index() const { return cache_index_; }

  DependencyTicket ticket() const { return ticket_; }

  void set_is_up_to_date(bool up_to_date) { is_up_to_date_flag_ = up_to_date; }

  /** Returns a mutable reference to an unused cache entry value object, which
  has no valid CacheIndex or DependencyTicket and has a meaningless value. The
  reference is to a singleton %CacheEntryValue and will always return the same
  address. You may invoke set_is_up_to_date() harmlessly on this object, but may
  not depend on its contents in any way as they may change unexpectedly. The
  intention is that this object is used as a common throw-away destination for
  non-cache DependencyTracker invalidations so that invalidation can be done
  unconditionally, and to the same memory location, for speed. */
  static CacheEntryValue& dummy() {
    static never_destroyed<CacheEntryValue> dummy;
    return dummy.access();
  }

 private:
  // Allow never_destroyed to invoke the private constructor on our behalf.
  friend class never_destroyed<CacheEntryValue>;

  // Default constructor can only be used privately to construct an empty
  // CacheEntryValue with description "DUMMY" and a meaningless value.
  CacheEntryValue()
      : description_("DUMMY"), value_(AbstractValue::Make<int>(0)) {}

  std::string description_;
  CacheIndex cache_index_;
  copyable_unique_ptr<AbstractValue> value_;
  bool is_up_to_date_flag_{false};
  DependencyTicket ticket_;
};

// TODO(sherm1) Stubbed for DependencyTracker/Graph review; don't review.
class Cache {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Cache)

  Cache() = default;

  // Allocates a new CacheEntryValue and corresponding DependencyTracker using
  // the given CacheIndex and DependencyTicket number. The CacheEntryValue
  // object is owned by this Cache and the returned reference remains valid
  // if other cache entry values are created. The created DependencyTracker
  // object is owned by the given DependencyGraph, which must be owned by
  // the same Context that owns this Cache. The graph must already contain
  // trackers for the indicated prerequisites. The new tracker will retain a
  // pointer to the created CacheEntryValue for invalidation purposes.
  CacheEntryValue& CreateNewCacheEntryValue(
      CacheIndex index, DependencyTicket ticket,
      const std::string& description,
      const std::vector<DependencyTicket>& prerequisites,
      DependencyGraph* graph);

  int num_entries() const { return static_cast<int>(store_.size()); }

  CacheEntryValue& get_mutable_cache_entry_value(CacheIndex index) {
    CacheEntryValue& cache_value = *store_[index];
    return cache_value;
  }

 private:
  std::vector<copyable_unique_ptr<CacheEntryValue>> store_;
};
#endif  // Hiding from Doxygen.

}  // namespace systems
}  // namespace drake
