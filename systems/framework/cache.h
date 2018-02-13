#pragma once

/** @file
Declares CacheEntryValue and Cache, which is the container for cache entry
values. */

#include <cstddef>
#include <memory>
#include <string>
#include <typeindex>
#include <typeinfo>
#include <utility>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/type_safe_index.h"
#include "drake/common/unused.h"
#include "drake/systems/framework/framework_common.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

class DependencyGraph;
class CacheEntry;

// TODO(sherm1) Stubbed for testing DependencyTracker; do not review.
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

#ifndef DRAKE_DOXYGEN_CXX
  // (Internal use only) Constructs an empty CacheEntryValue with description
  // "DUMMY" and a meaningless value. Used only as a default destination for
  // non-cache DependencyTracker invalidations.
  explicit CacheEntryValue(bool not_used)
      : description_("DUMMY"), value_(AbstractValue::Make<int>(0)) {
    unused(not_used);
  }
#endif

 private:
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

}  // namespace systems
}  // namespace drake
