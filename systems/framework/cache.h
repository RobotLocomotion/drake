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

//==============================================================================
//                             CACHE ENTRY VALUE
//==============================================================================
/** This is the representation in the Context for the value of one of a System's
CacheEntry objects. It consists of a single type-erased value, a serial number,
an "is-up-to-date" flag, and a DependencyTracker ticket. Details:
- "Up to date" means that the stored value is up to date with its prerequisites;
  that is, if you were to recompute it using the current Context values you
  would get the identical value (so don't bother!).
- The "serial number" is incremented whenever the value is modified, or made
  available for mutable access. You can use it to recognize that you are looking
  at the same value as you saw at some earlier time. It is also useful for
  performance studies since it is a count of how many times this value was
  recomputed. Note that marking the value "out of date" is _not_ a modification;
  that does not change the serial number.
- The associated DependencyTracker maintains lists of all upstream prerequisites
  and downstream dependents of the value stored here, and also has a pointer to
  this %CacheEntryValue that it uses for invalidation. Upstream modifications
  cause the DependencyTracker to clear the up-to-date flag here, and mark all
  downstream dependents out of date also. The ticket is always interpreted
  within the same subcontext that owns the Cache that owns this
  %CacheEntryValue.

For debugging purposes, caching may be disabled for an entire Context or for
particular cache entry values. This is independent of the up-to-date flag
described above, which is still expected to be operational when caching is
disabled. However, when caching is disabled the Eval() methods will recompute
the contained value even if it is marked up-to-date. That should have no effect
other than to slow down computation; if it does, something is wrong. There
could be a problem with the dependencies, or a bug in the caching system, or
something more subtle in user code. */
class CacheEntryValue {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CacheEntryValue)

  /** Create a new cache value with the given human-readable description and
  (optionally) an abstract value with the right concrete type for this value.
  The given cache index and dependency ticket are recorded here. Unless you
  have a good reason to do otherwise, make the description identical to the
  CacheEntry for which this is the value. */
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

  /** Destructs the cache value, but does not issue any notifications to
  downstream dependents. */
  ~CacheEntryValue() = default;

  /** Defines the concrete value type by providing an initial AbstractValue
  object containing an object of the appropriate concrete type. This value
  is _not_ marked up-to-date. It is an error to call this if there is already
  a value here; use has_value() if you want to check first. The serial number
  is set to 1. No out-of-date notifications are sent to downstream
  dependents. */
  void SetInitialValue(std::unique_ptr<AbstractValue> init_value) {
    ThrowIfHasValue(__func__);
    value_ = std::move(init_value);
    serial_number_ = 1;
    set_is_up_to_date(false);
  }

  /** Provides fast, const access to the contained abstract value, which must
  already be up to date with respect to all of its prerequisites. It is an error
  to call this if there is no stored value, or it is out of date. Because this
  is often used in performance-critical contexts, these requirements will be
  checked only in Debug builds. If you are not in a performance-critical
  situation, use GetAbstractValueOrThrow() instead. */
  const AbstractValue& get_abstract_value() const {
  #ifdef DRAKE_ASSERT_IS_ARMED
    return GetAbstractValueOrThrowHelper(__func__);
  #else
    return *value_;
  #endif
  }

  /** Returns a const reference to this %CacheEntryValue's abstract value. It is
  an error to call this if there is no stored value object, or if the value is
  not up to date. */
  const AbstractValue& GetAbstractValueOrThrow() const {
    return GetAbstractValueOrThrowHelper(__func__);
  }

  /** Provides fast access to the contained value. It is an error to call
  this if there is no stored value, or it is out of date, or it doesn't actually
  have type V. Because this is often used in performance-critical contexts,
  these requirements will be checked only in Debug builds. If you are not in
  a performance-critical situation, use GetValueOrThrow() instead.
  @tparam V The known actual value type. */
  template <typename V>
  const V& get_value() const {
    #ifdef DRAKE_ASSERT_IS_ARMED
      return GetValueOrThrowHelper<V>(__func__);
    #else
      return value_->GetValue<V>();
    #endif
  }

  /** Same as get_value() but validates all preconditions even in Release
  builds. */
  template <typename V>
  const V& GetValueOrThrow() const {
    return GetValueOrThrowHelper<V>(__func__);
  }

  /** This is the normal method for assigning a new value to a cache value.
  The cache value must already have a value object of type V to which the
  new value is assigned, and that value must not already be up to date.
  The new value is assumed to be up to date with its prerequisites, so the
  up-to-date flag is set. No out-of-date notifications are issued by this
  method; we assume downstream dependents were marked out of date at the time
  this value went out of date. The serial number is incremented. If you are not
  in a performance-critical situation, use SetValueOrThrow() instead.
  @tparam V The known actual value type. */
  template <typename V>
  void set_value(const V& new_value) {
    #ifdef DRAKE_ASSERT_IS_ARMED
      SetValueOrThrowHelper<V>(__func__, new_value);
    #else
      value_->SetValue<V>(new_value);
    #endif
    ++serial_number_;
    set_is_up_to_date(true);
  }

  /** Same as set_value() but validates all preconditions even in Release
  builds. */
  template <typename V>
  void SetValueOrThrow(const V& new_value) {
    SetValueOrThrowHelper<V>(__func__, new_value);
    ++serial_number_;
    set_is_up_to_date(true);
  }

  /** Return the human-readable description for this %CacheEntryValue. */
  const std::string& description() const { return description_; }

  /** Returns `true` if this %CacheEntryValue currently contains a value object
  at all, regardless of the state of its value. There will be no value object
  after default construction, prior to SetInitialValue(). */
  bool has_value() const { return value_ != nullptr; }

  /** Returns `true` if the current value is up to date with all of its
  prerequisites. It is an error to call this if there is no stored value
  object. This refers only to the up-to-date flag and is independent of whether
  caching is enabled or disabled.
  @see needs_recomputation() */
  bool is_up_to_date() const {
    DRAKE_ASSERT_VOID(ThrowIfNoValue(__func__));
    return (flags_ & kValueIsOutOfDate) == 0;
  }

  /** (Advanced) Returns `true` if caching is disabled for this cache entry.
  This is independent of the up-to-date flag. */
  bool is_entry_disabled() const {
    return (flags_ & kCacheEntryIsDisabled) != 0;
  }

  /** Returns `true` if either (a) the value is not up to date, or (b) caching
  is disabled for this entry. This is a _very_ fast inline method intended
  to be called every time a cache value is obtained with Eval(). This is
  equivalent to `!is_up_to_date() || is_entry_disabled()` but much faster. */
  bool needs_recomputation() const {
    DRAKE_ASSERT_VOID(ThrowIfNoValue(__func__));
    return flags_ != 0;
  }

  /** Returns the serial number of the contained value. This counts up every
  time the contained value changes, or when mutable access is granted. */
  int64_t serial_number() const { return serial_number_; }

  /** Returns the CacheIndex used to locate this %CacheEntryValue within the
  containing Context. */
  CacheIndex cache_index() const {return cache_index_;}

  /** Returns the DependencyTicket used to locate the DependencyTracker that
  manages dependencies for this %CacheEntryValue. */
  DependencyTicket ticket() const {return ticket_;}

  /** (Advanced) Provides direct mutable access to the contained value, for the
  purpose of performing an update or extended computation in place. This is only
  permitted if the value is already marked out of date (meaning that all
  downstream dependents have already been notified). It is an error to call
  this if there is no stored value, or it is already up to date. Since this is
  intended for relatively expensive computations, those preconditions are
  checked even in Release builds. If you have a small, fast computation to
  perform, use set_value() instead. If your computation completes successfully,
  you must set the up-to-date flag yourself if you want anyone to be able to
  use the new value. The serial number is incremented. */
  AbstractValue& GetMutableAbstractValueOrThrow() {
    return GetMutableAbstractValueOrThrowHelper(__func__);
  }

  /** (Advanced) Convenience method that provides mutable access to the
  contained value downcast to its known concrete type. Throws an exception if
  the contained value does not have the indicated concrete type.
  @see GetMutableAbstractValueOrThrow() for more information.
  @tparam V The known actual value type. */
  template <typename V>
  const V& GetMutableValueOrThrow() {
    AbstractValue& value = GetMutableAbstractValueOrThrowHelper(__func__);
    return value.GetMutableValueOrThrow<V>();
  }

  /** (Advanced) Returns a reference to the contained value without checking
  whether it is valid. This can be used to check type and size information
  but should not be used to look at the value unless you _really_ know what
  you're doing. Fails only if there is no contained value. */
  const AbstractValue& GetUncheckedAbstractValueOrThrow() const {
    ThrowIfNoValue(__func__);
    return *value_;
  }

  /** (Advanced) Convenience method that provides unchecked access to the
  contained value downcast to its known concrete type. Throws an exception if
  there is no contained value, or if the contained value does not have the
  indicated concrete type.
  @see GetUncheckedAbstractValueOrThrow() for more information.
  @tparam V The known actual value type. */
  template <typename V>
  const V& GetUncheckedValueOrThrow() const {
    ThrowIfNoValue(__func__);
    return value_->GetMutableValueOrThrow<V>();
  }

  /** (Advanced) Sets the up-to-date flag. Regardless of the flag setting, this
  has no other effects. In particular, it does not modify the value, does not
  change the serial number, and does not notify downstream dependents. You
  should not call this method, with either argument, unless you really know what
  you're doing, or have a death wish. It is an error to call this with
  `up_to_date==true` if there is no stored value object. */
  void set_is_up_to_date(bool up_to_date) {
    DRAKE_ASSERT_VOID(ThrowIfValidatingNoValue(__func__, up_to_date));
    set_or_clear_flags_bit(kValueIsOutOfDate, !up_to_date);
  }

  /** (Advanced) Disable or enable caching for just this cache entry value.
  If this flag is set, Eval() will unconditionally invoke Calc() to recompute
  the value, regardless of the setting of the up-to-date flag. The `disabled`
  flag is independent of the up-to-date flag, which will continue to be managed
  even if caching is disabled. */
  void set_is_entry_disabled(bool disabled) {
    set_or_clear_flags_bit(kCacheEntryIsDisabled, disabled);
  }

  /** (Advanced) Swaps ownership of the stored value object with the given
  one. The value is marked out-of-date and the serial number is incremented.
  This is useful for discrete updates of abstract state variables that contain
  large objects. Both values must be non-null and of the same concrete type. */
  void swap_value(std::unique_ptr<AbstractValue>* other_value) {
    DRAKE_ASSERT_VOID(ThrowIfNoValue(__func__));
    DRAKE_ASSERT_VOID(ThrowIfBadOtherValue(__func__, other_value));
    value_.swap(*other_value);
    ++serial_number_;
    set_is_up_to_date(false);
  }

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
  // Fully-checked method with API name to use in error messages.
  const AbstractValue& GetAbstractValueOrThrowHelper(const char* api) const {
    ThrowIfNoValue(api);
    ThrowIfOutOfDate(api);  // Must *not* be out of date!
    return *value_;
  }

  // Note that serial number is incremented here since caller will be stomping
  // on this value.
  AbstractValue& GetMutableAbstractValueOrThrowHelper(const char* api) {
    ThrowIfNoValue(api);
    ThrowIfAlreadyComputed(api);  // *Must* be out of date!
    ++serial_number_;
    return *value_;
  }

  // Adds a check on the concrete value type also.
  template <typename T>
  const T& GetValueOrThrowHelper(const char* api) const {
    return GetAbstractValueOrThrowHelper(api).GetValueOrThrow<T>();
  }

  // Fully-checked method with API name to use in error messages.
  template <typename T>
  void SetValueOrThrowHelper(const char* api, const T& new_value) const {
    ThrowIfNoValue(api);
    ThrowIfAlreadyComputed(api);
    return value_->SetValueOrThrow<T>(new_value);
  }

  std::string FormatName(const char* api) const {
    return "CacheEntryValue(" + description_ + ")::" + api + "(): ";
  }

  void ThrowIfNoValue(const char* api) const {
    if (!has_value())
      throw std::logic_error(FormatName(api) + "no value is present.");
  }

  void ThrowIfValidatingNoValue(const char* api, bool up_to_date) const {
    if (up_to_date && !has_value())
      throw std::logic_error(FormatName(api) +
                             "tried to validate but no value is present.");
  }

  void ThrowIfHasValue(const char* api) {
    if (has_value()) {
      throw std::logic_error(FormatName(api) +
          "there is already a value object in this CacheEntryValue.");
    }
  }

  void ThrowIfBadOtherValue(const char* api,
                            std::unique_ptr<AbstractValue>* other_value_ptr) {
    DRAKE_DEMAND(other_value_ptr);
    auto& other_value = *other_value_ptr;
    if (other_value == nullptr)
      throw std::logic_error(FormatName(api) + "other value is empty.");

    // Extract these outside typeid() to avoid warnings.
    const AbstractValue& abs_value = *value_;
    const AbstractValue& other_abs_value = *other_value;
    if (std::type_index(typeid(abs_value)) !=
        std::type_index(typeid(other_abs_value))) {
      throw std::logic_error(FormatName(api) +
                             "other value has wrong concrete type " +
                             NiceTypeName::Get(*other_value) + ". Expected " +
                             NiceTypeName::Get(*value_) + ".");
    }
  }

  // This means literally that the out-of-date bit is set; it does not look
  // at whether caching is disabled.
  void ThrowIfOutOfDate(const char* api) const {
    if (!is_up_to_date())
      throw std::logic_error(FormatName(api) +
                             "the current value is out of date.");
  }

  // This checks that there is *some* reason to recompute -- either out-of-date
  // or caching is disabled.
  void ThrowIfAlreadyComputed(const char* api) const {
    if (!needs_recomputation())
      throw std::logic_error(FormatName(api) +
                             "the current value is already up to date.");
  }

  // The sense of these flag bits is chosen so that Eval() can check
  // in a single instruction whether it must recalculate. Only if flags==0 can
  // we reuse the existing value. See needs_recomputation() above.
  enum Flags : int {
    kValueIsOutOfDate     = 0b01,
    kCacheEntryIsDisabled = 0b10
  };

  void set_or_clear_flags_bit(Flags bit, bool should_set) {
    if (should_set)
      flags_ |= bit;
    else
      flags_ &= ~bit;
  }

  // A human-readable description of this cache entry. Not interpreted by code
  // but useful for error messages.
  std::string description_;

  // Records the ticket number for this CacheEntryValue within its containing
  // Context.
  CacheIndex cache_index_;

  // The value and its serial number.
  copyable_unique_ptr<AbstractValue> value_;
  int64_t serial_number_{0};

  // Value bookkeeping.
  int flags_{kValueIsOutOfDate};
  DependencyTicket ticket_;
};

//==============================================================================
//                                  CACHE
//==============================================================================
/** Stores all the CacheEntryValue objects owned by a particular LeafContext,
organized to allow fast access using a CacheIndex as an index. Memory addresses
of CacheEntryValue objects are stable once allocated, but CacheIndex numbers are
stable even after a Context has been copied so should be preferred. A %Cache is
copyable, assignable, and movable, but not thread-safe. */
class Cache {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Cache)

  /** Default construction creates an empty cache. */
  Cache() = default;

  /** Destruction deletes all cache entries and their contained values; no
  dependency notifications are issued. */
  ~Cache() = default;

  /** Allocates a new CacheEntryValue and corresponding DependencyTracker using
  information provided in the given CacheEntry, which includes the required
  CacheIndex and DependencyTicket number. We cannot yet allocate a value for
  this cache entry since that requires invoking the cache entry's Allocate()
  method on a completed Context. Note that a CacheEntryValue
  memory address is stable after allocation in a particular Context while the
  CacheIndex remains stable even after copying the Context. */
  CacheEntryValue& CreateNewCacheEntryValue(
      CacheIndex index, DependencyTicket ticket,
      const std::string& description,
      const std::vector<DependencyTicket>& prerequisites,
      DependencyGraph* graph);

  /** Returns the number of CacheEntryValue objects currently stored in this
  cache. */
  int num_entries() const { return static_cast<int>(store_.size()); }

  /** Returns a const CacheEntryValue given an index. This is very fast.
  Behavior is undefined if the ticket is out of range [0..num_entries()-1]. */
  const CacheEntryValue& get_cache_entry_value(CacheIndex index) const {
    DRAKE_ASSERT(index.is_valid());
    DRAKE_ASSERT(0 <= index && index < num_entries());
    CacheEntryValue& cache_value = *store_[index];
    DRAKE_ASSERT(cache_value.cache_index() == index);
    return cache_value;
  }

  /** Returns a mutable CacheEntryValue given an index. This is very fast.
  Behavior is undefined if the index is out of range [0..num_entries()-1]. */
  CacheEntryValue& get_mutable_cache_entry_value(CacheIndex index) {
    return const_cast<CacheEntryValue&>(get_cache_entry_value(index));
  }

  /** (Advanced) Enable or disable caching for all the entries in this %Cache.
  Note that this is done by setting or clearing individual `is_disabled` flags
  in the entries, so it can be changed on a per-entry basis later. This has no
  effect on the "up to date" flags. */
  void SetIsCacheDisabled(bool disabled) {
    for (auto& entry : store_)
      entry->set_is_entry_disabled(disabled);
  }

  /** (Advanced) Mark every entry in this cache as "out of date". This forces
  the next Eval() request for an entry to perform a recalculation. After that
  normal caching behavior resumes. */
  void SetAllEntriesOutOfDate() {
    for (auto& entry : store_)
      entry->set_is_up_to_date(false);
  }

 private:
  // All CacheEntryValue objects, indexed by CacheIndex.
  std::vector<copyable_unique_ptr<CacheEntryValue>> store_;
};

}  // namespace systems
}  // namespace drake
