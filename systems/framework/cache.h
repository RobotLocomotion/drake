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
#include "drake/common/never_destroyed.h"
#include "drake/common/reset_on_copy.h"
#include "drake/common/text_logging.h"
#include "drake/systems/framework/framework_common.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

class DependencyGraph;

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
  /** @name  Does not allow move or assignment; copy constructor limited.
  The copy constructor does not copy internal pointers so requires special
  handling. */
  /** @{ */
  CacheEntryValue(CacheEntryValue&&) = delete;
  CacheEntryValue& operator=(const CacheEntryValue&) = delete;
  CacheEntryValue& operator=(CacheEntryValue&&) = delete;
  /** @} */

  /** Destructs the cache value, but does not issue any notifications to
  downstream dependents. */
  ~CacheEntryValue() = default;

  /** Defines the concrete value type by providing an initial AbstractValue
  object containing an object of the appropriate concrete type. This value
  is _not_ marked up-to-date. It is an error to call this if there is already
  a value here; use has_value() if you want to check first. The serial number
  is set to 1. No out-of-date notifications are sent to downstream
  dependents.
  @throws std::logic_error if there is already a value. */
  void SetInitialValue(std::unique_ptr<AbstractValue> init_value) {
    ThrowIfHasValue(__func__);
    value_ = std::move(init_value);
    serial_number_ = 1;
    mark_out_of_date();
  }

  /** Provides fast, const access to the contained abstract value, which must
  already be up to date with respect to all of its prerequisites. It is an error
  to call this if there is no stored value, or it is out of date. Because this
  is often used in performance-critical contexts, these requirements will be
  checked only in Debug builds. If you are not in a performance-critical
  situation (and you probably are not!), use GetAbstractValueOrThrow()
  instead. */
  const AbstractValue& get_abstract_value() const {
  #ifdef DRAKE_ASSERT_IS_ARMED
    return GetAbstractValueOrThrowHelper(__func__);
  #else
    return *value_;
  #endif
  }

  /** Returns a const reference to this %CacheEntryValue's abstract value. It is
  an error to call this if there is no stored value object, or if the value is
  not up to date.
  @throws std::logic_error if there is no value or it is out of date. */
  const AbstractValue& GetAbstractValueOrThrow() const {
    return GetAbstractValueOrThrowHelper(__func__);
  }

  /** Provides fast, const access to the contained value of known type V. It is
  an error to call this if there is no stored value, or it is out of date, or it
  doesn't actually have type V. Because this is expected to be used in
  performance-critical, inner-loop circumstances, these requirements will be
  checked only in Debug builds. If you are not in a performance-critical
  situation (and you probably are not!), use `GetValueOrThrow<V>`() instead.
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
  builds.
  @throws std::logic_error if there is no stored value, or if it is out of
                           date, or it doesn't actually have type V. */
  template <typename V>
  const V& GetValueOrThrow() const {
    return GetValueOrThrowHelper<V>(__func__);
  }

  /** This is the normal method for assigning a new value to a cache entry.
  The cache value must already have a value object of type V to which the
  new value is assigned, and that value must not already be up to date.
  The new value is assumed to be up to date with its prerequisites, so the
  up-to-date flag is set. No out-of-date notifications are issued by this
  method; we assume downstream dependents were marked out of date at the time
  this value went out of date. The serial number is incremented. If you are not
  in a performance-critical situation (and you probably are not!), use
  `SetValueOrThrow<V>()` instead.
  @tparam V The known actual value type. */
  template <typename V>
  void set_value(const V& new_value) {
    #ifdef DRAKE_ASSERT_IS_ARMED
      SetValueOrThrowHelper<V>(__func__, new_value);
    #else
      value_->SetValue<V>(new_value);
    #endif
    ++serial_number_;
    mark_up_to_date();
  }

  /** Same as set_value() but validates all preconditions even in Release
  builds.
  @throws std::logic_error if there is no value, or the value is already up
                           to date, of it doesn't actually have type V. */
  template <typename V>
  void SetValueOrThrow(const V& new_value) {
    SetValueOrThrowHelper<V>(__func__, new_value);
    ++serial_number_;
    mark_up_to_date();
  }

  /** Returns the human-readable description for this %CacheEntryValue. */
  const std::string& description() const { return description_; }

  /** Returns the description, preceded by the full pathname of the subsystem
  associated with the owning subcontext. */
  std::string GetPathDescription() const;

  /** Returns `true` if this %CacheEntryValue currently contains a value object
  at all, regardless of whether it is up to date. There will be no value object
  after default construction, prior to SetInitialValue(). */
  bool has_value() const { return value_ != nullptr; }

  /** Returns `true` if the current value is up to date with respect to all its
  prerequisites. This refers only to the up-to-date flag and is independent of
  whether caching is enabled or disabled. Don't call this if there is no value
  here; use has_value() if you aren't sure.
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
  equivalent to `!is_up_to_date() || is_entry_disabled()` but faster.  Don't
  call this if there is no value here; use has_value() if you aren't sure.*/
  bool needs_recomputation() const {
    DRAKE_ASSERT_VOID(ThrowIfNoValue(__func__));
    return flags_ != 0;
  }

  /** Returns the serial number of the contained value. This counts up every
  time the contained value changes, or whenever mutable access is granted. */
  int64_t serial_number() const { return serial_number_; }

  /** Returns the CacheIndex used to locate this %CacheEntryValue within its
  containing subcontext. */
  CacheIndex cache_index() const {return cache_index_;}

  /** Returns the DependencyTicket used to locate the DependencyTracker that
  manages dependencies for this %CacheEntryValue. The ticket refers to a
  tracker that is owned by the same subcontext that owns this
  %CacheEntryValue. */
  DependencyTicket ticket() const {return ticket_;}

  /** (Advanced) Provides direct mutable access to the contained value, for the
  purpose of performing an update or extended computation in place. This is only
  permitted if the value is already marked out of date (meaning that all
  downstream dependents have already been notified). It is an error to call
  this if there is no stored value, or it is already up to date. Since this is
  intended for relatively expensive computations, these preconditions are
  checked even in Release builds. If you have a small, fast computation to
  perform, use set_value() instead. If your computation completes successfully,
  you must mark the entry up to date yourself if you want anyone to be able to
  use the new value. The serial number is incremented.
  @throws std::logic_error if there is no value, or if the value is already
                           up to date.
  @see set_value(), mark_up_to_date() */
  AbstractValue& GetMutableAbstractValueOrThrow() {
    return GetMutableAbstractValueOrThrowHelper(__func__);
  }

  /** (Advanced) Convenience method that provides mutable access to the
  contained value downcast to its known concrete type. Throws an exception if
  the contained value does not have the indicated concrete type.
  @throws std::logic_error if there is no value, or if the value is already
                           up to date, of it doesn't actually have type V.
  @see GetMutableAbstractValueOrThrow() for more information.
  @tparam V The known actual value type. */
  template <typename V>
  V& GetMutableValueOrThrow() {
    AbstractValue& value = GetMutableAbstractValueOrThrowHelper(__func__);
    return value.GetMutableValueOrThrow<V>();
  }

  /** (Advanced) Returns a reference to the contained value _without_ checking
  whether the value is up to date. This can be used to check type and size
  information but should not be used to look at the value unless you _really_
  know what you're doing.
  @throws std::logic_error if there is no contained value. */
  const AbstractValue& PeekAbstractValueOrThrow() const {
    ThrowIfNoValue(__func__);
    return *value_;
  }

  /** (Advanced) Convenience method that provides access to the contained value
  downcast to its known concrete type, _without_ checking whether the value is
  up to date.
  @throws std::logic_error if there is no contained value, or if the contained
                           value does not actually have type V.
  @see PeekAbstractValueOrThrow() for more information.
  @tparam V The known actual value type. */
  template <typename V>
  const V& PeekValueOrThrow() const {
    ThrowIfNoValue(__func__);
    return value_->GetMutableValueOrThrow<V>();
  }

  /** (Advanced) Marks the cache entry value as _up-to-date_ with respect to
  its prerequisites, with no other effects. In particular, this method does not
  modify the value, does not change the serial number, and does not notify
  downstream dependents of anything. This is a very dangerous method since it
  enables access to the value but can't independently determine whether it is
  really up to date. You should not call it unless you really know what you're
  doing, or have a death wish. Do not call this method if there is no stored
  value object; use has_value() if you aren't sure. This is intended
  to be very fast so doesn't check for a value object except in Debug builds. */
  void mark_up_to_date() {
    DRAKE_ASSERT_VOID(ThrowIfNoValue(__func__));
    flags_ &= ~kValueIsOutOfDate;
  }

  /** (Advanced) Marks the cache entry value as _out-of-date_ with respect to
  its prerequisites, with no other effects. In particular, it does not modify
  the value, does not change the serial number, and does not notify downstream
  dependents. You should not call this method unless you know that dependent
  notification has already been taken care of. There are no error conditions;
  even an empty cache entry can be marked out of date. */
  void mark_out_of_date() {
    flags_ |= kValueIsOutOfDate;
  }

  /** (Advanced) Disable caching for just this cache entry value. When disabled,
  the corresponding entry's Eval() method will unconditionally invoke Calc() to
  recompute the value, regardless of the setting of the `up-to-date` flag. The
  `disabled` flag is independent of the `up-to-date` flag, which will continue
  to be managed even if caching is disabled. */
  void disable_caching() {
    flags_ |= kCacheEntryIsDisabled;
  }

  /** (Advanced) Enable caching for this cache entry value if it was previously
  disabled. When enabled (the default condition) the corresponding entry's
  Eval() method will check the `up-to-date` flag and invoke Calc() only if the
  entry is marked out of date. */
  void enable_caching() {
    flags_ &= ~kCacheEntryIsDisabled;
  }

  /** (Advanced) Swaps ownership of the stored value object with the given
  one. The value is marked out-of-date and the serial number is incremented.
  This is useful for discrete updates of abstract state variables that contain
  large objects. Both values must be non-null and of the same concrete type but
  we won't check for errors except in Debug builds. */
  void swap_value(std::unique_ptr<AbstractValue>* other_value) {
    DRAKE_ASSERT_VOID(ThrowIfNoValue(__func__));
    DRAKE_ASSERT_VOID(ThrowIfBadOtherValue(__func__, other_value));
    value_.swap(*other_value);
    ++serial_number_;
    mark_out_of_date();
  }

  /** Returns a mutable reference to an unused cache entry value object, which
  has no valid CacheIndex or DependencyTicket and has a meaningless value. The
  reference is to a singleton %CacheEntryValue and will always return the same
  address. You may invoke mark_up_to_date() harmlessly on this object, but may
  not depend on its contents in any way as they may change unexpectedly. The
  intention is that this object is used as a common throw-away destination for
  non-cache DependencyTracker invalidations so that invalidation can be done
  unconditionally, and to the same memory location, for speed. */
  static CacheEntryValue& dummy() {
    static never_destroyed<CacheEntryValue> dummy;
    return dummy.access();
  }

  #ifndef DRAKE_DOXYGEN_CXX
  // (Internal use only) Requires post-copy cleanup via set_owning_subcontext().
  // Has to be public so copyable_unique_ptr can access it; making it a friend
  // is not enough.
  CacheEntryValue(const CacheEntryValue&) = default;
  #endif

 private:
  // So Cache and no one else can construct and copy CacheEntryValues.
  friend class Cache;

  // Allow never_destroyed to invoke the private constructor on our behalf.
  friend class never_destroyed<CacheEntryValue>;

  // Default constructor can only be used privately to construct an empty
  // CacheEntryValue with description "DUMMY" and a meaningless value.
  CacheEntryValue()
      : description_("DUMMY"), value_(AbstractValue::Make<int>(0)) {}

  // Create a new cache value with the given human-readable description and
  // (optionally) an abstract value that defines the right concrete type for
  // this value. The given cache index and dependency ticket must be valid and
  // are recorded here. Unless you have a good reason to do otherwise, make the
  // description identical to the CacheEntry for which this is the value.
  CacheEntryValue(CacheIndex index, DependencyTicket ticket,
                  std::string description,
                  const internal::SystemPathnameInterface* owning_subcontext,
                  std::unique_ptr<AbstractValue> initial_value)
      : cache_index_(index),
        ticket_(ticket),
        description_(std::move(description)),
        owning_subcontext_(owning_subcontext),
        value_(std::move(initial_value)) {
    DRAKE_DEMAND(index.is_valid() && ticket.is_valid());
    DRAKE_DEMAND(owning_subcontext != nullptr);
    // OK if initial_value is null here.
  }

  // This is the post-copy cleanup method.
  void set_owning_subcontext(
      const internal::SystemPathnameInterface* owning_subcontext) {
    DRAKE_DEMAND(owning_subcontext != nullptr);
    DRAKE_DEMAND(owning_subcontext_ == nullptr);
    owning_subcontext_ = owning_subcontext;
  }

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


  void ThrowIfNoValue(const char* api) const {
    if (!has_value())
      throw std::logic_error(FormatName(api) + "no value is present.");
  }

  void ThrowIfHasValue(const char* api) {
    if (has_value()) {
      throw std::logic_error(FormatName(api) +
          "there is already a value object in this CacheEntryValue.");
    }
  }

  // Throws if "other" doesn't have the same concrete type as this value.
  void ThrowIfBadOtherValue(const char* api,
                            std::unique_ptr<AbstractValue>* other_value_ptr);

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

  // Provides an identifying prefix for error messages.
  std::string FormatName(const char* api) const {
    return "CacheEntryValue(" + GetPathDescription() + ")::" + api + "(): ";
  }

  // The sense of these flag bits is chosen so that Eval() can check
  // in a single instruction whether it must recalculate. Only if flags==0 can
  // we reuse the existing value. See needs_recomputation() above.
  enum Flags : int {
    kValueIsOutOfDate     = 0b01,
    kCacheEntryIsDisabled = 0b10
  };

  // The index for this CacheEntryValue within its containing subcontext.
  CacheIndex cache_index_;

  // The ticket for this cache entry's managing DependencyTracker in the
  // containing subcontext.
  DependencyTicket ticket_;

  // A human-readable description of this cache entry. Not interpreted by code
  // but useful for error messages.
  std::string description_;

  // Pointer to the system name service of the owning subcontext. Used for
  // error messages.
  reset_on_copy<const internal::SystemPathnameInterface*>
      owning_subcontext_;

  // The value, its serial number, and its validity.
  copyable_unique_ptr<AbstractValue> value_;
  int64_t serial_number_{0};
  int flags_{kValueIsOutOfDate};
};

//==============================================================================
//                                  CACHE
//==============================================================================
/** Stores all the CacheEntryValue objects owned by a particular Context,
organized to allow fast access using a CacheIndex as an index. Memory addresses
of CacheEntryValue objects are stable once allocated, but CacheIndex numbers are
stable even after a Context has been copied so should be preferred. A %Cache is
copyable, assignable, and movable, but not thread-safe. */
class Cache {
 public:
  /** @name  Does not allow move or assignment; copy constructor limited.
  The copy constructor does not copy internal pointers so requires special
  handling. */
  /** @{ */
  Cache(Cache&&) = delete;
  Cache& operator=(const Cache&) = delete;
  Cache& operator=(Cache&&) = delete;
  /** @} */

  /** Constructor creates an empty cache referencing the system pathname
  service of its owning subcontext. The supplied pointer must not be null. */
  explicit Cache(const internal::SystemPathnameInterface* owning_subcontext)
      : owning_subcontext_(owning_subcontext) {
    DRAKE_DEMAND(owning_subcontext != nullptr);
  }

  /** Destruction deletes all cache entries and their contained values; no
  dependency notifications are issued. */
  ~Cache() = default;

  /** Allocates a new CacheEntryValue and corresponding DependencyTracker using
  the given CacheIndex and DependencyTicket number. The CacheEntryValue
  object is owned by this Cache and the returned reference remains valid
  if other cache entry values are created. The created DependencyTracker
  object is owned by the given DependencyGraph, which must be owned by
  the same Context that owns this Cache. The graph must already contain
  trackers for the indicated prerequisites. The new tracker will retain a
  pointer to the created CacheEntryValue for invalidation purposes. */
  CacheEntryValue& CreateNewCacheEntryValue(
      CacheIndex index, DependencyTicket ticket,
      const std::string& description,
      const std::vector<DependencyTicket>& prerequisites,
      DependencyGraph* graph);

  /** Returns true if there is a CacheEntryValue in this cache that has the
  given index. */
  bool has_cache_entry_value(CacheIndex index) const {
    DRAKE_DEMAND(index.is_valid());
    if (index >= cache_size()) return false;
    return store_[index] != nullptr;
  }

  /** Returns the current size of the Cache container, providing for CacheIndex
  values from `0..cache_size()-1`. Note that it is possible to have empty slots
  in the cache. Use has_cache_entry() to determine if there is a cache entry
  associated with a particular index. */
  int cache_size() const { return static_cast<int>(store_.size()); }

  /** Returns a const CacheEntryValue given an index. This is very fast.
  Behavior is undefined if the ticket is out of range [0..cache_size()-1]. */
  const CacheEntryValue& get_cache_entry_value(CacheIndex index) const {
    DRAKE_ASSERT(index.is_valid());
    DRAKE_ASSERT(0 <= index && index < cache_size());
    const CacheEntryValue& cache_value = *store_[index];
    DRAKE_ASSERT(cache_value.cache_index() == index);
    return cache_value;
  }

  /** Returns a mutable CacheEntryValue given an index. This is very fast.
  Behavior is undefined if the index is out of range [0..cache_size()-1]. */
  CacheEntryValue& get_mutable_cache_entry_value(CacheIndex index) {
    return const_cast<CacheEntryValue&>(get_cache_entry_value(index));
  }

  /** (Advanced) Enable or disable caching for all the entries in this %Cache.
  Note that this is done by setting or clearing individual `is_disabled` flags
  in the entries, so it can be changed on a per-entry basis later. This has no
  effect on the `up_to_date` flags. */
  void SetIsCacheDisabled(bool disabled);

  /** (Advanced) Mark every entry in this cache as "out of date". This forces
  the next Eval() request for an entry to perform a recalculation. After that
  normal caching behavior resumes. */
  void SetAllEntriesOutOfDate();

 private:
  // So ContextBase and no one else can copy a Cache.
  friend class ContextBase;

  // Copy constructor duplicates the source %Cache object, with identical
  // contents but with the "owning subcontext" back pointers set to null. Those
  // must be set properly using RepairCachePointers() once the new subcontext is
  // available. This should only be invoked by ContextBase code as part of
  // copying an entire Context tree.
  Cache(const Cache& source) = default;

  // Assumes `this` %Cache is a recent copy that does not yet have its pointers
  // to the system name-providing service of the new owning Context, and sets
  // those pointers. The supplied pointer must not be null, and there must not
  // already be an owning subcontext set here.
  void RepairCachePointers(
      const internal::SystemPathnameInterface* owning_subcontext);

  // The system name service of the subcontext that owns this cache.
  reset_on_copy<const internal::SystemPathnameInterface*>
      owning_subcontext_;

  // All CacheEntryValue objects, indexed by CacheIndex.
  std::vector<drake::copyable_unique_ptr<CacheEntryValue>> store_;
};

}  // namespace systems
}  // namespace drake
