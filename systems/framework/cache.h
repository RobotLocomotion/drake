#pragma once

/** @file
Declares CacheEntryValue and Cache, which is the container for cache entry
values. */

#include <cstdint>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_assert.h"
#include "drake/common/reset_on_copy.h"
#include "drake/common/value.h"
#include "drake/systems/framework/framework_common.h"

namespace drake {
namespace systems {

class DependencyGraph;

//==============================================================================
//                             CACHE ENTRY VALUE
//==============================================================================
/** (Advanced) This is the representation in the Context for the value of one of
a System's CacheEntry objects. Most users will not use this class directly --
System and CacheEntry provide the most common APIs, and Context provides
additional useful methods.

@see System, CacheEntry, Context for user-facing APIs.

A %CacheEntryValue consists of a single type-erased value, a serial number,
an `out_of_date` flag, and a DependencyTracker ticket. Details:
- "Out of date" means that some prerequisite of this cache entry's computation
  has been changed in the current Context since the stored value was last
  computed, and thus must be recomputed prior to use. On the other hand, if the
  entry is _not_ out of date, then it is "up to date" meaning that if you were
  to recompute it using the current Context values you would get the identical
  result (so don't bother!).
- The "serial number" is an integer that is incremented whenever the value is
  modified, or made available for mutable access. You can use it to recognize
  that you are looking at the same value as you saw at some earlier time. It is
  also useful for performance studies since it is a count of how many times this
  value was recomputed. Note that marking the value "out of date" is _not_ a
  modification; that does not change the serial number. The serial number is
  maintained internally and cannot be user-modified.
- The DependencyTicket ("ticket") stored here identifies the DependencyTracker
  ("tracker") associated with this cache entry. The tracker maintains lists of
  all upstream prerequisites and downstream dependents of the value stored here,
  and also has a pointer to this %CacheEntryValue that it uses for invalidation.
  Upstream modifications cause the tracker to set the `out_of_date` flag here,
  and mark all downstream dependents out of date also. The tracker lives in
  the same subcontext that owns the Cache that owns this %CacheEntryValue.

We sometimes use the terms "invalid" and "invalidate" as synonyms for "out of
date" and "mark out of date".

For debugging purposes, caching may be disabled for an entire Context or for
particular cache entry values. This is independent of the `out_of_date` flag
described above, which is still expected to be operational when caching is
disabled. However, when caching is disabled the Eval() methods will recompute
the contained value even if it is not marked out of date. That should have no
effect other than to slow computation; if results change, something is wrong.
There could be a problem with the specification of dependencies, a bug in user
code such as improper retention of a stale reference, or a bug in the caching
system. */
class CacheEntryValue {
 public:
  /** @name  Does not allow move or assignment; copy constructor is private. */
  /** @{ */
  CacheEntryValue(CacheEntryValue&&) = delete;
  void operator=(const CacheEntryValue&) = delete;
  void operator=(CacheEntryValue&&) = delete;
  /** @} */

  /** Destructs the cache value, but does not issue any notifications to
  downstream dependents. */
  ~CacheEntryValue() = default;

  /** Defines the concrete value type by providing an initial AbstractValue
  object containing an object of the appropriate concrete type. This value
  is marked out of date. It is an error to call this if there is already
  a value here; use has_value() if you want to check first. Also, the given
  initial value may not be null. The serial number is set to 1. No out-of-date
  notifications are sent to downstream dependents. Operation of this
  initialization method is not affected by whether the cache is currently
  frozen. However, the corresponding value won't be accessible while the cache
  remains frozen (since it is out of date).

  @throws std::exception if the given value is null or if there is already a
                         value, or if this %CacheEntryValue is malformed in
                         some detectable way. */
  void SetInitialValue(std::unique_ptr<AbstractValue> init_value) {
    if (init_value == nullptr) {
      throw std::logic_error(FormatName(__func__) +
                             "initial value may not be null.");
    }
    ThrowIfValuePresent(__func__);
    value_ = std::move(init_value);
    serial_number_ = 1;
    mark_out_of_date();
    ThrowIfBadCacheEntryValue();  // Sanity check.
  }

  /** @name      Safe methods for value access and modification
  These are the recommended methods for accessing and modifying the cache entry
  value. They unconditionally check all the relevant preconditions to catch
  usage errors. In particular, access is prevented when the value is out of
  date, and modification is permitted only when (a) the value is _already_ out
  of date (typically because a prerequisite changed), or (b) caching is
  disabled for this entry. For performance-sensitive code, you may need to use
  the parallel set of methods below that check preconditions only in Debug
  builds, but be sure you are gaining significant performance before giving up
  on Release-build validation. */
  //@{

  /** Returns a const reference to the contained abstract value, which must
  not be out of date with respect to any of its prerequisites. It is
  an error to call this if there is no stored value object, or if the value is
  out of date.
  @throws std::exception if there is no value or it is out of date.
  @see get_abstract_value() */
  const AbstractValue& GetAbstractValueOrThrow() const {
    return GetAbstractValueOrThrowHelper(__func__);
  }

  /** Returns a const reference to the contained value of known type V. It is
  an error to call this if there is no stored value, or the value is out of
  date, or the value doesn't actually have type V.
  @throws std::exception if there is no stored value, or if it is out of
                         date, or it doesn't actually have type V.
  @see get_value() */
  template <typename V>
  const V& GetValueOrThrow() const {
    return GetValueOrThrowHelper<V>(__func__);
  }

  /** Assigns a new value to a cache entry and marks it up to date.
  The cache entry must already contain a value object of type V to which the
  new value is assigned, and that value must currently be marked out of date.
  The supplied new value _must_ have been calculated using the current values
  in the owning Context, and we assume that here although this method cannot
  check that assumption. Consequently, this method clears the `out_of_date`
  flag. No out-of-date notifications are issued by this method; we assume
  downstream dependents were marked out of date at the time this value went out
  of date. The serial number is incremented.

  This method is the safest and most convenient way to assign a new value.
  However it requires that a new value be computed and then copied into the
  cache entry, which is fine for small types V but may be too expensive for
  large ones. You can alternatively obtain a mutable reference to the value
  already contained in the cache entry and update it in place via
  GetMutableValueOrThrow().
  @throws std::exception if there is no value, or the value is already up
                         to date, of it doesn't actually have type V.
  @throws std::exception if the cache is frozen.
  @see set_value(), GetMutableValueOrThrow() */
  template <typename V>
  void SetValueOrThrow(const V& new_value) {
    SetValueOrThrowHelper<V>(__func__, new_value);
    ++serial_number_;
    mark_up_to_date();
  }

  /** (Advanced) Returns a mutable reference to the contained value after
  incrementing the serial number. This is for the purpose of performing an
  update or extended computation in place. If possible, use the safer and more
  straightforward method SetValueOrThrow() rather than this method. Mutable
  access is only permitted if the value is already marked out of date (meaning
  that all downstream dependents have already been notified). It is an error to
  call this if there is no stored value, or it is already up to date. Since this
  is intended for relatively expensive computations, these preconditions are
  checked even in Release builds. If you have a small, fast computation to
  perform, use set_value() instead. If your computation completes successfully,
  you must mark the entry up to date yourself using mark_up_to_date() if you
  want anyone to be able to use the new value.
  @throws std::exception if there is no value, or if the value is already
                         up to date.
  @throws std::exception if the cache is frozen.
  @see SetValueOrThrow(), set_value(), mark_up_to_date() */
  AbstractValue& GetMutableAbstractValueOrThrow() {
    return GetMutableAbstractValueOrThrowHelper(__func__);
  }

  /** (Advanced) Convenience method that returns a mutable reference to the
  contained value downcast to its known concrete type. Throws an exception if
  the contained value does not have the indicated concrete type. Note that you
  must call mark_up_to_date() after modifying the value through the returned
  reference. See GetMutableAbstractValueOrThrow() above for more information.
  @throws std::exception if there is no value, or if the value is already
                         up to date, of it doesn't actually have type V.
  @throws std::exception if the cache is frozen.
  @see SetValueOrThrow(), set_value(), mark_up_to_date()
  @tparam V The known actual value type. */
  template <typename V>
  V& GetMutableValueOrThrow() {
    AbstractValue& value = GetMutableAbstractValueOrThrowHelper(__func__);
    return value.get_mutable_value<V>();
  }

  /** (Advanced) Returns a reference to the contained value _without_ checking
  whether the value is out of date. This can be used to check type and size
  information but should not be used to look at the value unless you _really_
  know what you're doing.
  @throws std::exception if there is no contained value. */
  const AbstractValue& PeekAbstractValueOrThrow() const {
    ThrowIfNoValuePresent(__func__);
    return *value_;
  }

  /** (Advanced) Convenience method that provides access to the contained value
  downcast to its known concrete type, _without_ checking whether the value is
  out of date. This can be used to check type and size information but should
  not be used to look at the value unless you _really_ know what you're doing.
  @throws std::exception if there is no contained value, or if the contained
                         value does not actually have type V.
  @tparam V The known actual value type. */
  template <typename V>
  const V& PeekValueOrThrow() const {
    ThrowIfNoValuePresent(__func__);
    return value_->get_value<V>();
  }
  //@}

  /** @name    Fast-but-dangerous methods for highest performance
  These methods check for errors only in Debug builds, but plunge
  blindly onward in Release builds so that they will execute as fast as
  possible. You should use them only in places where performance requirements
  preclude Release-build checks. The best way to determine that is to time the
  code using the always-checked methods vs. these ones. If that's not practical,
  use these only when the containing code is in a very high-rate loop, and
  is a substantial fraction of the total code being executed there. */
  //@{

  /** Returns a const reference to the contained abstract value, which must
  not be out of date with respect to any of its prerequisites. It is an error
  to call this if there is no stored value, or it is out of date. Because this
  is used in performance-critical contexts, these requirements will be
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

  /** Returns a const reference to the contained value of known type V. It is
  an error to call this if there is no stored value, or the value is out of
  date, or the value doesn't actually have type V. Because this is expected to
  be used in performance-critical, inner-loop circumstances, these requirements
  will be checked only in Debug builds. If you are not in a performance-critical
  situation (and you probably are not!), use `GetValueOrThrow<V>`() instead.
  @tparam V The known actual value type. */
  template <typename V>
  const V& get_value() const {
#ifdef DRAKE_ASSERT_IS_ARMED
    return GetValueOrThrowHelper<V>(__func__);
#else
    return value_->get_value<V>();
#endif
  }

  /** Assigns a new value to a cache entry and marks it up to date.
  The cache value must already have a value object of type V to which the
  new value is assigned, and that value must not already be up to date.
  The new value is assumed to be up to date with its prerequisites, so the
  `out_of_date` flag is cleared. No out-of-date notifications are issued by this
  method; we assume downstream dependents were marked out of date at the time
  this value went out of date. The serial number is incremented. If you are not
  in a performance-critical situation (and you probably are not!), use
  `SetValueOrThrow<V>()` instead.
  @throws std::exception if the cache is frozen.
  @tparam V The known actual value type. */
  template <typename V>
  void set_value(const V& new_value) {
#ifdef DRAKE_ASSERT_IS_ARMED
    SetValueOrThrowHelper<V>(__func__, new_value);
#else
    ThrowIfFrozen(__func__);
    value_->set_value<V>(new_value);
#endif
    ++serial_number_;
    mark_up_to_date();
  }

  /** (Advanced) Swaps ownership of the stored value object with the given
  one. The value is marked out of date and the serial number is incremented.
  This is useful for discrete updates of abstract state variables that contain
  large objects. Both values must be non-null and of the same concrete type but
  we won't check for errors except in Debug builds.
  @throws std::exception if the cache is frozen.
  */
  void swap_value(std::unique_ptr<AbstractValue>* other_value) {
    DRAKE_ASSERT_VOID(ThrowIfNoValuePresent(__func__));
    DRAKE_ASSERT_VOID(ThrowIfBadOtherValue(__func__, other_value));
    ThrowIfFrozen(__func__);
    value_.swap(*other_value);
    ++serial_number_;
    mark_out_of_date();
  }
  //@}

  /** @name                  Dependency management
  Methods here deal with management of the `out_of_date` flag and determining
  whether the contained value must be recomputed before use. */
  //@{

  /** Returns `true` if the current value is out of date with respect to any of
  its prerequisites. This refers only to the `out_of_date` flag and is
  independent of whether caching is enabled or disabled. Don't call this if
  there is no value here; use has_value() if you aren't sure.
  @see needs_recomputation() */
  bool is_out_of_date() const {
    DRAKE_ASSERT_VOID(ThrowIfNoValuePresent(__func__));
    return (flags_ & kValueIsOutOfDate) != 0;
  }

  /** Returns `true` if either (a) the value is out of date, or (b) caching
  is disabled for this entry. This is a _very_ fast inline method intended
  to be called every time a cache value is obtained with Eval(). This is
  equivalent to `is_out_of_date() || is_entry_disabled()` but faster.  Don't
  call this if there is no value here; use has_value() if you aren't sure.
  Note that if this returns true while the cache is frozen, any attempt to
  access the value will fail since recomputation is forbidden in that case.
  However, operation of _this_ method is unaffected by whether the cache
  is frozen. */
  bool needs_recomputation() const {
    DRAKE_ASSERT_VOID(ThrowIfNoValuePresent(__func__));
    return flags_ != kReadyToUse;
  }

  /** (Advanced) Marks the cache entry value as up to date with respect to
  its prerequisites, with no other effects. That is, this method clears the
  `out_of_date` flag. In particular, this method does not
  modify the value, does not change the serial number, and does not notify
  downstream dependents of anything. This is a very dangerous method since it
  enables access to the value but can't independently determine whether it is
  really up to date. You should not call it unless you really know what you're
  doing, or have a death wish. Do not call this method if there is no stored
  value object; use has_value() if you aren't sure. This is intended
  to be very fast so doesn't check for a value object except in Debug builds.

  @note Operation of this method is unaffected by whether the cache is
  frozen. It may be useful for testing and debugging in that case but you
  should be _very_ careful if you use it -- once you call this the value
  will be accessible in the frozen cache, regardless of whether it is any
  good! */
  void mark_up_to_date() {
    DRAKE_ASSERT_VOID(ThrowIfNoValuePresent(__func__));
    flags_ &= ~kValueIsOutOfDate;
  }

  /** (Advanced) Marks the cache entry value as _out-of-date_ with respect to
  its prerequisites, with no other effects. In particular, it does not modify
  the value, does not change the serial number, and does not notify downstream
  dependents. You should not call this method unless you know that dependent
  notification has already been taken care of. There are no error conditions;
  even an empty cache entry can be marked out of date.

  @note Operation of this method is unaffected by whether the cache is frozen.
  If you call it in that case the corresponding value will become
  inaccessible since it would require recomputation. */
  void mark_out_of_date() {
    flags_ |= kValueIsOutOfDate;
  }

  /** Returns the serial number of the contained value. This counts up every
  time the contained value changes, or whenever mutable access is granted. */
  int64_t serial_number() const { return serial_number_; }
  //@}

  /** @name                 Bookkeeping methods
  Miscellaneous methods of limited use to most users. */
  //@{

  /** Returns the human-readable description for this %CacheEntryValue. */
  const std::string& description() const { return description_; }

  /** Returns the description, preceded by the full pathname of the subsystem
  associated with the owning subcontext. */
  std::string GetPathDescription() const;

  /** Returns `true` if this %CacheEntryValue currently contains a value object
  at all, regardless of whether it is up to date. There will be no value object
  after default construction, prior to SetInitialValue(). */
  bool has_value() const { return value_ != nullptr; }

  /** Returns the CacheIndex used to locate this %CacheEntryValue within its
  containing subcontext. */
  CacheIndex cache_index() const { return cache_index_; }

  /** Returns the DependencyTicket used to locate the DependencyTracker that
  manages dependencies for this %CacheEntryValue. The ticket refers to a
  tracker that is owned by the same subcontext that owns this
  %CacheEntryValue. */
  DependencyTicket ticket() const { return ticket_; }
  //@}

  /** @name            Testing/debugging utilities
  These are used for disabling and re-enabling caching to determine correctness
  and effectiveness of caching. Usually all cache entries are disabled or
  enabled together using higher-level methods that invoke these ones, but you
  can disable just a single entry if necessary. */
  //@{

  /** Throws an std::exception if there is something clearly wrong with this
  %CacheEntryValue object. If the owning subcontext is known, provide a pointer
  to it here and we'll check that this cache entry agrees. In addition we check
  for other internal inconsistencies.
  @throws std::exception for anything that goes wrong, with an appropriate
                         explanatory message. */
  // These invariants hold for all CacheEntryValues except the dummy one.
  void ThrowIfBadCacheEntryValue(const internal::ContextMessageInterface*
                                     owning_subcontext = nullptr) const;

  /** (Advanced) Disables caching for just this cache entry value. When
  disabled, the corresponding entry's Eval() method will unconditionally invoke
  Calc() to recompute the value, regardless of the setting of the `out_of_date`
  flag. The `disabled` flag is independent of the `out_of_date` flag, which
  will continue to be managed even if caching is disabled. It is also
  independent of whether the cache is frozen, although in that case any
  cache access will fail since recomputation is not permitted in a frozen
  cache. Once unfrozen, caching will remain disabled unless enable_caching()
  is called. */
  void disable_caching() {
    flags_ |= kCacheEntryIsDisabled;
  }

  /** (Advanced) Enables caching for this cache entry value if it was previously
  disabled. When enabled (the default condition) the corresponding entry's
  Eval() method will check the `out_of_date` flag and invoke Calc() only if the
  entry is marked out of date. It is also independent of whether the cache is
  frozen; in that case caching will be enabled once the cache is unfrozen. */
  void enable_caching() {
    flags_ &= ~kCacheEntryIsDisabled;
  }

  /** (Advanced) Returns `true` if caching is disabled for this cache entry.
  This is independent of the `out_of_date` flag, and independent of whether
  the cache is currently frozen. */
  bool is_cache_entry_disabled() const {
    return (flags_ & kCacheEntryIsDisabled) != 0;
  }
  //@}

 private:
  // So Cache and no one else can construct and copy CacheEntryValues.
  friend class Cache;

  // Allow this adapter access to our private constructors on our behalf.
  // TODO(sherm1) This friend declaration allows us to hide constructors we
  //   don't want users to call. But there is still a loophole in that a user
  //   could create objects of this type and get access indirectly. Consider
  //   whether that is a real problem that needs to be solved and if so fix it.
  friend class copyable_unique_ptr<CacheEntryValue>;

  // Default constructor can only be used privately to construct an empty
  // CacheEntryValue with description "DUMMY" and a meaningless value.
  CacheEntryValue()
      : description_("DUMMY"), value_(AbstractValue::Make<int>()) {}

  // Creates a new cache value with the given human-readable description and
  // (optionally) an abstract value that defines the right concrete type for
  // this value. The given cache index and dependency ticket must be valid and
  // are recorded here. Unless you have a good reason to do otherwise, make the
  // description identical to the CacheEntry for which this is the value.
  CacheEntryValue(CacheIndex index, DependencyTicket ticket,
                  std::string description,
                  const internal::ContextMessageInterface* owning_subcontext,
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

  // Copy constructor is private because it requires post-copy cleanup via
  // set_owning_subcontext().
  CacheEntryValue(const CacheEntryValue&) = default;

  // This is the post-copy cleanup method.
  void set_owning_subcontext(
      const internal::ContextMessageInterface* owning_subcontext) {
    DRAKE_DEMAND(owning_subcontext != nullptr);
    DRAKE_DEMAND(owning_subcontext_ == nullptr);
    owning_subcontext_ = owning_subcontext;
  }

  // Fully-checked method with API name to use in error messages.
  const AbstractValue& GetAbstractValueOrThrowHelper(const char* api) const {
    ThrowIfNoValuePresent(api);
    ThrowIfOutOfDate(api);  // Must *not* be out of date!
    return *value_;
  }

  // Note that serial number is incremented here since caller will be stomping
  // on this value.
  AbstractValue& GetMutableAbstractValueOrThrowHelper(const char* api) {
    ThrowIfNoValuePresent(api);
    ThrowIfAlreadyComputed(api);  // *Must* be out of date!
    ThrowIfFrozen(api);
    ++serial_number_;
    return *value_;
  }

  // Adds a check on the concrete value type also.
  template <typename T>
  const T& GetValueOrThrowHelper(const char* api) const {
    return GetAbstractValueOrThrowHelper(api).get_value<T>();
  }

  // Fully-checked method with API name to use in error messages.
  template <typename T>
  void SetValueOrThrowHelper(const char* api, const T& new_value) const {
    ThrowIfNoValuePresent(api);
    ThrowIfAlreadyComputed(api);  // *Must* be out of date!
    ThrowIfFrozen(api);
    return value_->set_value<T>(new_value);
  }

  void ThrowIfNoValuePresent(const char* api) const {
    if (!has_value())
      throw std::logic_error(FormatName(api) + "no value is present.");
  }

  void ThrowIfValuePresent(const char* api) const {
    if (has_value()) {
      throw std::logic_error(FormatName(api) +
          "there is already a value object in this CacheEntryValue.");
    }
  }

  // Throws if "other" doesn't have the same concrete type as this value.
  // Don't call this unless you've already verified that there is a value.
  void ThrowIfBadOtherValue(
      const char* api,
      const std::unique_ptr<AbstractValue>* other_value_ptr) const;

  // This means literally that the out-of-date bit is set; it does not look
  // at whether caching is disabled.
  void ThrowIfOutOfDate(const char* api) const {
    if (is_out_of_date()) {
      throw std::logic_error(FormatName(api) +
                             "the current value is out of date.");
    }
  }

  // This checks that there is *some* reason to recompute -- either out-of-date
  // or caching is disabled.
  void ThrowIfAlreadyComputed(const char* api) const {
    if (!needs_recomputation()) {
      throw std::logic_error(FormatName(api) +
          "the current value is already up to date.");
    }
  }

  // Invoke from any attempt to set or get mutable access to an out-of-date
  // cache entry value.
  void ThrowIfFrozen(const char* api) const {
    if (owning_subcontext_->is_cache_frozen()) {
      throw std::logic_error(FormatName(api) +
          "the cache is frozen but this entry is out of date.");
    }
  }

  // Provides an identifying prefix for error messages.
  std::string FormatName(const char* api) const {
    return "CacheEntryValue(" + GetPathDescription() + ")::" + api + "(): ";
  }

  // The sense of these flag bits is chosen so that Eval() can check in a single
  // instruction whether it must recalculate. Only if flags==0 (kReadyToUse) can
  // we reuse the existing value. See needs_recomputation() above.
  enum Flags : int {
    kReadyToUse           = 0b00,
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
  reset_on_copy<const internal::ContextMessageInterface*>
      owning_subcontext_;

  // The value, its serial number, and its validity. The value is copyable so
  // that we can use a default copy constructor. The serial number is
  // 0 on construction but is always >= 1 once we get an initial value.
  copyable_unique_ptr<AbstractValue> value_;
  int64_t serial_number_{0};
  int flags_{kValueIsOutOfDate};
};

//==============================================================================
//                                  CACHE
//==============================================================================
/** (Advanced) Stores all the CacheEntryValue objects owned by a particular
Context, organized to allow fast access using a CacheIndex as an index. Most
users will not use this class directly -- System and CacheEntry provide the
most common APIs, and Context provides additional useful methods.

@see System, CacheEntry, Context for user-facing APIs.

Memory addresses of CacheEntryValue objects are stable once allocated, but
CacheIndex numbers are stable even after a Context has been copied so should be
preferred as a means for identifying particular cache entries. */
class Cache {
 public:
  /** @name  Does not allow move or assignment; copy constructor is private. */
  //@{
  Cache(Cache&&) = delete;
  void operator=(const Cache&) = delete;
  void operator=(Cache&&) = delete;
  //@}

  /** Constructor creates an empty cache referencing the system pathname
  service of its owning subcontext. The supplied pointer must not be null. */
  explicit Cache(const internal::ContextMessageInterface* owning_subcontext)
      : owning_subcontext_(owning_subcontext) {
    DRAKE_DEMAND(owning_subcontext != nullptr);
  }

  /** Destruction deletes all cache entries and their contained values; no
  dependency notifications are issued. */
  ~Cache() = default;

  /** Allocates a new CacheEntryValue and provides it a DependencyTracker using
  the given CacheIndex and DependencyTicket number. The CacheEntryValue object
  is owned by this Cache and the returned reference remains valid if other cache
  entry values are created. If there is a pre-existing tracker with the given
  ticket number (allowed only for well-known cached computations, such as time
  derivatives), it is assigned the new cache entry value to manage. Otherwise a
  new DependencyTracker is created. The created tracker object is owned by the
  given DependencyGraph, which must be owned by the same Context that owns this
  Cache. The graph must already contain trackers for the indicated
  prerequisites. The tracker will retain a pointer to the created
  CacheEntryValue for invalidation purposes. */
  CacheEntryValue& CreateNewCacheEntryValue(
      CacheIndex index, DependencyTicket ticket,
      const std::string& description,
      const std::set<DependencyTicket>& prerequisites,
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
  in the cache. Use has_cache_entry_value() to determine if there is a cache
  entry associated with a particular index. */
  int cache_size() const { return static_cast<int>(store_.size()); }

  /** Returns a const CacheEntryValue given an index. This is very fast.
  Behavior is undefined if the index is out of range [0..cache_size()-1] or
  if there is no CacheEntryValue with that index. Use has_cache_entry_value()
  first if you aren't sure. */
  const CacheEntryValue& get_cache_entry_value(CacheIndex index) const {
    DRAKE_ASSERT(has_cache_entry_value(index));
    const CacheEntryValue& cache_value = *store_[index];
    DRAKE_ASSERT(cache_value.cache_index() == index);
    return cache_value;
  }

  /** Returns a mutable CacheEntryValue given an index. This is very fast.
  Behavior is undefined if the index is out of range [0..cache_size()-1] or
  if there is no CacheEntryValue with that index. Use has_cache_entry_value()
  first if you aren't sure.  */
  CacheEntryValue& get_mutable_cache_entry_value(CacheIndex index) {
    return const_cast<CacheEntryValue&>(get_cache_entry_value(index));
  }

  /** (Advanced) Disables caching for all the entries in this %Cache. Note that
  this is done by setting individual `is_disabled` flags in the entries, so it
  can be changed on a per-entry basis later. This has no effect on the
  `out_of_date` flags. */
  void DisableCaching();

  /** (Advanced) Re-enables caching for all entries in this %Cache if any were
  previously disabled. Note that this is done by clearing individual
  `is_disabled` flags in the entries, so it overrides any disabling that may
  have been done to individual entries. This has no effect on the `out_of_date`
  flags so subsequent Eval() calls might not initiate recomputation. Use
  SetAllEntriesOutOfDate() if you want to force recomputation. */
  void EnableCaching();

  /** (Advanced) Mark every entry in this cache as "out of date". This forces
  the next Eval() request for an entry to perform a recalculation. After that
  normal caching behavior resumes. */
  void SetAllEntriesOutOfDate();

  /** (Advanced) Sets the "is frozen" flag. Cache entry values should check this
  before permitting mutable access to values.
  @see ContextBase::FreezeCache() for the user-facing API */
  void freeze_cache() {
    is_cache_frozen_ = true;
  }

  /** (Advanced) Clears the "is frozen" flag, permitting normal cache
  activity.
  @see ContextBase::UnfreezeCache() for the user-facing API */
  void unfreeze_cache() {
    is_cache_frozen_ = false;
  }

  /** (Advanced) Reports the current value of the "is frozen" flag.
  @see ContextBase::is_cache_frozen() for the user-facing API */
  bool is_cache_frozen() const { return is_cache_frozen_; }

  /** (Internal use only) Returns a mutable reference to a dummy CacheEntryValue
  that can serve as a /dev/null-like destination for throw-away writes. */
  CacheEntryValue& dummy_cache_entry_value() { return dummy_; }

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
      const internal::ContextMessageInterface* owning_subcontext);

  // The system name service of the subcontext that owns this cache. This should
  // not be copied since it would still refer to the source subcontext.
  reset_on_copy<const internal::ContextMessageInterface*>
      owning_subcontext_;

  // All CacheEntryValue objects, indexed by CacheIndex.
  std::vector<copyable_unique_ptr<CacheEntryValue>> store_;

  // A per-Cache (and hence, per-Context) mutable, unused cache entry value
  // object, which has no valid CacheIndex or DependencyTicket and has a
  // meaningless value. A DependencyTracker may invoke mark_up_to_date()
  // harmlessly on this object, but may not depend on its contents in any way as
  // they may change unexpectedly. The intention is that this object is used as
  // a common throw-away destination for non-cache DependencyTracker
  // invalidations so that invalidation can be done unconditionally, and to the
  // same memory location within a LeafContext, for speed.
  CacheEntryValue dummy_;

  // Whether we are currently preventing mutable access to the cache.
  bool is_cache_frozen_{false};
};

}  // namespace systems
}  // namespace drake
