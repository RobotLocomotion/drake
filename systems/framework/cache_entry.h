#pragma once

#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/type_safe_index.h"
#include "drake/systems/framework/context_base.h"
#include "drake/systems/framework/framework_common.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

class ContextBase;
class SystemBase;

/** A %CacheEntry belongs to a System and represents the properties of one of
that System's cached computations. %CacheEntry objects are assigned CacheIndex
values in the order they are declared; these are unique within a single System
and can be used for quick access to both the %CacheEntry and the corresponding
CacheEntryValue in the System's Context.

%CacheEntry objects are allocated automatically for known System computations
like output ports and time derivatives, and may also be allocated by user code
for other cached computations.

A cache entry's value is always stored as an AbstractValue, though it is
required to maintain the same underlying concrete type after allocation.

%CacheEntry objects support four important operations:
- Allocate() returns an object that can hold the cached value.
- Calc() unconditionally computes the cached value.
- Eval() updates a cached value if necessary.
- Get() obtains the current value if it is up to date.

The allocation and calculation functions must be provided at the time a
cache entry is declared. That is typically done in a System constructor, in
a manner very similar to the declaration of output ports. */
class CacheEntry {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CacheEntry)

  /** Signature of a function suitable for allocating an object that can hold
  a value of a particular cache entry. The result is always returned as an
  AbstractValue but must contain the correct concrete type. */
  using AllocCallback =
      std::function<std::unique_ptr<AbstractValue>(const ContextBase&)>;

  /** Signature of a function suitable for calculating a value of a particular
  cache entry, given a place to put the value. */
  using CalcCallback = std::function<void(const ContextBase&, AbstractValue*)>;

  /** (Advanced) Constructs a cache entry within a System and specifies the
  resources it needs. This is intended only for use by the framework which
  provides much nicer APIs for end users.

  The supplied allocator must return a suitable AbstractValue in which to
  hold the result. The supplied calculator function must write to an
  AbstractValue of the same underlying concrete type as is returned by the
  allocator. The allocator function is not invoked here during construction of
  the cache entry so it may depend on data that becomes available only after
  completion of the containing System. The supplied prerequisite tickets are
  interpreted as belonging to the same subsystem that owns this %CacheEntry. */
  CacheEntry(const SystemBase* system, CacheIndex index,
             DependencyTicket ticket, std::string description,
             AllocCallback alloc_function, CalcCallback calc_function,
             std::vector<DependencyTicket> prerequisites)
      : system_(*system),
        cache_index_(index),  // Must be valid.
        ticket_(ticket),      // Must be valid.
        description_(std::move(description)),
        alloc_function_(std::move(alloc_function)),
        calc_function_(std::move(calc_function)),
        prerequisites_(std::move(prerequisites)) {
    DRAKE_DEMAND(index.is_valid() && ticket.is_valid());
    DRAKE_DEMAND(system && alloc_function_ && calc_function_);
  }

  /** Replaces the current list of prerequisites with this one. The supplied
  tickets are interpreted as belonging to the same subsystem that owns this
  %CacheEntry. */
  void set_prerequisites(std::vector<DependencyTicket> prerequisites) {
    prerequisites_ = std::move(prerequisites);
  }

  /** Returns a reference to the list of prerequisites needed by this cache
  entry's Calc() function. These are all within the same subsystem that
  owns this %CacheEntry. */
  const std::vector<DependencyTicket>& prerequisites() const {
    return prerequisites_;
  }

  virtual ~CacheEntry() = default;

  /** Returns a reference to the up-to-date value of this cache entry contained
  in the given Context. This is the preferred way to obtain a cached value. If
  the value is not already up to date with respect to its prerequisites, this
  entry's Calc() method is used first to update the value before the reference
  is returned. The Calc() method may be arbitrarily expensive, but this method
  is constant time and _very_ fast if the value is already up to date. If you
  are certain the value should be up to date already, use the Get() method
  instead. */
  template <typename ValueType>
  const ValueType& Eval(const ContextBase& context) const {
    const AbstractValue& abstract_value = EvalAbstract(context);
    return ExtractValueOrThrow<ValueType>(abstract_value, __func__);
  }

  /** Returns a reference to the _known up-to-date_ value of this cache
  entry contained in the given Context. You may not call this method if the
  value is not already up to date with respect to its prerequisites. This method
  is constant time and very fast in all circumstances.
  @throws std::logic_error if the value is not up to date. */
  // Keep this method as small as possible to encourage inlining; it gets
  // called *a lot*.
  template <typename ValueType>
  const ValueType& Get(const ContextBase& context) const {
    const CacheEntryValue& cache_value = get_cache_entry_value(context);
    if (!cache_value.is_up_to_date()) ThrowOutOfDate(__func__);
    return ExtractValueOrThrow<ValueType>(cache_value.get_abstract_value(),
                                          __func__);
  }

  /** Allocates a concrete object suitable for holding the value to be held in
  this cache entry, and returns that as an AbstractValue. The returned object
  will never be null. */
  std::unique_ptr<AbstractValue> Allocate(const ContextBase& context) const;

  /** Unconditionally computes the value this cache entry should have given a
  particular context, into an already-allocated object whose concrete type must
  be exactly the same as the concrete type underlying the AbstractValue that is
  returned by this entry's Allocate() method. */
  void Calc(const ContextBase& context, AbstractValue* value) const;

  /** Returns a reference to the up-to-date abstract value of this cache entry
  contained in the given Context. If the value is not already up to date with
  respect to its prerequisites, or caching is disabled for this entry, the
  Calc() method above is used first to update the value before the reference is
  returned. This method is constant time and very fast if the value doesn't need
  to be recomputed. */
  // Keep this method as small as possible to encourage inlining; it gets
  // called *a lot*.
  const AbstractValue& EvalAbstract(const ContextBase& context) const {
    const CacheEntryValue& cache_value = get_cache_entry_value(context);
    if (cache_value.needs_recomputation()) UpdateValue(context);
    return cache_value.get_abstract_value();
  }

  /** Returns a reference to the _known up-to-date_ abstract value of this cache
  entry contained in the given Context. You may not call this method if the
  value is not already up to date with respect to its prerequisites. This method
  is constant time and very fast in all circumstances. It does not check whether
  caching is enabled for this cache entry.
  @throws std::logic_error if the value is not up to date. */
  // Keep this method as small as possible to encourage inlining; it gets
  // called *a lot*.
  const AbstractValue& GetAbstract(const ContextBase& context) const {
    const CacheEntryValue& cache_value = get_cache_entry_value(context);
    if (!cache_value.is_up_to_date()) ThrowOutOfDate(__func__);
    return cache_value.get_abstract_value();
  }

  /** Returns `true` if the current value of this cache entry is already up to
  date with respect to its prerequisites. If this returns `true` then the
  Eval() method will not perform any computation when invoked, unless caching
  has been disabled for this entry. If this returns `false` the Get() methods
  will fail if invoked. */
  bool is_up_to_date(const ContextBase& context) const {
    return get_cache_entry_value(context).is_up_to_date();
  }

  /** (Debugging) Returns `true` if caching has been disabled for this cache
  entry. That means Eval() will recalculate even if the up-to-date flag is
  set. */
  bool is_entry_disabled(const ContextBase& context) const {
    return get_cache_entry_value(context).is_entry_disabled();
  }

  /** (Debugging) Enables or disables caching for this cache entry. If
  `disabled` is `true`, Eval() will recompute the cached value every time it is
  invoked, regardless of the state of the up-to-date flag. That should have no
  effect on any computed results, other than speed. See class documentation for
  ideas as to what might be wrong if you see a change. Note that the `context`
  is `const` here; cache entry values are mutable. */
  void set_is_entry_disabled(const ContextBase& context, bool disabled) {
    get_mutable_cache_entry_value(context).set_is_entry_disabled(disabled);
  }

  /** Return the human-readable description for this %CacheEntry. */
  const std::string& description() const { return description_; }

  /** (Advanced) Returns a const reference to the CacheEntryValue object that
  corresponds to this %CacheEntry, from the supplied Context. The returned
  object contains the current value and tracks whether it is up to date with
  respect to its prerequisites. If you just need the value, use the Eval()
  method rather than this one. This method is constant time and very fast in all
  circumstances. */
  const CacheEntryValue& get_cache_entry_value(
      const ContextBase& context) const {
    return context.get_cache().get_cache_entry_value(cache_index_);
  }

  /** (Advanced) Returns a mutable reference to the CacheEntryValue object that
  corresponds to this %CacheEntry, from the supplied Context. Note that
  `context` is const; cache values are mutable. Don't call this method unless
  you know what you're doing. This method is constant time and very fast in all
  circumstances. */
  CacheEntryValue& get_mutable_cache_entry_value(
      const ContextBase& context) const {
    return context.get_mutable_cache().get_mutable_cache_entry_value(
        cache_index_);
  }

  /** Returns a reference to the System that owns this cache entry. */
  const SystemBase& get_system() const { return system_; }

  /** Returns the CacheIndex used to locate this %CacheEntry within the
  containing System. */
  CacheIndex cache_index() const { return cache_index_; }

  /** Returns the DependencyTicket used to register dependencies on the value
  of this %CacheEntry. This can also be used to locate the DependencyTracker
  that manages dependencies at runtime for the associated %CacheEntryValue in
  a Context. */
  DependencyTicket ticket() const { return ticket_; }

 private:
  // Unconditionally update the cache value, which has already been determined
  // to be out of date.
  void UpdateValue(const ContextBase& context) const {
    // We can get a mutable cache entry value from a const context.
    CacheEntryValue& mutable_cache_value =
        get_mutable_cache_entry_value(context);
    AbstractValue& value = mutable_cache_value.GetMutableAbstractValueOrThrow();
    // If this throws a recoverable exception, the cache remains out of date.
    Calc(context, &value);
    mutable_cache_value.set_is_up_to_date(true);
  }

  // The value was unexpectedly out of date. Issue a helpful message.
  void ThrowOutOfDate(const char* func_name) const {
    std::ostringstream msg;
    msg << "CacheEntry::" << func_name << "(): value out of date -- "
        << GetCacheEntryIdString() << ".";
    throw std::logic_error(msg.str());
  }

  // The user told us the cache entry would have a particular concrete type
  // but it doesn't.
  template <typename ValueType>
  void ThrowBadValueType(const char* func_name,
                         const AbstractValue& abstract) const {
    std::ostringstream msg;
    msg << "CacheEntry::" << func_name << "(): wrong value type <"
        << NiceTypeName::Get<ValueType>() << "> specified but actual type was <"
        << abstract.GetNiceTypeName() << ">."
        << "\n-- " << GetCacheEntryIdString();
    throw std::logic_error(msg.str());
  }

  // Pull a value of a given type from an abstract value or issue a nice
  // message if the type is not correct.
  template <typename ValueType>
  const ValueType& ExtractValueOrThrow(const AbstractValue& abstract,
                                       const char* func_name) const {
    const ValueType* value = abstract.GetValueIfPossible<ValueType>();
    if (!value)
      ThrowBadValueType<ValueType>(func_name, abstract);
    return *value;
  }

  // This is useful for error messages and produces
  // "cache entry <#> (<description>) of GetSystemIdString()" with whatever
  // System identification string is produced by that method. The angle brackets
  // are not included.
  std::string GetCacheEntryIdString() const;

  // Check that an AbstractValue provided to Calc() is suitable for this cache
  // entry. (Very expensive; use in Debug only.)
  void CheckValidAbstractValue(const ContextBase& context,
                               const AbstractValue& proposed) const;

  const SystemBase& system_;
  const CacheIndex cache_index_;
  const DependencyTicket ticket_;

  // A human-readable description of this cache entry. Not interpreted by code
  // but useful for error messages.
  std::string description_;

  AllocCallback alloc_function_;
  CalcCallback calc_function_;

  // The list of prerequisites for the calc_function. Whenever one of these
  // changes, the cache value must be recalculated. Note that all possible
  // prerequisites are internal to the containing subsystem, so the ticket
  // alone is a unique specification of a prerequisite.
  std::vector<DependencyTicket> prerequisites_;
};

}  // namespace systems
}  // namespace drake
