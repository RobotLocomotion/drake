#pragma once

#include <functional>
#include <memory>
#include <set>
#include <string>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/common/value.h"
#include "drake/systems/framework/context_base.h"
#include "drake/systems/framework/framework_common.h"

namespace drake {
namespace systems {

/** A %CacheEntry belongs to a System and represents the properties of one of
that System's cached computations. %CacheEntry objects are assigned CacheIndex
values in the order they are declared; these are unique within a single System
and can be used for quick access to both the %CacheEntry and the corresponding
CacheEntryValue in the System's Context.

%CacheEntry objects are allocated automatically for known System computations
like output ports and time derivatives, and may also be allocated by user code
for other cached computations.

A cache entry's value is always stored as an AbstractValue, which can hold a
concrete value of any copyable type. However, once a value has been allocated
using a particular concrete type, the type cannot be changed.

%CacheEntry objects support four important operations:
- Allocate() returns an object that can hold the cached value.
- Calc() unconditionally computes the cached value.
- Eval() returns a reference to the cached value, first updating with Calc()
    if it was out of date.
- GetKnownUpToDate() returns a reference to the current value that you are
    certain must already be up to date.

The allocation and calculation functions must be provided at the time a
cache entry is declared. That is typically done in a System constructor, in
a manner very similar to the declaration of output ports. */
class CacheEntry {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CacheEntry)

  // TODO(sherm1) These callbacks should not be specific to this class. Move
  // elsewhere, e.g. framework_common.h so they can be shared with output port.

  /** Signature of a function suitable for allocating an object that can hold
  a value of a particular cache entry. The result is always returned as an
  AbstractValue but must contain the correct concrete type. */
  using AllocCallback =
      std::function<std::unique_ptr<AbstractValue>()>;

  /** Signature of a function suitable for calculating a value of a particular
  cache entry, given a place to put the value. */
  using CalcCallback = std::function<void(const ContextBase&, AbstractValue*)>;

  /** (Advanced) Constructs a cache entry within a System and specifies the
  resources it needs.

  This method is intended only for use by the framework which provides much
  nicer APIs for end users. See
  @ref DeclareCacheEntry_documentation "DeclareCacheEntry" for the
  user-facing API documentation.

  The supplied allocator must return a suitable AbstractValue in which to
  hold the result. The supplied calculator function must write to an
  AbstractValue of the same underlying concrete type as is returned by the
  allocator. The allocator function is not invoked here during construction of
  the cache entry. Instead allocation is deferred until the allocator can be
  provided with a complete Context, which cannot occur until the full Diagram
  containing this subsystem has been completed. That way the initial type, size,
  or value can be Context-dependent. The supplied prerequisite tickets are
  interpreted as belonging to the same subsystem that owns this %CacheEntry.

  The list of prerequisites cannot be empty -- a cache entry that really has
  no prerequisites must say so explicitly by providing a list containing only
  `nothing_ticket()` as a prerequisite. The subsystem pointer must not be null,
  and the cache index and ticket must be valid. The description is an arbitrary
  string not interpreted in any way by Drake.

  @throws std::logic_error if the prerequisite list is empty.

  @see drake::systems::SystemBase::DeclareCacheEntry() */
  // All the nontrivial parameters here are moved to the CacheEntry which is
  // why they aren't references.
  CacheEntry(const internal::SystemMessageInterface* owning_system,
             CacheIndex index, DependencyTicket ticket, std::string description,
             AllocCallback alloc_function, CalcCallback calc_function,
             std::set<DependencyTicket> prerequisites_of_calc);

  /** Returns a reference to the set of prerequisites needed by this cache
  entry's Calc() function. These are all within the same subsystem that
  owns this %CacheEntry. */
  const std::set<DependencyTicket>& prerequisites() const {
    return prerequisites_of_calc_;
  }

  /** (Advanced) Returns a mutable reference to the set of prerequisites needed
  by this entry's Calc() function. Any tickets in this set are interpreted as
  referring to prerequisites within the same subsystem that owns this
  %CacheEntry. Modifications take effect the next time the containing System is
  asked to create a Context.

  A cache entry should normally be given its complete set of prerequisites
  at the time it is declared (typically in a System constructor). If
  possible, defer declaration of cache entries until all their prerequisites
  have been declared so that all necessary tickets are available. In Systems
  with complicated extended construction phases it may be awkward or impossible
  to know all the prerequisites at that time. In that case, consider choosing
  a comprehensive prerequisite like `all_input_ports_ticket()` that can include
  as-yet-undeclared prerequisites. If performance requirements preclude that
  approach, then an advanced user may use this method to add more prerequisites
  as their tickets become available. */
  std::set<DependencyTicket>& mutable_prerequisites() {
    return prerequisites_of_calc_;
  }

  /** Invokes this cache entry's allocator function to allocate a concrete
  object suitable for holding the value to be held in this cache entry, and
  returns that as an AbstractValue. The returned object will never be null.
  @throws std::logic_error if the allocator function returned null. */
  std::unique_ptr<AbstractValue> Allocate() const;

  /** Unconditionally computes the value this cache entry should have given a
  particular context, into an already-allocated object.
  @pre `context` is a subcontext that is compatible with the subsystem that owns
       this cache entry.
  @pre `value` is non null and has exactly the same concrete type as that of
       the object returned by this entry's Allocate() method. */
  void Calc(const ContextBase& context, AbstractValue* value) const;

  /** Returns a reference to the up-to-date value of this cache entry contained
  in the given Context. This is the preferred way to obtain a cached value. If
  the value is not already up to date with respect to its prerequisites, or if
  caching is disabled for this entry, then this entry's Calc() method is used
  first to update the value before the reference is returned. The Calc() method
  may be arbitrarily expensive, but this method is constant time and _very_ fast
  if the value is already up to date. If you are certain the value should be up
  to date already, you may use the GetKnownUpToDate() method instead.
  @pre `context` is a subcontext that is compatible with the subsystem that owns
       this cache entry.
  @throws std::logic_error if the value doesn't actually have type V. */
  template <typename ValueType>
  const ValueType& Eval(const ContextBase& context) const {
    const AbstractValue& abstract_value = EvalAbstract(context);
    return ExtractValueOrThrow<ValueType>(abstract_value, __func__);
  }

  /** Returns a reference to the up-to-date abstract value of this cache entry
  contained in the given Context. If the value is not already up to date with
  respect to its prerequisites, or if caching is disabled for this entry, the
  Calc() method above is used first to update the value before the reference is
  returned. This method is constant time and _very_ fast if the value doesn't
  need to be recomputed.
  @pre `context` is a subcontext that is compatible with the subsystem that owns
       this cache entry. */
  // Keep this method as small as possible to encourage inlining; it gets
  // called *a lot*.
  const AbstractValue& EvalAbstract(const ContextBase& context) const {
    const CacheEntryValue& cache_value = get_cache_entry_value(context);
    if (cache_value.needs_recomputation()) UpdateValue(context);
    return cache_value.get_abstract_value();
  }

  /** Returns a reference to the _known up-to-date_ value of this cache entry
  contained in the given Context. The purpose of this method is to avoid
  unexpected recomputations in circumstances where you believe the value must
  already be up to date. Unlike Eval(), this method will throw an exception
  if the value is out of date; it will never attempt a recomputation. The
  behavior here is unaffected if caching is disabled for this cache entry --
  it looks only at the out_of_date flag which is still maintained when caching
  is disabled. This method is constant time and _very_ fast if it succeeds.
  @pre `context` is a subcontext that is compatible with the subsystem that owns
       this cache entry.
  @throws std::logic_error if the value is out of date or if it does not
                           actually have type `ValueType`. */
  // Keep this method as small as possible to encourage inlining; it gets
  // called *a lot*.
  template <typename ValueType>
  const ValueType& GetKnownUpToDate(const ContextBase& context) const {
    const CacheEntryValue& cache_value = get_cache_entry_value(context);
    if (cache_value.is_out_of_date()) ThrowOutOfDate(__func__);
    return ExtractValueOrThrow<ValueType>(cache_value.get_abstract_value(),
                                          __func__);
  }

  /** Returns a reference to the _known up-to-date_ abstract value of this cache
  entry contained in the given Context. See GetKnownUpToDate() for more
  information.
  @pre `context` is a subcontext that is compatible with the subsystem that owns
       this cache entry.
  @throws std::logic_error if the value is not up to date. */
  // Keep this method as small as possible to encourage inlining; it gets
  // called *a lot*.
  const AbstractValue& GetKnownUpToDateAbstract(
      const ContextBase& context) const {
    const CacheEntryValue& cache_value = get_cache_entry_value(context);
    if (cache_value.is_out_of_date()) ThrowOutOfDate(__func__);
    return cache_value.get_abstract_value();
  }

  /** Returns `true` if the current value of this cache entry is out of
  date with respect to its prerequisites. If this returns `false` then the
  Eval() method will not perform any computation when invoked, unless caching
  has been disabled for this entry. If this returns `true` the
  GetKnownUpToDate() methods will fail if invoked. */
  bool is_out_of_date(const ContextBase& context) const {
    return get_cache_entry_value(context).is_out_of_date();
  }

  /** (Debugging) Returns `true` if caching has been disabled for this cache
  entry in the given `context`. That means Eval() will recalculate even if the
  entry is marked up to date. */
  bool is_cache_entry_disabled(const ContextBase& context) const {
    return get_cache_entry_value(context).is_cache_entry_disabled();
  }

  /** (Debugging) Disables caching for this cache entry in the given `context`.
  Eval() will recompute the cached value every time it is invoked, regardless
  of the state of the out_of_date flag. That should have no effect on any
  computed results, other than speed. See class documentation for ideas as to
  what might be wrong if you see a change. Note that the `context` is `const`
  here; cache entry values are mutable. */
  void disable_caching(const ContextBase& context) const {
    CacheEntryValue& value = get_mutable_cache_entry_value(context);
    value.disable_caching();
  }

  /** (Debugging) Enables caching for this cache entry in the given `context`
  if it was previously disabled. */
  void enable_caching(const ContextBase& context) const {
    CacheEntryValue& value = get_mutable_cache_entry_value(context);
    value.enable_caching();
  }

  /** (Debugging) Marks this cache entry so that the corresponding
  CacheEntryValue object in any allocated Context is created with its
  `disabled` flag initially set. This can be useful for debugging when you have
  observed a difference between cached and non-cached behavior that can't be
  diagnosed with the runtime disable_caching() method.
  @see disable_caching() */
  void disable_caching_by_default() {
    is_disabled_by_default_ = true;
  }

  /** (Debugging) Returns the current value of this flag. It is `false` unless
  a call to `disable_caching_by_default()` has previously been made. */
  bool is_disabled_by_default() const { return is_disabled_by_default_; }

  /** Return the human-readable description for this %CacheEntry. */
  const std::string& description() const { return description_; }

  /** (Advanced) Returns a const reference to the CacheEntryValue object that
  corresponds to this %CacheEntry, from the supplied Context. The returned
  object contains the current value and tracks whether it is up to date with
  respect to its prerequisites. If you just need the value, use the Eval()
  method rather than this one. This method is constant time and _very_ fast in
  all circumstances. */
  const CacheEntryValue& get_cache_entry_value(
      const ContextBase& context) const {
    return context.get_cache().get_cache_entry_value(cache_index_);
  }

  /** (Advanced) Returns a mutable reference to the CacheEntryValue object that
  corresponds to this %CacheEntry, from the supplied Context. Note that
  `context` is const; cache values are mutable. Don't call this method unless
  you know what you're doing. This method is constant time and _very_ fast in
  all circumstances. */
  CacheEntryValue& get_mutable_cache_entry_value(
      const ContextBase& context) const {
    return context.get_mutable_cache().get_mutable_cache_entry_value(
        cache_index_);
  }

  /** Returns the CacheIndex used to locate this %CacheEntry within the
  containing System. */
  CacheIndex cache_index() const { return cache_index_; }

  /** Returns the DependencyTicket used to register dependencies on the value
  of this %CacheEntry. This can also be used to locate the DependencyTracker
  that manages dependencies at runtime for the associated CacheEntryValue in
  a Context. */
  DependencyTicket ticket() const { return ticket_; }

 private:
  // Unconditionally update the cache value, which has already been determined
  // to be in need of recomputation (either because it is out of date or
  // because caching was disabled).
  void UpdateValue(const ContextBase& context) const {
    // We can get a mutable cache entry value from a const context.
    CacheEntryValue& mutable_cache_value =
        get_mutable_cache_entry_value(context);
    AbstractValue& value = mutable_cache_value.GetMutableAbstractValueOrThrow();
    // If Calc() throws a recoverable exception, the cache remains out of date.
    Calc(context, &value);
    mutable_cache_value.mark_up_to_date();
  }

  // The value was unexpectedly out of date. Issue a helpful message.
  void ThrowOutOfDate(const char* api) const {
    throw std::logic_error(FormatName(api) + "value out of date.");
  }

  // The user told us the cache entry would have a particular concrete type
  // but it doesn't.
  template <typename ValueType>
  void ThrowBadValueType(const char* api, const AbstractValue& abstract) const {
    throw std::logic_error(FormatName(api) + "wrong value type <" +
                           NiceTypeName::Get<ValueType>() +
                           "> specified but actual type was <" +
                           abstract.GetNiceTypeName() + ">.");
  }

  // Pull a value of a given type from an abstract value or issue a nice
  // message if the type is not correct. Keep this small and inlineable.
  template <typename ValueType>
  const ValueType& ExtractValueOrThrow(const AbstractValue& abstract,
                                       const char* api) const {
    const ValueType* value = abstract.maybe_get_value<ValueType>();
    if (!value)
      ThrowBadValueType<ValueType>(api, abstract);
    return *value;
  }

  // Check that an AbstractValue provided to Calc() is suitable for this cache
  // entry. (Very expensive; use in Debug only.)
  void CheckValidAbstractValue(const AbstractValue& proposed) const;

  // Provides an identifying prefix for error messages.
  std::string FormatName(const char* api) const;

  const internal::SystemMessageInterface* const owning_system_;
  const CacheIndex cache_index_;
  const DependencyTicket ticket_;

  // A human-readable description of this cache entry. Not interpreted by code
  // but useful for error messages.
  const std::string description_;

  const AllocCallback alloc_function_;
  const CalcCallback calc_function_;

  // The list of prerequisites for the calc_function. Whenever one of these
  // changes, the cache value must be recalculated. Note that all possible
  // prerequisites are internal to the containing subsystem, so the ticket
  // alone is a unique specification of a prerequisite.
  std::set<DependencyTicket> prerequisites_of_calc_;

  bool is_disabled_by_default_{false};
};

}  // namespace systems
}  // namespace drake
