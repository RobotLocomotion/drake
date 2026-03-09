#pragma once

// GENERATED FILE DO NOT EDIT
// This file contains docstrings for the Python bindings that were
// automatically extracted by mkdoc.py.

#include <array>
#include <utility>

#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

// #include "drake/systems/framework/abstract_value_cloner.h"
// #include "drake/systems/framework/abstract_values.h"
// #include "drake/systems/framework/basic_vector.h"
// #include "drake/systems/framework/bus_value.h"
// #include "drake/systems/framework/cache.h"
// #include "drake/systems/framework/cache_entry.h"
// #include "drake/systems/framework/context.h"
// #include "drake/systems/framework/context_base.h"
// #include "drake/systems/framework/continuous_state.h"
// #include "drake/systems/framework/dependency_tracker.h"
// #include "drake/systems/framework/diagram.h"
// #include "drake/systems/framework/diagram_builder.h"
// #include "drake/systems/framework/diagram_context.h"
// #include "drake/systems/framework/diagram_continuous_state.h"
// #include "drake/systems/framework/diagram_discrete_values.h"
// #include "drake/systems/framework/diagram_output_port.h"
// #include "drake/systems/framework/diagram_state.h"
// #include "drake/systems/framework/discrete_values.h"
// #include "drake/systems/framework/event.h"
// #include "drake/systems/framework/event_collection.h"
// #include "drake/systems/framework/event_status.h"
// #include "drake/systems/framework/fixed_input_port_value.h"
// #include "drake/systems/framework/framework_common.h"
// #include "drake/systems/framework/input_port.h"
// #include "drake/systems/framework/input_port_base.h"
// #include "drake/systems/framework/leaf_context.h"
// #include "drake/systems/framework/leaf_output_port.h"
// #include "drake/systems/framework/leaf_system.h"
// #include "drake/systems/framework/model_values.h"
// #include "drake/systems/framework/output_port.h"
// #include "drake/systems/framework/output_port_base.h"
// #include "drake/systems/framework/parameters.h"
// #include "drake/systems/framework/port_base.h"
// #include "drake/systems/framework/scalar_conversion_traits.h"
// #include "drake/systems/framework/single_output_vector_source.h"
// #include "drake/systems/framework/state.h"
// #include "drake/systems/framework/subvector.h"
// #include "drake/systems/framework/supervector.h"
// #include "drake/systems/framework/system.h"
// #include "drake/systems/framework/system_base.h"
// #include "drake/systems/framework/system_constraint.h"
// #include "drake/systems/framework/system_output.h"
// #include "drake/systems/framework/system_scalar_converter.h"
// #include "drake/systems/framework/system_symbolic_inspector.h"
// #include "drake/systems/framework/system_type_tag.h"
// #include "drake/systems/framework/system_visitor.h"
// #include "drake/systems/framework/value_checker.h"
// #include "drake/systems/framework/value_producer.h"
// #include "drake/systems/framework/value_to_abstract_value.h"
// #include "drake/systems/framework/vector_base.h"
// #include "drake/systems/framework/vector_system.h"
// #include "drake/systems/framework/witness_function.h"
// #include "drake/systems/framework/wrapped_system.h"

// Symbol: pydrake_doc_systems_framework
constexpr struct /* pydrake_doc_systems_framework */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::systems
    struct /* systems */ {
      // Symbol: drake::systems::AbstractParameterIndex
      struct /* AbstractParameterIndex */ {
        // Source: drake/systems/framework/framework_common.h
        const char* doc =
R"""(Serves as the local index for abstract parameters within a given
System and its corresponding Context.)""";
      } AbstractParameterIndex;
      // Symbol: drake::systems::AbstractStateIndex
      struct /* AbstractStateIndex */ {
        // Source: drake/systems/framework/framework_common.h
        const char* doc =
R"""(Serves as the local index for abstract state variables within a given
System and its corresponding Context.)""";
      } AbstractStateIndex;
      // Symbol: drake::systems::AbstractValues
      struct /* AbstractValues */ {
        // Source: drake/systems/framework/abstract_values.h
        const char* doc =
R"""(AbstractValues is a container for non-numerical state and parameters.
It may or may not own the underlying data, and therefore is suitable
for both leaf Systems and diagrams.)""";
        // Symbol: drake::systems::AbstractValues::AbstractValues
        struct /* ctor */ {
          // Source: drake/systems/framework/abstract_values.h
          const char* doc_0args = R"""(Constructs an empty AbstractValues.)""";
          // Source: drake/systems/framework/abstract_values.h
          const char* doc_1args =
R"""(Constructs an AbstractValues that does not own the underlying data.)""";
        } ctor;
        // Symbol: drake::systems::AbstractValues::Clone
        struct /* Clone */ {
          // Source: drake/systems/framework/abstract_values.h
          const char* doc =
R"""(Returns a deep copy of all the data in this AbstractValues. The clone
will own its own data. This is true regardless of whether the data
being cloned had ownership of its data or not.)""";
        } Clone;
        // Symbol: drake::systems::AbstractValues::SetFrom
        struct /* SetFrom */ {
          // Source: drake/systems/framework/abstract_values.h
          const char* doc =
R"""(Copies all of the AbstractValues in ``other`` into this. Asserts if
the two are not equal in size.

Raises:
    RuntimeError if any of the elements are of incompatible type.)""";
        } SetFrom;
        // Symbol: drake::systems::AbstractValues::get_mutable_value
        struct /* get_mutable_value */ {
          // Source: drake/systems/framework/abstract_values.h
          const char* doc =
R"""(Returns the element of AbstractValues at the given ``index``, or
aborts if the index is out-of-bounds.)""";
        } get_mutable_value;
        // Symbol: drake::systems::AbstractValues::get_value
        struct /* get_value */ {
          // Source: drake/systems/framework/abstract_values.h
          const char* doc =
R"""(Returns the element of AbstractValues at the given ``index``, or
aborts if the index is out-of-bounds.)""";
        } get_value;
        // Symbol: drake::systems::AbstractValues::size
        struct /* size */ {
          // Source: drake/systems/framework/abstract_values.h
          const char* doc =
R"""(Returns the number of elements of AbstractValues.)""";
        } size;
      } AbstractValues;
      // Symbol: drake::systems::BasicVector
      struct /* BasicVector */ {
        // Source: drake/systems/framework/basic_vector.h
        const char* doc =
R"""(BasicVector is a semantics-free wrapper around an Eigen vector that
satisfies VectorBase. Once constructed, its size is fixed.)""";
        // Symbol: drake::systems::BasicVector::BasicVector<T>
        struct /* ctor */ {
          // Source: drake/systems/framework/basic_vector.h
          const char* doc_0args = R"""(Constructs an empty BasicVector.)""";
          // Source: drake/systems/framework/basic_vector.h
          const char* doc_1args_size =
R"""(Initializes with the given ``size`` using the drake∷dummy_value<T>,
which is NaN when T = double.)""";
          // Source: drake/systems/framework/basic_vector.h
          const char* doc_1args_vec =
R"""(Constructs a BasicVector with the specified ``vec`` data.)""";
          // Source: drake/systems/framework/basic_vector.h
          const char* doc_1args_init =
R"""(Constructs a BasicVector whose elements are the elements of ``init``.)""";
        } ctor;
        // Symbol: drake::systems::BasicVector::Clone
        struct /* Clone */ {
          // Source: drake/systems/framework/basic_vector.h
          const char* doc =
R"""(Copies the entire vector to a new BasicVector, with the same concrete
implementation type.

Uses the Non-Virtual Interface idiom because smart pointers do not
have type covariance.)""";
        } Clone;
        // Symbol: drake::systems::BasicVector::CopyToVector
        struct /* CopyToVector */ {
          // Source: drake/systems/framework/basic_vector.h
          const char* doc = R"""()""";
        } CopyToVector;
        // Symbol: drake::systems::BasicVector::DoClone
        struct /* DoClone */ {
          // Source: drake/systems/framework/basic_vector.h
          const char* doc =
R"""(Returns a new BasicVector containing a copy of the entire vector.
Caller must take ownership, and may rely on the NVI wrapper to
initialize the clone elementwise.

Subclasses of BasicVector must override DoClone to return their
covariant type.)""";
        } DoClone;
        // Symbol: drake::systems::BasicVector::DoGetAtIndexChecked
        struct /* DoGetAtIndexChecked */ {
          // Source: drake/systems/framework/basic_vector.h
          const char* doc = R"""()""";
        } DoGetAtIndexChecked;
        // Symbol: drake::systems::BasicVector::DoGetAtIndexUnchecked
        struct /* DoGetAtIndexUnchecked */ {
          // Source: drake/systems/framework/basic_vector.h
          const char* doc = R"""()""";
        } DoGetAtIndexUnchecked;
        // Symbol: drake::systems::BasicVector::Make
        struct /* Make */ {
          // Source: drake/systems/framework/basic_vector.h
          const char* doc_1args_init =
R"""(Constructs a BasicVector whose elements are the elements of ``init``.)""";
          // Source: drake/systems/framework/basic_vector.h
          const char* doc_1args_Fargs =
R"""(Constructs a BasicVector where each element is constructed using the
placewise-corresponding member of ``args`` as the sole constructor
argument. For instance: BasicVector<symbolic∷Expression>∷Make("x",
"y", "z");)""";
        } Make;
        // Symbol: drake::systems::BasicVector::MakeRecursive
        struct /* MakeRecursive */ {
          // Source: drake/systems/framework/basic_vector.h
          const char* doc_4args =
R"""(Sets ``data`` at ``index`` to an object of type T, which must have a
single-argument constructor invoked via ``constructor_arg``, and then
recursively invokes itself on the next index with ``recursive`` args.
Helper for BasicVector<T>∷Make.)""";
          // Source: drake/systems/framework/basic_vector.h
          const char* doc_3args =
R"""(Base case for the MakeRecursive template recursion.)""";
        } MakeRecursive;
        // Symbol: drake::systems::BasicVector::ScaleAndAddToVector
        struct /* ScaleAndAddToVector */ {
          // Source: drake/systems/framework/basic_vector.h
          const char* doc = R"""()""";
        } ScaleAndAddToVector;
        // Symbol: drake::systems::BasicVector::SetFromVector
        struct /* SetFromVector */ {
          // Source: drake/systems/framework/basic_vector.h
          const char* doc = R"""()""";
        } SetFromVector;
        // Symbol: drake::systems::BasicVector::SetZero
        struct /* SetZero */ {
          // Source: drake/systems/framework/basic_vector.h
          const char* doc = R"""()""";
        } SetZero;
        // Symbol: drake::systems::BasicVector::get_mutable_value
        struct /* get_mutable_value */ {
          // Source: drake/systems/framework/basic_vector.h
          const char* doc =
R"""(Returns the entire vector as a mutable Eigen∷VectorBlock, which allows
mutation of the values, but does not allow ``resize()`` to be invoked
on the returned object.)""";
        } get_mutable_value;
        // Symbol: drake::systems::BasicVector::get_value
        struct /* get_value */ {
          // Source: drake/systems/framework/basic_vector.h
          const char* doc =
R"""((Don't use this in new code) Returns the entire vector as a const
Eigen∷VectorBlock. Prefer ``value()`` which returns direct access to
the underlying VectorX rather than wrapping it in a VectorBlock.)""";
        } get_value;
        // Symbol: drake::systems::BasicVector::set_value
        struct /* set_value */ {
          // Source: drake/systems/framework/basic_vector.h
          const char* doc =
R"""(Sets the vector to the given value. After a.set_value(b.get_value()),
a must be identical to b.

Raises:
    RuntimeError if the new value has different dimensions.)""";
        } set_value;
        // Symbol: drake::systems::BasicVector::size
        struct /* size */ {
          // Source: drake/systems/framework/basic_vector.h
          const char* doc = R"""()""";
        } size;
        // Symbol: drake::systems::BasicVector::value
        struct /* value */ {
          // Source: drake/systems/framework/basic_vector.h
          const char* doc =
R"""(Returns a const reference to the contained ``VectorX<T>``. This is the
preferred method for examining a BasicVector's value.)""";
        } value;
        // Symbol: drake::systems::BasicVector::values
        struct /* values */ {
          // Source: drake/systems/framework/basic_vector.h
          const char* doc_0args_const =
R"""(Provides const access to the element storage. Prefer the synonymous
public ``value()`` method -- this protected method remains for
backwards compatibility in derived classes.)""";
          // Source: drake/systems/framework/basic_vector.h
          const char* doc_0args_nonconst =
R"""((Advanced) Provides mutable access to the element storage. Be careful
not to resize the storage unless you really know what you're doing.)""";
        } values;
      } BasicVector;
      // Symbol: drake::systems::BusValue
      struct /* BusValue */ {
        // Source: drake/systems/framework/bus_value.h
        const char* doc =
R"""(BusValue is a value type used on input ports and output ports to group
labeled signals into a single port. Each signal is referred to by a
unique name and stored using an AbstractValue.

In some cases the signal names are used only for human-readable
logging or debugging, so can be anything. In other cases, the systems
using the signals will require the signals to use specific names per
some convention (e.g., for a BusSelector the signal names must match
the output port names).

See also:
    BusCreator, BusSelector)""";
        // Symbol: drake::systems::BusValue::BusValue
        struct /* ctor */ {
          // Source: drake/systems/framework/bus_value.h
          const char* doc = R"""(Constructs an empty BusValue.)""";
        } ctor;
        // Symbol: drake::systems::BusValue::Clear
        struct /* Clear */ {
          // Source: drake/systems/framework/bus_value.h
          const char* doc =
R"""(Removes all signals from this. Invalidates all iterators.)""";
        } Clear;
        // Symbol: drake::systems::BusValue::Find
        struct /* Find */ {
          // Source: drake/systems/framework/bus_value.h
          const char* doc =
R"""(Gets one signal value. Returns nullptr if not found. Does not
invalidate any iterators, but the return value is invalidated by a
call to any non-const method on this.)""";
        } Find;
        // Symbol: drake::systems::BusValue::Iterator
        struct /* Iterator */ {
          // Source: drake/systems/framework/bus_value.h
          const char* doc =
R"""(Provides a forward_iterator over BusValue signals. The iteration order
is deterministic but unspecified.)""";
          // Symbol: drake::systems::BusValue::Iterator::Iterator
          struct /* ctor */ {
            // Source: drake/systems/framework/bus_value.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::systems::BusValue::Iterator::difference_type
          struct /* difference_type */ {
            // Source: drake/systems/framework/bus_value.h
            const char* doc = R"""()""";
          } difference_type;
          // Symbol: drake::systems::BusValue::Iterator::operator*
          struct /* operator_mul */ {
            // Source: drake/systems/framework/bus_value.h
            const char* doc = R"""()""";
          } operator_mul;
          // Symbol: drake::systems::BusValue::Iterator::operator++
          struct /* operator_inc */ {
            // Source: drake/systems/framework/bus_value.h
            const char* doc = R"""()""";
          } operator_inc;
          // Symbol: drake::systems::BusValue::Iterator::value_type
          struct /* value_type */ {
            // Source: drake/systems/framework/bus_value.h
            const char* doc = R"""()""";
          } value_type;
        } Iterator;
        // Symbol: drake::systems::BusValue::Set
        struct /* Set */ {
          // Source: drake/systems/framework/bus_value.h
          const char* doc =
R"""(Sets one signal value. Invalidates all iterators. The ``name`` can be
any string without restriction, although we encourage valid UTF-8.

Warning:
    Within a group of BusValue objects that are expected to
    inter-operate (i.e., to be copied or assigned to each other), the
    type of the ``value`` for a given ``name`` is expected to be
    consistent (i.e., homogenous) across the entire group of objects.
    After setting a ``name`` to some value the first time, every
    subsequent call to Set on the same BusValue object for that same
    ``name`` must provide a value of the same type, even if the object
    has since been cleared or copied onto another object. The only way
    to reset the hysteresis for the "presumed type" of a name is to
    construct a new BusValue object. Failure to keep the types
    consistent may result in an exception at runtime. However, we
    might relax this restriction in the future, so don't count on it
    for error handling.)""";
        } Set;
        // Symbol: drake::systems::BusValue::begin
        struct /* begin */ {
          // Source: drake/systems/framework/bus_value.h
          const char* doc = R"""()""";
        } begin;
        // Symbol: drake::systems::BusValue::const_iterator
        struct /* const_iterator */ {
          // Source: drake/systems/framework/bus_value.h
          const char* doc = R"""(@name Container-like type aliases)""";
        } const_iterator;
        // Symbol: drake::systems::BusValue::end
        struct /* end */ {
          // Source: drake/systems/framework/bus_value.h
          const char* doc = R"""()""";
        } end;
        // Symbol: drake::systems::BusValue::value_type
        struct /* value_type */ {
          // Source: drake/systems/framework/bus_value.h
          const char* doc = R"""()""";
        } value_type;
      } BusValue;
      // Symbol: drake::systems::Cache
      struct /* Cache */ {
        // Source: drake/systems/framework/cache.h
        const char* doc =
R"""((Advanced) Stores all the CacheEntryValue objects owned by a
particular Context, organized to allow fast access using a CacheIndex
as an index. Most users will not use this class directly -- System and
CacheEntry provide the most common APIs, and Context provides
additional useful methods.

See also:
    System, CacheEntry, Context for user-facing APIs.

Memory addresses of CacheEntryValue objects are stable once allocated,
but CacheIndex numbers are stable even after a Context has been copied
so should be preferred as a means for identifying particular cache
entries.)""";
        // Symbol: drake::systems::Cache::Cache
        struct /* ctor */ {
          // Source: drake/systems/framework/cache.h
          const char* doc =
R"""(Constructor creates an empty cache referencing the system pathname
service of its owning subcontext. The supplied pointer must not be
null.)""";
        } ctor;
        // Symbol: drake::systems::Cache::CreateNewCacheEntryValue
        struct /* CreateNewCacheEntryValue */ {
          // Source: drake/systems/framework/cache.h
          const char* doc =
R"""(Allocates a new CacheEntryValue and provides it a DependencyTracker
using the given CacheIndex and DependencyTicket number. The
CacheEntryValue object is owned by this Cache and the returned
reference remains valid if other cache entry values are created. If
there is a pre-existing tracker with the given ticket number (allowed
only for well-known cached computations, such as time derivatives), it
is assigned the new cache entry value to manage. Otherwise a new
DependencyTracker is created. The created tracker object is owned by
the given DependencyGraph, which must be owned by the same Context
that owns this Cache. The graph must already contain trackers for the
indicated prerequisites. The tracker will retain a pointer to the
created CacheEntryValue for invalidation purposes.)""";
        } CreateNewCacheEntryValue;
        // Symbol: drake::systems::Cache::DisableCaching
        struct /* DisableCaching */ {
          // Source: drake/systems/framework/cache.h
          const char* doc =
R"""((Advanced) Disables caching for all the entries in this Cache. Note
that this is done by setting individual ``is_disabled`` flags in the
entries, so it can be changed on a per-entry basis later. This has no
effect on the ``out_of_date`` flags.)""";
        } DisableCaching;
        // Symbol: drake::systems::Cache::EnableCaching
        struct /* EnableCaching */ {
          // Source: drake/systems/framework/cache.h
          const char* doc =
R"""((Advanced) Re-enables caching for all entries in this Cache if any
were previously disabled. Note that this is done by clearing
individual ``is_disabled`` flags in the entries, so it overrides any
disabling that may have been done to individual entries. This has no
effect on the ``out_of_date`` flags so subsequent Eval() calls might
not initiate recomputation. Use SetAllEntriesOutOfDate() if you want
to force recomputation.)""";
        } EnableCaching;
        // Symbol: drake::systems::Cache::SetAllEntriesOutOfDate
        struct /* SetAllEntriesOutOfDate */ {
          // Source: drake/systems/framework/cache.h
          const char* doc =
R"""((Advanced) Mark every entry in this cache as "out of date". This
forces the next Eval() request for an entry to perform a
recalculation. After that normal caching behavior resumes.)""";
        } SetAllEntriesOutOfDate;
        // Symbol: drake::systems::Cache::cache_size
        struct /* cache_size */ {
          // Source: drake/systems/framework/cache.h
          const char* doc =
R"""(Returns the current size of the Cache container, providing for
CacheIndex values from ``0..cache_size()-1``. Note that it is possible
to have empty slots in the cache. Use has_cache_entry_value() to
determine if there is a cache entry associated with a particular
index.)""";
        } cache_size;
        // Symbol: drake::systems::Cache::dummy_cache_entry_value
        struct /* dummy_cache_entry_value */ {
          // Source: drake/systems/framework/cache.h
          const char* doc =
R"""((Internal use only) Returns a mutable reference to a dummy
CacheEntryValue that can serve as a /dev/null-like destination for
throw-away writes.)""";
        } dummy_cache_entry_value;
        // Symbol: drake::systems::Cache::freeze_cache
        struct /* freeze_cache */ {
          // Source: drake/systems/framework/cache.h
          const char* doc =
R"""((Advanced) Sets the "is frozen" flag. Cache entry values should check
this before permitting mutable access to values.

See also:
    ContextBase∷FreezeCache() for the user-facing API)""";
        } freeze_cache;
        // Symbol: drake::systems::Cache::get_cache_entry_value
        struct /* get_cache_entry_value */ {
          // Source: drake/systems/framework/cache.h
          const char* doc =
R"""(Returns a const CacheEntryValue given an index. This is very fast.
Behavior is undefined if the index is out of range [0..cache_size()-1]
or if there is no CacheEntryValue with that index. Use
has_cache_entry_value() first if you aren't sure.)""";
        } get_cache_entry_value;
        // Symbol: drake::systems::Cache::get_mutable_cache_entry_value
        struct /* get_mutable_cache_entry_value */ {
          // Source: drake/systems/framework/cache.h
          const char* doc =
R"""(Returns a mutable CacheEntryValue given an index. This is very fast.
Behavior is undefined if the index is out of range [0..cache_size()-1]
or if there is no CacheEntryValue with that index. Use
has_cache_entry_value() first if you aren't sure.)""";
        } get_mutable_cache_entry_value;
        // Symbol: drake::systems::Cache::has_cache_entry_value
        struct /* has_cache_entry_value */ {
          // Source: drake/systems/framework/cache.h
          const char* doc =
R"""(Returns true if there is a CacheEntryValue in this cache that has the
given index.)""";
        } has_cache_entry_value;
        // Symbol: drake::systems::Cache::is_cache_frozen
        struct /* is_cache_frozen */ {
          // Source: drake/systems/framework/cache.h
          const char* doc =
R"""((Advanced) Reports the current value of the "is frozen" flag.

See also:
    ContextBase∷is_cache_frozen() for the user-facing API)""";
        } is_cache_frozen;
        // Symbol: drake::systems::Cache::unfreeze_cache
        struct /* unfreeze_cache */ {
          // Source: drake/systems/framework/cache.h
          const char* doc =
R"""((Advanced) Clears the "is frozen" flag, permitting normal cache
activity.

See also:
    ContextBase∷UnfreezeCache() for the user-facing API)""";
        } unfreeze_cache;
      } Cache;
      // Symbol: drake::systems::CacheEntry
      struct /* CacheEntry */ {
        // Source: drake/systems/framework/cache_entry.h
        const char* doc =
R"""(A CacheEntry belongs to a System and represents the properties of one
of that System's cached computations. CacheEntry objects are assigned
CacheIndex values in the order they are declared; these are unique
within a single System and can be used for quick access to both the
CacheEntry and the corresponding CacheEntryValue in the System's
Context.

CacheEntry objects are allocated automatically for known System
computations like output ports and time derivatives, and may also be
allocated by user code for other cached computations.

A cache entry's value is always stored as an AbstractValue, which can
hold a concrete value of any copyable type. However, once a value has
been allocated using a particular concrete type, the type cannot be
changed.

CacheEntry objects support four important operations: - Allocate()
returns an object that can hold the cached value. - Calc()
unconditionally computes the cached value. - Eval() returns a
reference to the cached value, first updating with Calc() if it was
out of date. - GetKnownUpToDate() returns a reference to the current
value that you are certain must already be up to date.

The allocation and calculation functions must be provided at the time
a cache entry is declared. That is typically done in a System
constructor, in a manner very similar to the declaration of output
ports.)""";
        // Symbol: drake::systems::CacheEntry::Allocate
        struct /* Allocate */ {
          // Source: drake/systems/framework/cache_entry.h
          const char* doc =
R"""(Invokes this cache entry's allocator function to allocate a concrete
object suitable for holding the value to be held in this cache entry,
and returns that as an AbstractValue. The returned object will never
be null.

Raises:
    RuntimeError if the allocator function returned null.)""";
        } Allocate;
        // Symbol: drake::systems::CacheEntry::CacheEntry
        struct /* ctor */ {
          // Source: drake/systems/framework/cache_entry.h
          const char* doc =
R"""((Advanced) Constructs a cache entry within a System and specifies the
resources it needs.

This method is intended only for use by the framework which provides
much nicer APIs for end users. See DeclareCacheEntry_documentation
"DeclareCacheEntry" for the user-facing API documentation.

The ValueProducer embeds both an allocator and calculator function.
The supplied allocator must return a suitable AbstractValue in which
to hold the result. The supplied calculator function must write to an
AbstractValue of the same underlying concrete type as is returned by
the allocator. The allocator function is not invoked here during
construction of the cache entry. Instead allocation is deferred until
the allocator can be provided with a complete Context, which cannot
occur until the full Diagram containing this subsystem has been
completed. That way the initial type, size, or value can be
Context-dependent. The supplied prerequisite tickets are interpreted
as belonging to the same subsystem that owns this CacheEntry.

The list of prerequisites cannot be empty -- a cache entry that really
has no prerequisites must say so explicitly by providing a list
containing only ``nothing_ticket()`` as a prerequisite. The subsystem
pointer must not be null, and the cache index and ticket must be
valid. The description is an arbitrary string not interpreted in any
way by Drake.

Raises:
    RuntimeError if the prerequisite list is empty.

See also:
    drake∷systems∷SystemBase∷DeclareCacheEntry())""";
        } ctor;
        // Symbol: drake::systems::CacheEntry::Calc
        struct /* Calc */ {
          // Source: drake/systems/framework/cache_entry.h
          const char* doc =
R"""(Unconditionally computes the value this cache entry should have given
a particular context, into an already-allocated object.

Precondition:
    ``context`` is a subcontext that is compatible with the subsystem
    that owns this cache entry.

Precondition:
    ``value`` is non null and has exactly the same concrete type as
    that of the object returned by this entry's Allocate() method.)""";
        } Calc;
        // Symbol: drake::systems::CacheEntry::Eval
        struct /* Eval */ {
          // Source: drake/systems/framework/cache_entry.h
          const char* doc =
R"""(Returns a reference to the up-to-date value of this cache entry
contained in the given Context. This is the preferred way to obtain a
cached value. If the value is not already up to date with respect to
its prerequisites, or if caching is disabled for this entry, then this
entry's Calc() method is used first to update the value before the
reference is returned. The Calc() method may be arbitrarily expensive,
but this method is constant time and *very* fast if the value is
already up to date. If you are certain the value should be up to date
already, you may use the GetKnownUpToDate() method instead.

Precondition:
    ``context`` is a subcontext that is compatible with the subsystem
    that owns this cache entry.

Raises:
    RuntimeError if the value doesn't actually have type V.)""";
        } Eval;
        // Symbol: drake::systems::CacheEntry::EvalAbstract
        struct /* EvalAbstract */ {
          // Source: drake/systems/framework/cache_entry.h
          const char* doc = R"""()""";
        } EvalAbstract;
        // Symbol: drake::systems::CacheEntry::GetKnownUpToDate
        struct /* GetKnownUpToDate */ {
          // Source: drake/systems/framework/cache_entry.h
          const char* doc =
R"""(Returns a reference to the *known up-to-date* value of this cache
entry contained in the given Context. The purpose of this method is to
avoid unexpected recomputations in circumstances where you believe the
value must already be up to date. Unlike Eval(), this method will
throw an exception if the value is out of date; it will never attempt
a recomputation. The behavior here is unaffected if caching is
disabled for this cache entry -- it looks only at the out_of_date flag
which is still maintained when caching is disabled. This method is
constant time and *very* fast if it succeeds.

Precondition:
    ``context`` is a subcontext that is compatible with the subsystem
    that owns this cache entry.

Raises:
    RuntimeError if the value is out of date or if it does not
    actually have type ``ValueType``.)""";
        } GetKnownUpToDate;
        // Symbol: drake::systems::CacheEntry::GetKnownUpToDateAbstract
        struct /* GetKnownUpToDateAbstract */ {
          // Source: drake/systems/framework/cache_entry.h
          const char* doc =
R"""(Returns a reference to the *known up-to-date* abstract value of this
cache entry contained in the given Context. See GetKnownUpToDate() for
more information.

Precondition:
    ``context`` is a subcontext that is compatible with the subsystem
    that owns this cache entry.

Raises:
    RuntimeError if the value is not up to date.)""";
        } GetKnownUpToDateAbstract;
        // Symbol: drake::systems::CacheEntry::cache_index
        struct /* cache_index */ {
          // Source: drake/systems/framework/cache_entry.h
          const char* doc =
R"""(Returns the CacheIndex used to locate this CacheEntry within the
containing System.)""";
        } cache_index;
        // Symbol: drake::systems::CacheEntry::description
        struct /* description */ {
          // Source: drake/systems/framework/cache_entry.h
          const char* doc =
R"""(Return the human-readable description for this CacheEntry.)""";
        } description;
        // Symbol: drake::systems::CacheEntry::disable_caching
        struct /* disable_caching */ {
          // Source: drake/systems/framework/cache_entry.h
          const char* doc =
R"""((Debugging) Disables caching for this cache entry in the given
``context``. Eval() will recompute the cached value every time it is
invoked, regardless of the state of the out_of_date flag. That should
have no effect on any computed results, other than speed. See class
documentation for ideas as to what might be wrong if you see a change.
Note that the ``context`` is ``const`` here; cache entry values are
mutable.)""";
        } disable_caching;
        // Symbol: drake::systems::CacheEntry::disable_caching_by_default
        struct /* disable_caching_by_default */ {
          // Source: drake/systems/framework/cache_entry.h
          const char* doc =
R"""((Debugging) Marks this cache entry so that the corresponding
CacheEntryValue object in any allocated Context is created with its
``disabled`` flag initially set. This can be useful for debugging when
you have observed a difference between cached and non-cached behavior
that can't be diagnosed with the runtime disable_caching() method.

See also:
    disable_caching())""";
        } disable_caching_by_default;
        // Symbol: drake::systems::CacheEntry::enable_caching
        struct /* enable_caching */ {
          // Source: drake/systems/framework/cache_entry.h
          const char* doc =
R"""((Debugging) Enables caching for this cache entry in the given
``context`` if it was previously disabled.)""";
        } enable_caching;
        // Symbol: drake::systems::CacheEntry::get_cache_entry_value
        struct /* get_cache_entry_value */ {
          // Source: drake/systems/framework/cache_entry.h
          const char* doc =
R"""((Advanced) Returns a const reference to the CacheEntryValue object
that corresponds to this CacheEntry, from the supplied Context. The
returned object contains the current value and tracks whether it is up
to date with respect to its prerequisites. If you just need the value,
use the Eval() method rather than this one. This method is constant
time and *very* fast in all circumstances.)""";
        } get_cache_entry_value;
        // Symbol: drake::systems::CacheEntry::get_mutable_cache_entry_value
        struct /* get_mutable_cache_entry_value */ {
          // Source: drake/systems/framework/cache_entry.h
          const char* doc =
R"""((Advanced) Returns a mutable reference to the CacheEntryValue object
that corresponds to this CacheEntry, from the supplied Context. Note
that ``context`` is const; cache values are mutable. Don't call this
method unless you know what you're doing. This method is constant time
and *very* fast in all circumstances.)""";
        } get_mutable_cache_entry_value;
        // Symbol: drake::systems::CacheEntry::has_default_prerequisites
        struct /* has_default_prerequisites */ {
          // Source: drake/systems/framework/cache_entry.h
          const char* doc =
R"""((Advanced) Returns ``True`` if this cache entry was created without
specifying any prerequisites. This can be useful in determining
whether the apparent dependencies should be believed, or whether they
may just be due to some user's ignorance.

Note:
    Currently we can't distinguish between "no prerequisites given"
    (in which case we default to ``all_sources_ticket()``) and
    "prerequisite specified as ``all_sources_ticket()`". Either of
    those cases makes this method return `true`` now, but we intend to
    change so that *explicit* specification of
    ``all_sources_ticket()`` will be considered non-default. So don't
    depend on the current behavior.)""";
        } has_default_prerequisites;
        // Symbol: drake::systems::CacheEntry::is_cache_entry_disabled
        struct /* is_cache_entry_disabled */ {
          // Source: drake/systems/framework/cache_entry.h
          const char* doc =
R"""((Debugging) Returns ``True`` if caching has been disabled for this
cache entry in the given ``context``. That means Eval() will
recalculate even if the entry is marked up to date.)""";
        } is_cache_entry_disabled;
        // Symbol: drake::systems::CacheEntry::is_disabled_by_default
        struct /* is_disabled_by_default */ {
          // Source: drake/systems/framework/cache_entry.h
          const char* doc =
R"""((Debugging) Returns the current value of this flag. It is ``False``
unless a call to ``disable_caching_by_default()`` has previously been
made.)""";
        } is_disabled_by_default;
        // Symbol: drake::systems::CacheEntry::is_out_of_date
        struct /* is_out_of_date */ {
          // Source: drake/systems/framework/cache_entry.h
          const char* doc =
R"""(Returns ``True`` if the current value of this cache entry is out of
date with respect to its prerequisites. If this returns ``False`` then
the Eval() method will not perform any computation when invoked,
unless caching has been disabled for this entry. If this returns
``True`` the GetKnownUpToDate() methods will fail if invoked.)""";
        } is_out_of_date;
        // Symbol: drake::systems::CacheEntry::mutable_prerequisites
        struct /* mutable_prerequisites */ {
          // Source: drake/systems/framework/cache_entry.h
          const char* doc =
R"""((Advanced) Returns a mutable reference to the set of prerequisites
needed by this entry's Calc() function. Any tickets in this set are
interpreted as referring to prerequisites within the same subsystem
that owns this CacheEntry. Modifications take effect the next time the
containing System is asked to create a Context.

A cache entry should normally be given its complete set of
prerequisites at the time it is declared (typically in a System
constructor). If possible, defer declaration of cache entries until
all their prerequisites have been declared so that all necessary
tickets are available. In Systems with complicated extended
construction phases it may be awkward or impossible to know all the
prerequisites at that time. In that case, consider choosing a
comprehensive prerequisite like ``all_input_ports_ticket()`` that can
include as-yet-undeclared prerequisites. If performance requirements
preclude that approach, then an advanced user may use this method to
add more prerequisites as their tickets become available.)""";
        } mutable_prerequisites;
        // Symbol: drake::systems::CacheEntry::prerequisites
        struct /* prerequisites */ {
          // Source: drake/systems/framework/cache_entry.h
          const char* doc =
R"""(Returns a reference to the set of prerequisites needed by this cache
entry's Calc() function. These are all within the same subsystem that
owns this CacheEntry.)""";
        } prerequisites;
        // Symbol: drake::systems::CacheEntry::ticket
        struct /* ticket */ {
          // Source: drake/systems/framework/cache_entry.h
          const char* doc =
R"""(Returns the DependencyTicket used to register dependencies on the
value of this CacheEntry. This can also be used to locate the
DependencyTracker that manages dependencies at runtime for the
associated CacheEntryValue in a Context.)""";
        } ticket;
      } CacheEntry;
      // Symbol: drake::systems::CacheEntryValue
      struct /* CacheEntryValue */ {
        // Source: drake/systems/framework/cache.h
        const char* doc =
R"""((Advanced) This is the representation in the Context for the value of
one of a System's CacheEntry objects. Most users will not use this
class directly -- System and CacheEntry provide the most common APIs,
and Context provides additional useful methods.

See also:
    System, CacheEntry, Context for user-facing APIs.

A CacheEntryValue consists of a single type-erased value, a serial
number, an ``out_of_date`` flag, and a DependencyTracker ticket.
Details: - "Out of date" means that some prerequisite of this cache
entry's computation has been changed in the current Context since the
stored value was last computed, and thus must be recomputed prior to
use. On the other hand, if the entry is *not* out of date, then it is
"up to date" meaning that if you were to recompute it using the
current Context values you would get the identical result (so don't
bother!). - The "serial number" is an integer that is incremented
whenever the value is modified, or made available for mutable access.
You can use it to recognize that you are looking at the same value as
you saw at some earlier time. It is also useful for performance
studies since it is a count of how many times this value was
recomputed. Note that marking the value "out of date" is *not* a
modification; that does not change the serial number. The serial
number is maintained internally and cannot be user-modified. - The
DependencyTicket ("ticket") stored here identifies the
DependencyTracker ("tracker") associated with this cache entry. The
tracker maintains lists of all upstream prerequisites and downstream
dependents of the value stored here, and also has a pointer to this
CacheEntryValue that it uses for invalidation. Upstream modifications
cause the tracker to set the ``out_of_date`` flag here, and mark all
downstream dependents out of date also. The tracker lives in the same
subcontext that owns the Cache that owns this CacheEntryValue.

We sometimes use the terms "invalid" and "invalidate" as synonyms for
"out of date" and "mark out of date".

For debugging purposes, caching may be disabled for an entire Context
or for particular cache entry values. This is independent of the
``out_of_date`` flag described above, which is still expected to be
operational when caching is disabled. However, when caching is
disabled the Eval() methods will recompute the contained value even if
it is not marked out of date. That should have no effect other than to
slow computation; if results change, something is wrong. There could
be a problem with the specification of dependencies, a bug in user
code such as improper retention of a stale reference, or a bug in the
caching system.)""";
        // Symbol: drake::systems::CacheEntryValue::CacheEntryValue
        struct /* ctor */ {
          // Source: drake/systems/framework/cache.h
          const char* doc_move =
R"""(@name Does not allow move or assignment; copy constructor is private.)""";
        } ctor;
        // Symbol: drake::systems::CacheEntryValue::GetAbstractValueOrThrow
        struct /* GetAbstractValueOrThrow */ {
          // Source: drake/systems/framework/cache.h
          const char* doc =
R"""(Returns a const reference to the contained abstract value, which must
not be out of date with respect to any of its prerequisites. It is an
error to call this if there is no stored value object, or if the value
is out of date.

Raises:
    RuntimeError if there is no value or it is out of date.

See also:
    get_abstract_value())""";
        } GetAbstractValueOrThrow;
        // Symbol: drake::systems::CacheEntryValue::GetMutableAbstractValueOrThrow
        struct /* GetMutableAbstractValueOrThrow */ {
          // Source: drake/systems/framework/cache.h
          const char* doc =
R"""((Advanced) Returns a mutable reference to the contained value after
incrementing the serial number. This is for the purpose of performing
an update or extended computation in place. If possible, use the safer
and more straightforward method SetValueOrThrow() rather than this
method. Mutable access is only permitted if the value is already
marked out of date (meaning that all downstream dependents have
already been notified). It is an error to call this if there is no
stored value, or it is already up to date. Since this is intended for
relatively expensive computations, these preconditions are checked
even in Release builds. If you have a small, fast computation to
perform, use set_value() instead. If your computation completes
successfully, you must mark the entry up to date yourself using
mark_up_to_date() if you want anyone to be able to use the new value.

Raises:
    RuntimeError if there is no value, or if the value is already up
    to date.

Raises:
    RuntimeError if the cache is frozen.

See also:
    SetValueOrThrow(), set_value(), mark_up_to_date())""";
        } GetMutableAbstractValueOrThrow;
        // Symbol: drake::systems::CacheEntryValue::GetMutableValueOrThrow
        struct /* GetMutableValueOrThrow */ {
          // Source: drake/systems/framework/cache.h
          const char* doc =
R"""((Advanced) Convenience method that returns a mutable reference to the
contained value downcast to its known concrete type. Throws an
exception if the contained value does not have the indicated concrete
type. Note that you must call mark_up_to_date() after modifying the
value through the returned reference. See
GetMutableAbstractValueOrThrow() above for more information.

Raises:
    RuntimeError if there is no value, or if the value is already up
    to date, of it doesn't actually have type V.

Raises:
    RuntimeError if the cache is frozen.

See also:
    SetValueOrThrow(), set_value(), mark_up_to_date()

Template parameter ``V``:
    The known actual value type.)""";
        } GetMutableValueOrThrow;
        // Symbol: drake::systems::CacheEntryValue::GetPathDescription
        struct /* GetPathDescription */ {
          // Source: drake/systems/framework/cache.h
          const char* doc =
R"""(Returns the description, preceded by the full pathname of the
subsystem associated with the owning subcontext.)""";
        } GetPathDescription;
        // Symbol: drake::systems::CacheEntryValue::GetValueOrThrow
        struct /* GetValueOrThrow */ {
          // Source: drake/systems/framework/cache.h
          const char* doc =
R"""(Returns a const reference to the contained value of known type V. It
is an error to call this if there is no stored value, or the value is
out of date, or the value doesn't actually have type V.

Raises:
    RuntimeError if there is no stored value, or if it is out of date,
    or it doesn't actually have type V.

See also:
    get_value())""";
        } GetValueOrThrow;
        // Symbol: drake::systems::CacheEntryValue::PeekAbstractValueOrThrow
        struct /* PeekAbstractValueOrThrow */ {
          // Source: drake/systems/framework/cache.h
          const char* doc =
R"""((Advanced) Returns a reference to the contained value *without*
checking whether the value is out of date. This can be used to check
type and size information but should not be used to look at the value
unless you *really* know what you're doing.

Raises:
    RuntimeError if there is no contained value.)""";
        } PeekAbstractValueOrThrow;
        // Symbol: drake::systems::CacheEntryValue::PeekValueOrThrow
        struct /* PeekValueOrThrow */ {
          // Source: drake/systems/framework/cache.h
          const char* doc =
R"""((Advanced) Convenience method that provides access to the contained
value downcast to its known concrete type, *without* checking whether
the value is out of date. This can be used to check type and size
information but should not be used to look at the value unless you
*really* know what you're doing.

Raises:
    RuntimeError if there is no contained value, or if the contained
    value does not actually have type V.

Template parameter ``V``:
    The known actual value type.)""";
        } PeekValueOrThrow;
        // Symbol: drake::systems::CacheEntryValue::SetInitialValue
        struct /* SetInitialValue */ {
          // Source: drake/systems/framework/cache.h
          const char* doc =
R"""(Defines the concrete value type by providing an initial AbstractValue
object containing an object of the appropriate concrete type. This
value is marked out of date. It is an error to call this if there is
already a value here; use has_value() if you want to check first.
Also, the given initial value may not be null. The serial number is
set to 1. No out-of-date notifications are sent to downstream
dependents. Operation of this initialization method is not affected by
whether the cache is currently frozen. However, the corresponding
value won't be accessible while the cache remains frozen (since it is
out of date).

Raises:
    RuntimeError if the given value is null or if there is already a
    value, or if this CacheEntryValue is malformed in some detectable
    way.)""";
        } SetInitialValue;
        // Symbol: drake::systems::CacheEntryValue::SetValueOrThrow
        struct /* SetValueOrThrow */ {
          // Source: drake/systems/framework/cache.h
          const char* doc =
R"""(Assigns a new value to a cache entry and marks it up to date. The
cache entry must already contain a value object of type V to which the
new value is assigned, and that value must currently be marked out of
date. The supplied new value *must* have been calculated using the
current values in the owning Context, and we assume that here although
this method cannot check that assumption. Consequently, this method
clears the ``out_of_date`` flag. No out-of-date notifications are
issued by this method; we assume downstream dependents were marked out
of date at the time this value went out of date. The serial number is
incremented.

This method is the safest and most convenient way to assign a new
value. However it requires that a new value be computed and then
copied into the cache entry, which is fine for small types V but may
be too expensive for large ones. You can alternatively obtain a
mutable reference to the value already contained in the cache entry
and update it in place via GetMutableValueOrThrow().

Raises:
    RuntimeError if there is no value, or the value is already up to
    date, of it doesn't actually have type V.

Raises:
    RuntimeError if the cache is frozen.

See also:
    set_value(), GetMutableValueOrThrow())""";
        } SetValueOrThrow;
        // Symbol: drake::systems::CacheEntryValue::ThrowIfBadCacheEntryValue
        struct /* ThrowIfBadCacheEntryValue */ {
          // Source: drake/systems/framework/cache.h
          const char* doc =
R"""(Throws an RuntimeError if there is something clearly wrong with this
CacheEntryValue object. If the owning subcontext is known, provide a
pointer to it here and we'll check that this cache entry agrees. In
addition we check for other internal inconsistencies.

Raises:
    RuntimeError for anything that goes wrong, with an appropriate
    explanatory message.)""";
        } ThrowIfBadCacheEntryValue;
        // Symbol: drake::systems::CacheEntryValue::cache_index
        struct /* cache_index */ {
          // Source: drake/systems/framework/cache.h
          const char* doc =
R"""(Returns the CacheIndex used to locate this CacheEntryValue within its
containing subcontext.)""";
        } cache_index;
        // Symbol: drake::systems::CacheEntryValue::description
        struct /* description */ {
          // Source: drake/systems/framework/cache.h
          const char* doc =
R"""(Returns the human-readable description for this CacheEntryValue.)""";
        } description;
        // Symbol: drake::systems::CacheEntryValue::disable_caching
        struct /* disable_caching */ {
          // Source: drake/systems/framework/cache.h
          const char* doc =
R"""((Advanced) Disables caching for just this cache entry value. When
disabled, the corresponding entry's Eval() method will unconditionally
invoke Calc() to recompute the value, regardless of the setting of the
``out_of_date`` flag. The ``disabled`` flag is independent of the
``out_of_date`` flag, which will continue to be managed even if
caching is disabled. It is also independent of whether the cache is
frozen, although in that case any cache access will fail since
recomputation is not permitted in a frozen cache. Once unfrozen,
caching will remain disabled unless enable_caching() is called.)""";
        } disable_caching;
        // Symbol: drake::systems::CacheEntryValue::enable_caching
        struct /* enable_caching */ {
          // Source: drake/systems/framework/cache.h
          const char* doc =
R"""((Advanced) Enables caching for this cache entry value if it was
previously disabled. When enabled (the default condition) the
corresponding entry's Eval() method will check the ``out_of_date``
flag and invoke Calc() only if the entry is marked out of date. It is
also independent of whether the cache is frozen; in that case caching
will be enabled once the cache is unfrozen.)""";
        } enable_caching;
        // Symbol: drake::systems::CacheEntryValue::get_abstract_value
        struct /* get_abstract_value */ {
          // Source: drake/systems/framework/cache.h
          const char* doc =
R"""(Returns a const reference to the contained abstract value, which must
not be out of date with respect to any of its prerequisites. It is an
error to call this if there is no stored value, or it is out of date.
Because this is used in performance-critical contexts, these
requirements will be checked only in Debug builds. If you are not in a
performance-critical situation (and you probably are not!), use
GetAbstractValueOrThrow() instead.)""";
        } get_abstract_value;
        // Symbol: drake::systems::CacheEntryValue::get_value
        struct /* get_value */ {
          // Source: drake/systems/framework/cache.h
          const char* doc =
R"""(Returns a const reference to the contained value of known type V. It
is an error to call this if there is no stored value, or the value is
out of date, or the value doesn't actually have type V. Because this
is expected to be used in performance-critical, inner-loop
circumstances, these requirements will be checked only in Debug
builds. If you are not in a performance-critical situation (and you
probably are not!), use ``GetValueOrThrow<V>``() instead.

Template parameter ``V``:
    The known actual value type.)""";
        } get_value;
        // Symbol: drake::systems::CacheEntryValue::has_value
        struct /* has_value */ {
          // Source: drake/systems/framework/cache.h
          const char* doc =
R"""(Returns ``True`` if this CacheEntryValue currently contains a value
object at all, regardless of whether it is up to date. There will be
no value object after default construction, prior to
SetInitialValue().)""";
        } has_value;
        // Symbol: drake::systems::CacheEntryValue::is_cache_entry_disabled
        struct /* is_cache_entry_disabled */ {
          // Source: drake/systems/framework/cache.h
          const char* doc =
R"""((Advanced) Returns ``True`` if caching is disabled for this cache
entry. This is independent of the ``out_of_date`` flag, and
independent of whether the cache is currently frozen.)""";
        } is_cache_entry_disabled;
        // Symbol: drake::systems::CacheEntryValue::is_out_of_date
        struct /* is_out_of_date */ {
          // Source: drake/systems/framework/cache.h
          const char* doc =
R"""(Returns ``True`` if the current value is out of date with respect to
any of its prerequisites. This refers only to the ``out_of_date`` flag
and is independent of whether caching is enabled or disabled. Don't
call this if there is no value here; use has_value() if you aren't
sure.

See also:
    needs_recomputation())""";
        } is_out_of_date;
        // Symbol: drake::systems::CacheEntryValue::mark_out_of_date
        struct /* mark_out_of_date */ {
          // Source: drake/systems/framework/cache.h
          const char* doc =
R"""((Advanced) Marks the cache entry value as *out-of-date* with respect
to its prerequisites, with no other effects. In particular, it does
not modify the value, does not change the serial number, and does not
notify downstream dependents. You should not call this method unless
you know that dependent notification has already been taken care of.
There are no error conditions; even an empty cache entry can be marked
out of date.

Note:
    Operation of this method is unaffected by whether the cache is
    frozen. If you call it in that case the corresponding value will
    become inaccessible since it would require recomputation.)""";
        } mark_out_of_date;
        // Symbol: drake::systems::CacheEntryValue::mark_up_to_date
        struct /* mark_up_to_date */ {
          // Source: drake/systems/framework/cache.h
          const char* doc =
R"""((Advanced) Marks the cache entry value as up to date with respect to
its prerequisites, with no other effects. That is, this method clears
the ``out_of_date`` flag. In particular, this method does not modify
the value, does not change the serial number, and does not notify
downstream dependents of anything. This is a very dangerous method
since it enables access to the value but can't independently determine
whether it is really up to date. You should not call it unless you
really know what you're doing, or have a death wish. Do not call this
method if there is no stored value object; use has_value() if you
aren't sure. This is intended to be very fast so doesn't check for a
value object except in Debug builds.

Note:
    Operation of this method is unaffected by whether the cache is
    frozen. It may be useful for testing and debugging in that case
    but you should be *very* careful if you use it -- once you call
    this the value will be accessible in the frozen cache, regardless
    of whether it is any good!)""";
        } mark_up_to_date;
        // Symbol: drake::systems::CacheEntryValue::needs_recomputation
        struct /* needs_recomputation */ {
          // Source: drake/systems/framework/cache.h
          const char* doc =
R"""(Returns ``True`` if either (a) the value is out of date, or (b)
caching is disabled for this entry. This is a *very* fast inline
method intended to be called every time a cache value is obtained with
Eval(). This is equivalent to ``is_out_of_date() ||
is_entry_disabled()`` but faster. Don't call this if there is no value
here; use has_value() if you aren't sure. Note that if this returns
true while the cache is frozen, any attempt to access the value will
fail since recomputation is forbidden in that case. However, operation
of *this* method is unaffected by whether the cache is frozen.)""";
        } needs_recomputation;
        // Symbol: drake::systems::CacheEntryValue::serial_number
        struct /* serial_number */ {
          // Source: drake/systems/framework/cache.h
          const char* doc =
R"""(Returns the serial number of the contained value. This counts up every
time the contained value changes, or whenever mutable access is
granted.)""";
        } serial_number;
        // Symbol: drake::systems::CacheEntryValue::set_value
        struct /* set_value */ {
          // Source: drake/systems/framework/cache.h
          const char* doc =
R"""(Assigns a new value to a cache entry and marks it up to date. The
cache value must already have a value object of type V to which the
new value is assigned, and that value must not already be up to date.
The new value is assumed to be up to date with its prerequisites, so
the ``out_of_date`` flag is cleared. No out-of-date notifications are
issued by this method; we assume downstream dependents were marked out
of date at the time this value went out of date. The serial number is
incremented. If you are not in a performance-critical situation (and
you probably are not!), use ``SetValueOrThrow<V>()`` instead.

Raises:
    RuntimeError if the cache is frozen.

Template parameter ``V``:
    The known actual value type.)""";
        } set_value;
        // Symbol: drake::systems::CacheEntryValue::swap_value
        struct /* swap_value */ {
          // Source: drake/systems/framework/cache.h
          const char* doc =
R"""((Advanced) Swaps ownership of the stored value object with the given
one. The value is marked out of date and the serial number is
incremented. This is useful for discrete updates of abstract state
variables that contain large objects. Both values must be non-null and
of the same concrete type but we won't check for errors except in
Debug builds.

Raises:
    RuntimeError if the cache is frozen.)""";
        } swap_value;
        // Symbol: drake::systems::CacheEntryValue::ticket
        struct /* ticket */ {
          // Source: drake/systems/framework/cache.h
          const char* doc =
R"""(Returns the DependencyTicket used to locate the DependencyTracker that
manages dependencies for this CacheEntryValue. The ticket refers to a
tracker that is owned by the same subcontext that owns this
CacheEntryValue.)""";
        } ticket;
      } CacheEntryValue;
      // Symbol: drake::systems::CacheIndex
      struct /* CacheIndex */ {
        // Source: drake/systems/framework/framework_common.h
        const char* doc =
R"""(Serves as a unique identifier for a particular CacheEntry in a System
and the corresponding CacheEntryValue in that System's Context. This
is an index providing extremely fast constant-time access to both.)""";
      } CacheIndex;
      // Symbol: drake::systems::CompositeEventCollection
      struct /* CompositeEventCollection */ {
        // Source: drake/systems/framework/event_collection.h
        const char* doc =
R"""(This class bundles an instance of each EventCollection<EventType> into
one object that stores the heterogeneous collection. This is intended
to hold heterogeneous events returned by methods like
System∷CalcNextUpdateTime.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    CompositeEventCollection<T> = {
      EventCollection<PublishEvent<T>>,
      EventCollection<DiscreteUpdateEvent<T>>,
      EventCollection<UnrestrictedUpdate<T>>}

.. raw:: html

    </details>

There are two concrete derived classes: LeafCompositeEventCollection
and DiagramCompositeEventCollection. Adding new events to the
collection is only allowed for LeafCompositeEventCollection.

End users should never need to use or know about this class. It is for
internal use only.)""";
        // Symbol: drake::systems::CompositeEventCollection::AddDiscreteUpdateEvent
        struct /* AddDiscreteUpdateEvent */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Assuming the internal discrete update event collection is an instance
of LeafEventCollection, adds the discrete update event ``event``
(ownership is also transferred) to it.

Raises:
    RuntimeError if the assumption is incorrect.)""";
        } AddDiscreteUpdateEvent;
        // Symbol: drake::systems::CompositeEventCollection::AddPublishEvent
        struct /* AddPublishEvent */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Assuming the internal publish event collection is an instance of
LeafEventCollection, adds the publish event ``event`` (ownership is
also transferred) to it.

Raises:
    RuntimeError if the assumption is incorrect.)""";
        } AddPublishEvent;
        // Symbol: drake::systems::CompositeEventCollection::AddToEnd
        struct /* AddToEnd */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Adds the contained homogeneous event collections (e.g.,
EventCollection<PublishEvent<T>>,
EventCollection<DiscreteUpdateEvent<T>>, etc.) from ``other`` to the
end of ``this``.)""";
        } AddToEnd;
        // Symbol: drake::systems::CompositeEventCollection::AddUnrestrictedUpdateEvent
        struct /* AddUnrestrictedUpdateEvent */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Assuming the internal unrestricted update event collection is an
instance of LeafEventCollection, adds the unrestricted update event
``event`` (ownership is also transferred) to it.

Raises:
    RuntimeError if the assumption is incorrect.)""";
        } AddUnrestrictedUpdateEvent;
        // Symbol: drake::systems::CompositeEventCollection::Clear
        struct /* Clear */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc = R"""(Clears all the events.)""";
        } Clear;
        // Symbol: drake::systems::CompositeEventCollection::CompositeEventCollection<T>
        struct /* ctor */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Takes ownership of ``pub``, `discrete` and ``unrestricted``. Aborts if
any of these are null.)""";
        } ctor;
        // Symbol: drake::systems::CompositeEventCollection::HasDiscreteUpdateEvents
        struct /* HasDiscreteUpdateEvents */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Returns ``True`` if and only if this collection contains one or more
discrete update events.)""";
        } HasDiscreteUpdateEvents;
        // Symbol: drake::systems::CompositeEventCollection::HasEvents
        struct /* HasEvents */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Returns ``True`` if and only if this collection contains any events.)""";
        } HasEvents;
        // Symbol: drake::systems::CompositeEventCollection::HasPublishEvents
        struct /* HasPublishEvents */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Returns ``True`` if and only if this collection contains one or more
publish events.)""";
        } HasPublishEvents;
        // Symbol: drake::systems::CompositeEventCollection::HasUnrestrictedUpdateEvents
        struct /* HasUnrestrictedUpdateEvents */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Returns ``True`` if and only if this collection contains one or more
unrestricted update events.)""";
        } HasUnrestrictedUpdateEvents;
        // Symbol: drake::systems::CompositeEventCollection::SetFrom
        struct /* SetFrom */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Copies the collections of homogeneous events from ``other`` to
``this``.)""";
        } SetFrom;
        // Symbol: drake::systems::CompositeEventCollection::get_discrete_update_events
        struct /* get_discrete_update_events */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Returns a const reference to the collection of discrete update events.)""";
        } get_discrete_update_events;
        // Symbol: drake::systems::CompositeEventCollection::get_mutable_discrete_update_events
        struct /* get_mutable_discrete_update_events */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Returns a mutable reference to the collection of discrete update
events.)""";
        } get_mutable_discrete_update_events;
        // Symbol: drake::systems::CompositeEventCollection::get_mutable_publish_events
        struct /* get_mutable_publish_events */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Returns a mutable reference to the collection of publish events)""";
        } get_mutable_publish_events;
        // Symbol: drake::systems::CompositeEventCollection::get_mutable_unrestricted_update_events
        struct /* get_mutable_unrestricted_update_events */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Returns a mutable reference to the collection of unrestricted update
events.)""";
        } get_mutable_unrestricted_update_events;
        // Symbol: drake::systems::CompositeEventCollection::get_publish_events
        struct /* get_publish_events */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Returns a const reference to the collection of publish events.)""";
        } get_publish_events;
        // Symbol: drake::systems::CompositeEventCollection::get_system_id
        struct /* get_system_id */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""((Internal use only) Gets the id of the subsystem that created this
collection.)""";
        } get_system_id;
        // Symbol: drake::systems::CompositeEventCollection::get_unrestricted_update_events
        struct /* get_unrestricted_update_events */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Returns a const reference to the collection of unrestricted update
events.)""";
        } get_unrestricted_update_events;
        // Symbol: drake::systems::CompositeEventCollection::set_system_id
        struct /* set_system_id */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""((Internal use only) Records the id of the subsystem that created this
collection.)""";
        } set_system_id;
      } CompositeEventCollection;
      // Symbol: drake::systems::Context
      struct /* Context */ {
        // Source: drake/systems/framework/context.h
        const char* doc =
R"""(Context is an abstract class template that represents all the typed
values that are used in a System's computations: time, numeric-valued
input ports, numerical state, and numerical parameters. There are also
type-erased abstract state variables, abstract-valued input ports,
abstract parameters, and a double accuracy setting. The framework
provides two concrete subclasses of Context: LeafContext (for leaf
Systems) and DiagramContext (for composite System Diagrams). Users are
forbidden to extend DiagramContext and are discouraged from
subclassing LeafContext.

A Context is designed to be used only with the System that created it.
Data encapsulated with State and Parameter objects can be copied
between contexts for compatible systems with some restrictions. For
details, see system_compatibility.)""";
        // Symbol: drake::systems::Context::Clone
        struct /* Clone */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Returns a deep copy of this Context.

Raises:
    RuntimeError if this is not the root context.)""";
        } Clone;
        // Symbol: drake::systems::Context::CloneState
        struct /* CloneState */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Returns a deep copy of this Context's State.)""";
        } CloneState;
        // Symbol: drake::systems::Context::CloneWithoutPointers
        struct /* CloneWithoutPointers */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""((Internal use only) Clones a context but without any of its internal
pointers.)""";
        } CloneWithoutPointers;
        // Symbol: drake::systems::Context::Context<T>
        struct /* ctor */ {
          // Source: drake/systems/framework/context.h
          const char* doc_copy =
R"""(Copy constructor takes care of base class and ``Context<T>`` data
members. Derived classes must implement copy constructors that
delegate to this one for use in their DoCloneWithoutPointers()
implementations.)""";
        } ctor;
        // Symbol: drake::systems::Context::DoCloneState
        struct /* DoCloneState */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Returns the appropriate concrete State object to be returned by
CloneState(). The implementation should not set_system_id on the
result, the caller will set an id on the state after this method
returns.)""";
        } DoCloneState;
        // Symbol: drake::systems::Context::DoPropagateAccuracyChange
        struct /* DoPropagateAccuracyChange */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Invokes PropagateAccuracyChange() on all subcontexts of this Context.
The default implementation does nothing, which is suitable for leaf
contexts. Diagram contexts must override.)""";
        } DoPropagateAccuracyChange;
        // Symbol: drake::systems::Context::DoPropagateTimeChange
        struct /* DoPropagateTimeChange */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Invokes PropagateTimeChange() on all subcontexts of this Context. The
default implementation does nothing, which is suitable for leaf
contexts. Diagram contexts must override.)""";
        } DoPropagateTimeChange;
        // Symbol: drake::systems::Context::GetMutableVZVectors
        struct /* GetMutableVZVectors */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""((Advanced) Returns mutable references to the first-order continuous
state partitions v and z from xc. Performs a single notification sweep
to avoid duplicate notifications for computations that depend on both
v and z. Does *not* invalidate computations that depend on time or
pose q, unless those also depend on v or z.

See also:
    SetTimeAndGetMutableQVector())""";
        } GetMutableVZVectors;
        // Symbol: drake::systems::Context::NoteContinuousStateChange
        struct /* NoteContinuousStateChange */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""((Advanced) Registers an intention to modify the continuous state xc.
Intended use is for integrators that are already holding a mutable
reference to xc which they are going to modify. Performs a
notification sweep to invalidate computations that depend on any
continuous state variables. If you need to change the time also, use
SetTimeAndNoteContinuousStateChange() instead to avoid unnecessary
duplicate notifications.

See also:
    SetTimeAndNoteContinuousStateChange())""";
        } NoteContinuousStateChange;
        // Symbol: drake::systems::Context::PerturbTime
        struct /* PerturbTime */ {
          // Source: drake/systems/framework/context.h
          const char* doc = R"""()""";
        } PerturbTime;
        // Symbol: drake::systems::Context::PropagateAccuracyChange
        struct /* PropagateAccuracyChange */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""((Internal use only) Sets a new accuracy and notifies
accuracy-dependent quantities that they are now invalid, as part of a
given change event.)""";
        } PropagateAccuracyChange;
        // Symbol: drake::systems::Context::PropagateTimeChange
        struct /* PropagateTimeChange */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""((Internal use only) Sets a new time and notifies time-dependent
quantities that they are now invalid, as part of a given change event.)""";
        } PropagateTimeChange;
        // Symbol: drake::systems::Context::SetAbstractState
        struct /* SetAbstractState */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Sets the value of the abstract state variable selected by ``index``.
Sends out of date notifications for all computations that depend on
that abstract state variable. The template type will be inferred and
need not be specified explicitly.

Precondition:
    ``index`` must identify an existing abstract state variable.

Precondition:
    the abstract state's type must match the template argument.

Note:
    Currently notifies dependents of *any* abstract state variable.)""";
        } SetAbstractState;
        // Symbol: drake::systems::Context::SetAccuracy
        struct /* SetAccuracy */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Records the user's requested accuracy, which is a unit-less quantity
designed for use with simulation and other numerical studies. Since
accuracy is unit-less, algorithms and systems are free to interpret
this quantity as they wish. The intention is that more computational
work is acceptable as the accuracy setting is tightened (set closer to
zero). If no accuracy is requested, computations are free to choose
suitable defaults, or to refuse to proceed without an explicit
accuracy setting. The accuracy of a complete simulation or other
numerical study depends on the accuracy of *all* contributing
computations, so it is important that each computation is done in
accordance with the requested accuracy. Some examples of where this is
needed: - Error-controlled numerical integrators use the accuracy
setting to decide what step sizes to take. - The Simulator employs a
numerical integrator, but also uses accuracy to decide how precisely
to isolate witness function zero crossings. - Iterative calculations
reported as results or cached internally depend on accuracy to decide
how strictly to converge the results. Examples of these are:
constraint projection, calculation of distances between smooth shapes,
and deformation calculations for soft contact.

The common thread among these examples is that they all share the same
Context, so by keeping accuracy here it can be used effectively to
control all accuracy-dependent computations.

Any accuracy-dependent computation in this Context and its subcontexts
may be invalidated by a change to the accuracy setting, so out of date
notifications are sent to all such computations (at least if the
accuracy setting has actually changed). Accuracy must have the same
value in every subcontext within the same context tree so may only be
modified at the root context of a tree.

Requested accuracy is stored in the Context for two reasons: - It
permits all computations performed over a System to see the *same*
accuracy request since accuracy is stored in one shared place, and -
it allows us to notify accuracy-dependent cached results that they are
out of date when the accuracy setting changes.

Raises:
    RuntimeError if this is not the root context.)""";
        } SetAccuracy;
        // Symbol: drake::systems::Context::SetContinuousState
        struct /* SetContinuousState */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Sets the continuous state to ``xc``, including q, v, and z partitions.
The supplied vector must be the same size as the existing continuous
state. Sends out of date notifications for all
continuous-state-dependent computations.)""";
        } SetContinuousState;
        // Symbol: drake::systems::Context::SetDiscreteState
        struct /* SetDiscreteState */ {
          // Source: drake/systems/framework/context.h
          const char* doc_single_group =
R"""(Sets the discrete state to ``xd``, assuming there is just one discrete
state group. The supplied vector must be the same size as the existing
discrete state. Sends out of date notifications for all
discrete-state-dependent computations. Use the other signature for
this method if you have multiple discrete state groups.

Precondition:
    There is exactly one discrete state group.)""";
          // Source: drake/systems/framework/context.h
          const char* doc_select_one_group =
R"""(Sets the discrete state group indicated by ``group_index`` to ``xd``.
The supplied vector ``xd`` must be the same size as the existing
discrete state group. Sends out of date notifications for all
computations that depend on this discrete state group.

Precondition:
    ``group_index`` identifies an existing group.

Note:
    Currently notifies dependents of *all* groups.)""";
          // Source: drake/systems/framework/context.h
          const char* doc_set_everything =
R"""(Sets all the discrete state variables in this Context from a
compatible DiscreteValues object.

Raises:
    RuntimeError unless the number of groups and size of each group of
    ``xd`` matches those in this Context.)""";
        } SetDiscreteState;
        // Symbol: drake::systems::Context::SetStateAndParametersFrom
        struct /* SetStateAndParametersFrom */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Copies all state and parameters in ``source``, where numerical values
are of type ``U``, to ``this`` context. Time and accuracy are
unchanged in ``this`` context, which means that this method can be
called on a subcontext. Sends out of date notifications for all
dependent computations in ``this`` context.

Note:
    Currently does not copy fixed input port values from ``source``.
    See System∷FixInputPortsFrom() if you want to copy those.

See also:
    SetTimeStateAndParametersFrom() if you want to copy time and
    accuracy along with state and parameters to a root context.)""";
        } SetStateAndParametersFrom;
        // Symbol: drake::systems::Context::SetTime
        struct /* SetTime */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Sets the current time in seconds. Sends out of date notifications for
all time-dependent computations (at least if the time has actually
changed). Time must have the same value in every subcontext within the
same Diagram context tree so may only be modified at the root context
of the tree.

Raises:
    RuntimeError if this is not the root context.)""";
        } SetTime;
        // Symbol: drake::systems::Context::SetTimeAndContinuousState
        struct /* SetTimeAndContinuousState */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Sets time to ``time_sec`` and continuous state to ``xc``. Performs a
single notification sweep to avoid duplicate notifications for
computations that depend on both time and state.

Raises:
    RuntimeError if this is not the root context.)""";
        } SetTimeAndContinuousState;
        // Symbol: drake::systems::Context::SetTimeAndGetMutableContinuousStateVector
        struct /* SetTimeAndGetMutableContinuousStateVector */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""((Advanced) Sets time and returns a mutable reference to the continuous
state xc (including q, v, z) as a VectorBase. Performs a single
notification sweep to avoid duplicate notifications for computations
that depend on both time and state.

Raises:
    RuntimeError if this is not the root context.

See also:
    SetTimeAndNoteContinuousStateChange()

See also:
    SetTimeAndGetMutableContinuousState())""";
        } SetTimeAndGetMutableContinuousStateVector;
        // Symbol: drake::systems::Context::SetTimeAndGetMutableQVector
        struct /* SetTimeAndGetMutableQVector */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""((Advanced) Sets time and returns a mutable reference to the
second-order continuous state partition q from xc. Performs a single
notification sweep to avoid duplicate notifications for computations
that depend on both time and q.

Raises:
    RuntimeError if this is not the root context.

See also:
    GetMutableVZVectors()

See also:
    SetTimeAndGetMutableContinuousStateVector())""";
        } SetTimeAndGetMutableQVector;
        // Symbol: drake::systems::Context::SetTimeAndNoteContinuousStateChange
        struct /* SetTimeAndNoteContinuousStateChange */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""((Advanced) Sets time and registers an intention to modify the
continuous state xc. Intended use is for integrators that are already
holding a mutable reference to xc which they are going to modify.
Performs a single notification sweep to avoid duplicate notifications
for computations that depend on both time and state.

Raises:
    RuntimeError if this is not the root context.

See also:
    SetTimeAndGetMutableContinuousStateVector())""";
        } SetTimeAndNoteContinuousStateChange;
        // Symbol: drake::systems::Context::SetTimeStateAndParametersFrom
        struct /* SetTimeStateAndParametersFrom */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Copies time, accuracy, all state and all parameters in ``source``,
where numerical values are of type ``U``, to ``this`` context. This
method can only be called on root contexts because time and accuracy
are copied. Sends out of date notifications for all dependent
computations in this context.

Raises:
    RuntimeError if this is not the root context.

Note:
    Currently does not copy fixed input port values from ``source``.
    See System∷FixInputPortsFrom() if you want to copy those.

See also:
    SetStateAndParametersFrom() if you want to copy state and
    parameters to a non-root context.)""";
        } SetTimeStateAndParametersFrom;
        // Symbol: drake::systems::Context::access_mutable_parameters
        struct /* access_mutable_parameters */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""((Internal use only) Returns a reference to mutable parameters
*without* invalidation notifications. Use get_mutable_parameters()
instead for normal access.)""";
        } access_mutable_parameters;
        // Symbol: drake::systems::Context::access_mutable_state
        struct /* access_mutable_state */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""((Internal use only) Returns a reference to a mutable state *without*
invalidation notifications. Use get_mutable_state() instead for normal
access.)""";
        } access_mutable_state;
        // Symbol: drake::systems::Context::do_access_mutable_state
        struct /* do_access_mutable_state */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Returns a mutable reference to its concrete State object *without* any
invalidation. We promise not to allow user access to this object
without invalidation.)""";
        } do_access_mutable_state;
        // Symbol: drake::systems::Context::do_access_state
        struct /* do_access_state */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Returns a const reference to its concrete State object.)""";
        } do_access_state;
        // Symbol: drake::systems::Context::do_to_string
        struct /* do_to_string */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Returns a partial textual description of the Context, intended to be
human-readable. It is not guaranteed to be unambiguous nor complete.)""";
        } do_to_string;
        // Symbol: drake::systems::Context::get_abstract_parameter
        struct /* get_abstract_parameter */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Returns a const reference to the abstract-valued parameter at
``index``.

Precondition:
    ``index`` must identify an existing parameter.)""";
        } get_abstract_parameter;
        // Symbol: drake::systems::Context::get_abstract_state
        struct /* get_abstract_state */ {
          // Source: drake/systems/framework/context.h
          const char* doc_0args =
R"""(Returns a const reference to the abstract component of the state,
which may be of size zero.)""";
          // Source: drake/systems/framework/context.h
          const char* doc_1args =
R"""(Returns a const reference to the abstract component of the state at
``index``.

Precondition:
    ``index`` must identify an existing element.

Precondition:
    the abstract state's type must match the template argument.)""";
        } get_abstract_state;
        // Symbol: drake::systems::Context::get_accuracy
        struct /* get_accuracy */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Returns the accuracy setting (if any). Note that the return type is
``optional<double>`` rather than the double value itself.

See also:
    SetAccuracy() for details.)""";
        } get_accuracy;
        // Symbol: drake::systems::Context::get_continuous_state
        struct /* get_continuous_state */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Returns a const reference to the continuous component of the state,
which may be of size zero.)""";
        } get_continuous_state;
        // Symbol: drake::systems::Context::get_continuous_state_vector
        struct /* get_continuous_state_vector */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Returns a reference to the continuous state vector, devoid of
second-order structure. The vector may be of size zero.)""";
        } get_continuous_state_vector;
        // Symbol: drake::systems::Context::get_discrete_state
        struct /* get_discrete_state */ {
          // Source: drake/systems/framework/context.h
          const char* doc_0args =
R"""(Returns a reference to the entire discrete state, which may consist of
multiple discrete state vectors (groups).)""";
          // Source: drake/systems/framework/context.h
          const char* doc_1args =
R"""(Returns a const reference to group (vector) ``index`` of the discrete
state.

Precondition:
    ``index`` must identify an existing group.)""";
        } get_discrete_state;
        // Symbol: drake::systems::Context::get_discrete_state_vector
        struct /* get_discrete_state_vector */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Returns a reference to the *only* discrete state vector. The vector
may be of size zero.

Precondition:
    There is only one discrete state group.)""";
        } get_discrete_state_vector;
        // Symbol: drake::systems::Context::get_mutable_abstract_parameter
        struct /* get_mutable_abstract_parameter */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Returns a mutable reference to element ``index`` of the
abstract-valued parameters. Sends out of date notifications for all
computations dependent on this parameter.

Precondition:
    ``index`` must identify an existing abstract parameter.

Note:
    Currently notifies dependents of *all* abstract parameters.)""";
        } get_mutable_abstract_parameter;
        // Symbol: drake::systems::Context::get_mutable_abstract_state
        struct /* get_mutable_abstract_state */ {
          // Source: drake/systems/framework/context.h
          const char* doc_0args =
R"""(Returns a mutable reference to the abstract component of the state,
which may be of size zero. Sends out of date notifications for all
abstract-state-dependent computations.

Warning:
    You *must not* use the returned reference to modify the size,
    number, or types of abstract state variables.)""";
          // Source: drake/systems/framework/context.h
          const char* doc_1args =
R"""(Returns a mutable reference to element ``index`` of the abstract
state. Sends out of date notifications for all computations that
depend on this abstract state variable.

Precondition:
    ``index`` must identify an existing element.

Precondition:
    the abstract state's type must match the template argument.

Note:
    Currently notifies dependents of *any* abstract state variable.)""";
        } get_mutable_abstract_state;
        // Symbol: drake::systems::Context::get_mutable_continuous_state
        struct /* get_mutable_continuous_state */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Returns a mutable reference to the continuous component of the state,
which may be of size zero. Sends out of date notifications for all
continuous-state-dependent computations.)""";
        } get_mutable_continuous_state;
        // Symbol: drake::systems::Context::get_mutable_continuous_state_vector
        struct /* get_mutable_continuous_state_vector */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Returns a mutable reference to the continuous state vector, devoid of
second-order structure. The vector may be of size zero. Sends out of
date notifications for all continuous-state-dependent computations.)""";
        } get_mutable_continuous_state_vector;
        // Symbol: drake::systems::Context::get_mutable_discrete_state
        struct /* get_mutable_discrete_state */ {
          // Source: drake/systems/framework/context.h
          const char* doc_0args =
R"""(Returns a mutable reference to the discrete component of the state,
which may be of size zero. Sends out of date notifications for all
discrete-state-dependent computations.

Warning:
    You *must not* use the returned reference to modify the size or
    number of discrete state variables.)""";
          // Source: drake/systems/framework/context.h
          const char* doc_1args =
R"""(Returns a mutable reference to group (vector) ``index`` of the
discrete state. Sends out of date notifications for all computations
that depend on this discrete state group.

Precondition:
    ``index`` must identify an existing group.

Note:
    Currently notifies dependents of *all* groups.)""";
        } get_mutable_discrete_state;
        // Symbol: drake::systems::Context::get_mutable_discrete_state_vector
        struct /* get_mutable_discrete_state_vector */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Returns a mutable reference to the *only* discrete state vector. Sends
out of date notifications for all discrete-state-dependent
computations.

See also:
    get_discrete_state_vector().

Precondition:
    There is only one discrete state group.)""";
        } get_mutable_discrete_state_vector;
        // Symbol: drake::systems::Context::get_mutable_numeric_parameter
        struct /* get_mutable_numeric_parameter */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Returns a mutable reference to element ``index`` of the vector-valued
(numeric) parameters. Sends out of date notifications for all
computations dependent on this parameter.

Precondition:
    ``index`` must identify an existing numeric parameter.

Note:
    Currently notifies dependents of *all* numeric parameters.)""";
        } get_mutable_numeric_parameter;
        // Symbol: drake::systems::Context::get_mutable_parameters
        struct /* get_mutable_parameters */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Returns a mutable reference to this Context's parameters. Sends out of
date notifications for all parameter-dependent computations. If you
don't mean to change all the parameters, use the indexed methods to
modify only some of the parameters so that fewer computations are
invalidated and fewer notifications need be sent.

Warning:
    You *must not* use the returned reference to modify the size,
    number, or types of parameters.)""";
        } get_mutable_parameters;
        // Symbol: drake::systems::Context::get_mutable_state
        struct /* get_mutable_state */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Returns a mutable reference to the whole State, potentially
invalidating *all* state-dependent computations so requiring out of
date notifications to be made for all such computations. If you don't
mean to change the whole state, use more focused methods to modify
only a portion of the state. See class documentation for more
information.

Warning:
    You *must not* use the returned reference to modify the size,
    number, or types of state variables.)""";
        } get_mutable_state;
        // Symbol: drake::systems::Context::get_numeric_parameter
        struct /* get_numeric_parameter */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Returns a const reference to the vector-valued parameter at ``index``.

Precondition:
    ``index`` must identify an existing parameter.)""";
        } get_numeric_parameter;
        // Symbol: drake::systems::Context::get_parameters
        struct /* get_parameters */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Returns a const reference to this Context's parameters.)""";
        } get_parameters;
        // Symbol: drake::systems::Context::get_state
        struct /* get_state */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Returns a const reference to the whole State.)""";
        } get_state;
        // Symbol: drake::systems::Context::get_time
        struct /* get_time */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Returns the current time in seconds.

See also:
    SetTime())""";
        } get_time;
        // Symbol: drake::systems::Context::get_true_time
        struct /* get_true_time */ {
          // Source: drake/systems/framework/context.h
          const char* doc = R"""()""";
        } get_true_time;
        // Symbol: drake::systems::Context::has_only_continuous_state
        struct /* has_only_continuous_state */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Returns true if the Context has continuous state, but no discrete or
abstract state.)""";
        } has_only_continuous_state;
        // Symbol: drake::systems::Context::has_only_discrete_state
        struct /* has_only_discrete_state */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Returns true if the Context has discrete state, but no continuous or
abstract state.)""";
        } has_only_discrete_state;
        // Symbol: drake::systems::Context::init_abstract_state
        struct /* init_abstract_state */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""((Internal use only) Sets the abstract state to ``xa``, deleting
whatever was there before.

Warning:
    Does *not* invalidate state-dependent computations.)""";
        } init_abstract_state;
        // Symbol: drake::systems::Context::init_continuous_state
        struct /* init_continuous_state */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""((Internal use only) Sets the continuous state to ``xc``, deleting
whatever was there before.

Warning:
    Does *not* invalidate state-dependent computations.)""";
        } init_continuous_state;
        // Symbol: drake::systems::Context::init_discrete_state
        struct /* init_discrete_state */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""((Internal use only) Sets the discrete state to ``xd``, deleting
whatever was there before.

Warning:
    Does *not* invalidate state-dependent computations.)""";
        } init_discrete_state;
        // Symbol: drake::systems::Context::init_parameters
        struct /* init_parameters */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""((Internal use only) Sets the parameters to ``params``, deleting
whatever was there before. You must supply a Parameters object; null
is not acceptable.

Warning:
    Does *not* invalidate parameter-dependent computations.)""";
        } init_parameters;
        // Symbol: drake::systems::Context::is_stateless
        struct /* is_stateless */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Returns true if the Context has no state.)""";
        } is_stateless;
        // Symbol: drake::systems::Context::num_abstract_parameters
        struct /* num_abstract_parameters */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Returns the number of abstract-valued parameters.)""";
        } num_abstract_parameters;
        // Symbol: drake::systems::Context::num_abstract_states
        struct /* num_abstract_states */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Returns the number of elements in the abstract state.)""";
        } num_abstract_states;
        // Symbol: drake::systems::Context::num_continuous_states
        struct /* num_continuous_states */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Returns the number of continuous state variables ``xc = {q, v, z}``.)""";
        } num_continuous_states;
        // Symbol: drake::systems::Context::num_discrete_state_groups
        struct /* num_discrete_state_groups */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Returns the number of vectors (groups) in the discrete state.)""";
        } num_discrete_state_groups;
        // Symbol: drake::systems::Context::num_numeric_parameter_groups
        struct /* num_numeric_parameter_groups */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Returns the number of vector-valued parameters.)""";
        } num_numeric_parameter_groups;
        // Symbol: drake::systems::Context::num_total_states
        struct /* num_total_states */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Returns the total dimension of all of the basic vector states (as if
they were muxed).

Raises:
    RuntimeError if the system contains any abstract state.)""";
        } num_total_states;
        // Symbol: drake::systems::Context::to_string
        struct /* to_string */ {
          // Source: drake/systems/framework/context.h
          const char* doc =
R"""(Returns a partial textual description of the Context, intended to be
human-readable. It is not guaranteed to be unambiguous nor complete.)""";
        } to_string;
      } Context;
      // Symbol: drake::systems::ContextBase
      struct /* ContextBase */ {
        // Source: drake/systems/framework/context_base.h
        const char* doc =
R"""(Provides non-templatized Context functionality shared by the
templatized derived classes. That includes caching, dependency
tracking, and management of local values for fixed input ports.

Terminology: in general a Drake System is a tree structure composed of
"subsystems", which are themselves System objects. The corresponding
Context is a parallel tree structure composed of "subcontexts", which
are themselves Context objects. There is a one-to-one correspondence
between subsystems and subcontexts. Within a given System (Context),
its child subsystems (subcontexts) are indexed using a SubsystemIndex;
there is no separate SubcontextIndex since the numbering must be
identical.)""";
        // Symbol: drake::systems::ContextBase::AddAbstractParameterTicket
        struct /* AddAbstractParameterTicket */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(Adds a ticket to the list of abstract parameter tickets.)""";
        } AddAbstractParameterTicket;
        // Symbol: drake::systems::ContextBase::AddAbstractStateTicket
        struct /* AddAbstractStateTicket */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(Adds a ticket to the list of abstract state tickets.)""";
        } AddAbstractStateTicket;
        // Symbol: drake::systems::ContextBase::AddDiscreteStateTicket
        struct /* AddDiscreteStateTicket */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(Adds a ticket to the list of discrete state tickets.)""";
        } AddDiscreteStateTicket;
        // Symbol: drake::systems::ContextBase::AddInputPort
        struct /* AddInputPort */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(Adds the next input port. Expected index is supplied along with the
assigned ticket. Subscribes the "all input ports" tracker to this one.
The fixed_input_type_checker will be used for validation when setting
a fixed input, or may be null when no validation should be performed.
Typically the fixed_input_type_checker is created by
System∷MakeFixInputPortTypeChecker. The fixed_input_type_checker
lifetime will be the same as this ContextBase, so it should not depend
on pointers that may go out of scope. Most acutely, the function must
not depend on any captured SystemBase pointers.)""";
        } AddInputPort;
        // Symbol: drake::systems::ContextBase::AddNumericParameterTicket
        struct /* AddNumericParameterTicket */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(Adds a ticket to the list of numeric parameter tickets.)""";
        } AddNumericParameterTicket;
        // Symbol: drake::systems::ContextBase::AddOutputPort
        struct /* AddOutputPort */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(Adds the next output port. Expected index is supplied along with the
assigned ticket.)""";
        } AddOutputPort;
        // Symbol: drake::systems::ContextBase::BuildTrackerPointerMap
        struct /* BuildTrackerPointerMap */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""((Internal use only) Given a new context ``clone`` containing an
identically-structured dependency graph as the one in ``source``,
creates a mapping of all tracker memory addresses from ``source`` to
``clone``. This must be done for the whole Context tree because
pointers can point outside of their containing subcontext.)""";
        } BuildTrackerPointerMap;
        // Symbol: drake::systems::ContextBase::Clone
        struct /* Clone */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(Creates an identical copy of the concrete context object.

Raises:
    RuntimeError if this is not the root context.)""";
        } Clone;
        // Symbol: drake::systems::ContextBase::CloneWithoutPointers
        struct /* CloneWithoutPointers */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""((Internal use only) Clones a context but without copying any of its
internal pointers; the clone's pointers are set to null.)""";
        } CloneWithoutPointers;
        // Symbol: drake::systems::ContextBase::ContextBase
        struct /* ctor */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc_move =
R"""(@name Does not allow copy, move, or assignment.)""";
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(Default constructor creates an empty ContextBase but initializes all
the built-in dependency trackers that are the same in every System
(like time, q, all states, all inputs, etc.). We can't allocate
trackers for individual discrete & abstract states, parameters, or
input ports since we don't yet know how many there are.)""";
          // Source: drake/systems/framework/context_base.h
          const char* doc_copy =
R"""(Copy constructor takes care of base class data members, but *does not*
fix up base class pointers. Derived classes must implement copy
constructors that delegate to this one for use in their
DoCloneWithoutPointers() implementations. The cache and dependency
graph are copied, but any pointers contained in the source are left
null in the copy.)""";
        } ctor;
        // Symbol: drake::systems::ContextBase::DisableCaching
        struct /* DisableCaching */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""((Debugging) Disables caching recursively for this context and all its
subcontexts. Caching is enabled by default. Disabling forces every
``Eval()`` method to perform a full calculation rather than returning
the cached one. Results should be identical with or without caching,
except for performance. If they are not, there is likely a problem
with (a) the specified dependencies for some calculation, or (b) a
misuse of references into cached values that hides modifications from
the caching system, or (c) a bug in the caching system. The
``is_disabled`` flags are independent of the ``out_of_date`` flags,
which continue to be maintained even when caching is disabled (though
they are ignored). Caching can be re-enabled using EnableCaching().)""";
        } DisableCaching;
        // Symbol: drake::systems::ContextBase::DoCloneWithoutPointers
        struct /* DoCloneWithoutPointers */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(Derived classes must implement this so that it performs the complete
deep copy of the context, including all base class members but not
fixing up base class pointers. To do that, implement a protected copy
constructor that inherits from the base class copy constructor (which
doesn't repair the pointers), then implement DoCloneWithoutPointers()
as ``return std∷unique_ptr<ContextBase>(new DerivedType(*this));``.)""";
        } DoCloneWithoutPointers;
        // Symbol: drake::systems::ContextBase::DoPropagateBuildTrackerPointerMap
        struct /* DoPropagateBuildTrackerPointerMap */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(DiagramContext must implement this to invoke BuildTrackerPointerMap()
on each of its subcontexts. The default implementation does nothing
which is fine for a LeafContext.)""";
        } DoPropagateBuildTrackerPointerMap;
        // Symbol: drake::systems::ContextBase::DoPropagateBulkChange
        struct /* DoPropagateBulkChange */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(DiagramContext must implement this to invoke PropagateBulkChange() on
its subcontexts, passing along the indicated method that specifies the
particular bulk change (e.g. whole state, all parameters, all discrete
state variables, etc.). The default implementation does nothing which
is fine for a LeafContext.)""";
        } DoPropagateBulkChange;
        // Symbol: drake::systems::ContextBase::DoPropagateCachingChange
        struct /* DoPropagateCachingChange */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(DiagramContext must implement this to invoke a caching behavior change
on each of its subcontexts. The default implementation does nothing
which is fine for a LeafContext.)""";
        } DoPropagateCachingChange;
        // Symbol: drake::systems::ContextBase::DoPropagateFixContextPointers
        struct /* DoPropagateFixContextPointers */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(DiagramContext must implement this to invoke FixContextPointers() on
each of its subcontexts. The default implementation does nothing which
is fine for a LeafContext.)""";
        } DoPropagateFixContextPointers;
        // Symbol: drake::systems::ContextBase::EnableCaching
        struct /* EnableCaching */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""((Debugging) Re-enables caching recursively for this context and all
its subcontexts. Caching is enabled by default but may have been
disabled via a call to DisableCaching(). The ``is_disabled`` flags are
independent of the ``out_of_date`` flags, which continue to be
maintained even when caching is disabled (though they are ignored).
Hence re-enabling the cache with this method may result in some
entries being already considered up to date. See
SetAllCacheEntriesOutOfDate() if you want to ensure that caching
restarts with everything out of date. You might want to do that, for
example, for repeatability or because you modified something in the
debugger and want to make sure it gets used.)""";
        } EnableCaching;
        // Symbol: drake::systems::ContextBase::FixContextPointers
        struct /* FixContextPointers */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""((Internal use only) Assuming ``clone`` is a recently-cloned Context
that has yet to have its internal pointers updated, sets those
pointers now. The given map is used to update tracker pointers.)""";
        } FixContextPointers;
        // Symbol: drake::systems::ContextBase::FixInputPort
        struct /* FixInputPort */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""((Advanced) Connects the input port at ``index`` to a
FixedInputPortValue with the given abstract ``value``. Returns a
reference to the allocated FixedInputPortValue that will remain valid
until this input port's value source is replaced or the Context is
destroyed. You may use that reference to modify the input port's value
using the appropriate FixedInputPortValue method, which will ensure
that invalidation notifications are delivered.

This is the most general way to provide a value (type-erased) for an
unconnected input port. Using the ``FixValue()`` method of the input
port to set the value is the preferred workflow due to its additional
sugar.

Note:
    Calling this method on an already connected input port, i.e., an
    input port that has previously been passed into a call to
    DiagramBuilder∷Connect(), causes FixedInputPortValue to override
    any other value present on that port.

Precondition:
    ``index`` selects an existing input port of this Context.)""";
        } FixInputPort;
        // Symbol: drake::systems::ContextBase::FreezeCache
        struct /* FreezeCache */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""((Advanced) Freezes the cache at its current contents, preventing any
further cache updates. When frozen, accessing an out-of-date cache
entry causes an exception to be throw. This is applied recursively to
this Context and all its subcontexts, but *not* to its parent or
siblings so it is most useful when called on the root Context. If the
cache was already frozen this method does nothing but waste a little
time.)""";
        } FreezeCache;
        // Symbol: drake::systems::ContextBase::GetSystemName
        struct /* GetSystemName */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(Returns the local name of the subsystem for which this is the Context.
This is intended primarily for error messages and logging.

See also:
    SystemBase∷GetSystemName() for details.

See also:
    GetSystemPathname() if you want the full name.)""";
        } GetSystemName;
        // Symbol: drake::systems::ContextBase::GetSystemPathname
        struct /* GetSystemPathname */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(Returns the full pathname of the subsystem for which this is the
Context. This is intended primarily for error messages and logging.

See also:
    SystemBase∷GetSystemPathname() for details.)""";
        } GetSystemPathname;
        // Symbol: drake::systems::ContextBase::MaybeGetFixedInputPortValue
        struct /* MaybeGetFixedInputPortValue */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(For input port ``index``, returns a const FixedInputPortValue if the
port is fixed, otherwise nullptr.

Precondition:
    ``index`` selects an existing input port of this Context.)""";
        } MaybeGetFixedInputPortValue;
        // Symbol: drake::systems::ContextBase::MaybeGetMutableFixedInputPortValue
        struct /* MaybeGetMutableFixedInputPortValue */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(For input port ``index``, returns a mutable FixedInputPortValue if the
port is fixed, otherwise nullptr.

Precondition:
    ``index`` selects an existing input port of this Context.)""";
        } MaybeGetMutableFixedInputPortValue;
        // Symbol: drake::systems::ContextBase::NoteAccuracyChanged
        struct /* NoteAccuracyChanged */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(Notifies the local accuracy tracker that the accuracy setting may have
changed.)""";
        } NoteAccuracyChanged;
        // Symbol: drake::systems::ContextBase::NoteAllAbstractParametersChanged
        struct /* NoteAllAbstractParametersChanged */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(Notifies each local abstract parameter tracker that the value of the
parameter it manages may have changed. If there are no abstract
parameters owned by this context, nothing happens. A DiagramContext
does not own any parameters.)""";
        } NoteAllAbstractParametersChanged;
        // Symbol: drake::systems::ContextBase::NoteAllAbstractStateChanged
        struct /* NoteAllAbstractStateChanged */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(Notifies each local abstract state variable tracker that the value of
the abstract state variable it manages may have changed. If there are
no abstract state variables owned by this context, nothing happens. A
DiagramContext does not own any abstract state variables.)""";
        } NoteAllAbstractStateChanged;
        // Symbol: drake::systems::ContextBase::NoteAllContinuousStateChanged
        struct /* NoteAllContinuousStateChanged */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(Notifies the local q, v, and z trackers that each of them may have
changed, likely because someone has asked to modify continuous state
xc.)""";
        } NoteAllContinuousStateChanged;
        // Symbol: drake::systems::ContextBase::NoteAllDiscreteStateChanged
        struct /* NoteAllDiscreteStateChanged */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(Notifies each local discrete state group tracker that the value of the
discrete state group it manages may have changed. If there are no
discrete state groups owned by this context, nothing happens. A
DiagramContext does not own any discrete state groups.)""";
        } NoteAllDiscreteStateChanged;
        // Symbol: drake::systems::ContextBase::NoteAllNumericParametersChanged
        struct /* NoteAllNumericParametersChanged */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(Notifies each local numeric parameter tracker that the value of the
parameter it manages may have changed. If there are no numeric
parameters owned by this context, nothing happens. A DiagramContext
does not own any parameters.)""";
        } NoteAllNumericParametersChanged;
        // Symbol: drake::systems::ContextBase::NoteAllParametersChanged
        struct /* NoteAllParametersChanged */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(Notifies the local numeric and abstract parameter trackers that each
of them may have changed, likely because someone asked to modify all
the parameters.)""";
        } NoteAllParametersChanged;
        // Symbol: drake::systems::ContextBase::NoteAllQChanged
        struct /* NoteAllQChanged */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(Notifies the local q tracker that the q's may have changed.)""";
        } NoteAllQChanged;
        // Symbol: drake::systems::ContextBase::NoteAllStateChanged
        struct /* NoteAllStateChanged */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(Notifies the local continuous, discrete, and abstract state trackers
that each of them may have changed, likely because someone has asked
to modify the whole state x.)""";
        } NoteAllStateChanged;
        // Symbol: drake::systems::ContextBase::NoteAllVChanged
        struct /* NoteAllVChanged */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(Notifies the local v tracker that the v's may have changed.)""";
        } NoteAllVChanged;
        // Symbol: drake::systems::ContextBase::NoteAllVZChanged
        struct /* NoteAllVZChanged */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(Notifies the local v and z trackers that each of them may have
changed, likely because someone has asked to modify just the
first-order state variables in xc.)""";
        } NoteAllVZChanged;
        // Symbol: drake::systems::ContextBase::NoteAllZChanged
        struct /* NoteAllZChanged */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(Notifies the local z tracker that the z's may have changed.)""";
        } NoteAllZChanged;
        // Symbol: drake::systems::ContextBase::NoteTimeChanged
        struct /* NoteTimeChanged */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(Notifies the local time tracker that time may have changed.)""";
        } NoteTimeChanged;
        // Symbol: drake::systems::ContextBase::PropagateBulkChange
        struct /* PropagateBulkChange */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc_3args =
R"""((Internal use only) Applies the given bulk-change notification method
to the given ``context``, and propagates the notification to
subcontexts if this is a DiagramContext.)""";
          // Source: drake/systems/framework/context_base.h
          const char* doc_2args =
R"""((Internal use only) This is a convenience method for invoking the
eponymous static method on ``this`` context (which occurs frequently).)""";
        } PropagateBulkChange;
        // Symbol: drake::systems::ContextBase::PropagateCachingChange
        struct /* PropagateCachingChange */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""((Internal use only) Applies the given caching-change notification
method to ``context``, and propagates the notification to subcontexts
if ``context`` is a DiagramContext. Used, for example, to enable and
disable the cache. The supplied ``context`` is const so depends on the
cache being mutable.)""";
        } PropagateCachingChange;
        // Symbol: drake::systems::ContextBase::SetAllCacheEntriesOutOfDate
        struct /* SetAllCacheEntriesOutOfDate */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""((Debugging) Marks all cache entries out of date, recursively for this
context and all its subcontexts. This forces the next ``Eval()``
request for each cache entry to perform a full calculation rather than
returning the cached one. After that first recalculation, normal
caching behavior resumes (assuming the cache is not disabled). Results
should be identical whether this is called or not, since the caching
system should be maintaining this flag correctly. If they are not, see
the documentation for SetIsCacheDisabled() for suggestions.)""";
        } SetAllCacheEntriesOutOfDate;
        // Symbol: drake::systems::ContextBase::UnfreezeCache
        struct /* UnfreezeCache */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""((Advanced) Unfreezes the cache if it was previously frozen. This is
applied recursively to this Context and all its subcontexts, but *not*
to its parent or siblings. If the cache was not frozen, this does
nothing but waste a little time.)""";
        } UnfreezeCache;
        // Symbol: drake::systems::ContextBase::get_cache
        struct /* get_cache */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(Returns a const reference to this subcontext's cache.)""";
        } get_cache;
        // Symbol: drake::systems::ContextBase::get_dependency_graph
        struct /* get_dependency_graph */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(Returns a const reference to the collection of value trackers within
this subcontext. Together these form the dependency subgraph for the
values in this subcontext, plus edges leading to neighboring trackers.)""";
        } get_dependency_graph;
        // Symbol: drake::systems::ContextBase::get_mutable_cache
        struct /* get_mutable_cache */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""((Advanced) Returns a mutable reference to this subcontext's cache.
Note that this method is const because the cache is always writable.

Warning:
    Writing directly to the cache does not automatically propagate
    invalidations to downstream dependents of a contained cache entry,
    because invalidations would normally have been propagated when the
    cache entry itself went out of date. Cache entries are updated
    automatically when needed via their ``Calc()`` methods; most users
    should not bypass that mechanism by using this method.)""";
        } get_mutable_cache;
        // Symbol: drake::systems::ContextBase::get_mutable_dependency_graph
        struct /* get_mutable_dependency_graph */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(Returns a mutable reference to the dependency graph.)""";
        } get_mutable_dependency_graph;
        // Symbol: drake::systems::ContextBase::get_mutable_tracker
        struct /* get_mutable_tracker */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(Returns a mutable reference to a DependencyTracker in this subcontext.
(You do not need mutable access just to issue value change
notifications.))""";
        } get_mutable_tracker;
        // Symbol: drake::systems::ContextBase::get_system_id
        struct /* get_system_id */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""((Internal) Gets the id of the subsystem that created this context. For
more information, see system_compatibility.)""";
        } get_system_id;
        // Symbol: drake::systems::ContextBase::get_tracker
        struct /* get_tracker */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(Returns a const reference to a DependencyTracker in this subcontext.
Advanced users and internal code can use the returned reference to
issue value change notifications -- mutable access is not required for
that purpose.)""";
        } get_tracker;
        // Symbol: drake::systems::ContextBase::input_port_ticket
        struct /* input_port_ticket */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(Returns the dependency ticket associated with a particular input port.)""";
        } input_port_ticket;
        // Symbol: drake::systems::ContextBase::is_cache_frozen
        struct /* is_cache_frozen */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""((Advanced) Reports whether this Context's cache is currently frozen.
This checks only locally; it is possible that parent, child, or
sibling subcontext caches are in a different state than this one.)""";
        } is_cache_frozen;
        // Symbol: drake::systems::ContextBase::is_root_context
        struct /* is_root_context */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(Returns true if this context has no parent.)""";
        } is_root_context;
        // Symbol: drake::systems::ContextBase::num_input_ports
        struct /* num_input_ports */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(Returns the number of input ports in this context.)""";
        } num_input_ports;
        // Symbol: drake::systems::ContextBase::num_output_ports
        struct /* num_output_ports */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(Returns the number of output ports represented in this context.)""";
        } num_output_ports;
        // Symbol: drake::systems::ContextBase::output_port_ticket
        struct /* output_port_ticket */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(Returns the dependency ticket associated with a particular output
port.)""";
        } output_port_ticket;
        // Symbol: drake::systems::ContextBase::owns_any_variables_or_parameters
        struct /* owns_any_variables_or_parameters */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""((Internal use only) Returns true if this context provides resources
for its own individual state variables or parameters. That means those
variables or parameters were declared by this context's corresponding
System. Currently only leaf systems may declare variables and
parameters; diagram contexts can use this method to check that
invariant.)""";
        } owns_any_variables_or_parameters;
        // Symbol: drake::systems::ContextBase::set_parent
        struct /* set_parent */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""(Declares that ``parent`` is the context of the enclosing Diagram.
Aborts if the parent has already been set or is null.)""";
        } set_parent;
        // Symbol: drake::systems::ContextBase::start_new_change_event
        struct /* start_new_change_event */ {
          // Source: drake/systems/framework/context_base.h
          const char* doc =
R"""((Internal use only) Returns the next change event serial number that
is unique for this entire Context tree, not just this subcontext. This
number is not reset after a Context is copied but continues to count
up.)""";
        } start_new_change_event;
      } ContextBase;
      // Symbol: drake::systems::ContinuousState
      struct /* ContinuousState */ {
        // Source: drake/systems/framework/continuous_state.h
        const char* doc =
R"""(%ContinuousState is a view of, and optionally a container for, all the
continuous state variables ``xc`` of a Drake System. Continuous state
variables are those whose values are defined by differential
equations, so we expect there to be a well-defined time derivative
``xcdot`` ≜ ``d/dt xc``.

The contents of ``xc`` are conceptually partitioned into three groups:

- ``q`` is generalized position
- ``v`` is generalized velocity
- ``z`` is other continuous state

For a Drake LeafSystem these partitions are stored contiguously in
memory in this sequence: xc=[q v z]. But because a Drake System may be
a Diagram composed from subsystems, each with its own continuous state
variables ("substates"), the composite continuous state will not
generally be stored in contiguous memory. In that case the most we can
say is that xc={q,v,z}, that is, it consists of all the q's, v's, and
z's, in some order.

Nevertheless, this ContinuousState class provides a vector view of the
data that groups together all the q partitions, v partitions, and z
partitions. For example, if there are three subsystems (possibly
Diagrams) whose continuous state variables are respectively
xc₁={q₁,v₁,z₁}, xc₂={q₂,v₂,z₂}, and xc₃={q₃,v₃,z₃} the composite xc
includes all the partitions in an undefined order. However, composite
q, v, and z appear ordered as q=[q₁ q₂ q₃], v=[v₁ v₂ v₃], z=[z₁ z₂
z₃]. Note that the element ordering of the composite xc is *not* a
concatenation of the composite subgroups. Do not index elements of the
full state xc unless you know it is the continuous state of a
LeafSystem (a LeafSystem looking at its own Context can depend on
that).

Any of the groups may be empty. However, groups q and v must be either
both present or both empty, because the time derivative ``qdot`` of
the second-order state variables ``q`` must be computable using a
linear mapping ``qdot=N(q)*v``.

The time derivative ``xcdot`` has the identical substructure to
``xc``, with the partitions interpreted as ``qdot``, `vdot`, and
``zdot``. We use identical ContinuousState objects for both.

*Memory ownership*

When a ContinuousState represents the state of a LeafSystem, it always
owns the memory that is used for the state variables and is
responsible for destruction. For a Diagram, ContinuousState can
instead be a *view* of the underlying LeafSystem substates, so that
modifying the Diagram's continuous state affects the LeafSystems
appropriately. In that case, the memory is owned by the underlying
LeafSystems. However, when a ContinuousState object of any structure
is cloned, the resulting object *always* owns all its underlying
memory, which is initialized with a copy of the original state
variable values but is otherwise independent. The cloned object
retains the structure and ordering of the elements and does not
guarantee contiguous storage.

See also:
    DiagramContinuousState for more information.)""";
        // Symbol: drake::systems::ContinuousState::Clone
        struct /* Clone */ {
          // Source: drake/systems/framework/continuous_state.h
          const char* doc =
R"""(Creates a deep copy of this object with the same substructure but with
all data owned by the copy. That is, if the original was a Diagram
continuous state that merely referenced substates, the clone will not
include any references to the original substates and is thus decoupled
from the Context containing the original. The concrete type of the
BasicVector underlying each leaf ContinuousState is preserved. See the
class comments above for more information.)""";
        } Clone;
        // Symbol: drake::systems::ContinuousState::ContinuousState<T>
        struct /* ctor */ {
          // Source: drake/systems/framework/continuous_state.h
          const char* doc_1args_state =
R"""(Constructs a ContinuousState for a system that does not have
second-order structure. The ``q`` and ``v`` partitions are empty; all
of the state ``xc`` is miscellaneous continuous state ``z``.)""";
          // Source: drake/systems/framework/continuous_state.h
          const char* doc_4args_state_num_q_num_v_num_z =
R"""(Constructs a ContinuousState that exposes second-order structure.

Parameter ``state``:
    The source xc of continuous state information.

Parameter ``num_q``:
    The number of position variables q.

Parameter ``num_v``:
    The number of velocity variables v.

Parameter ``num_z``:
    The number of other continuous variables z.

We require that ``num_q ≥ num_v`` and that the sum of the partition
sizes adds up to the size of ``state``.)""";
          // Source: drake/systems/framework/continuous_state.h
          const char* doc_0args =
R"""(Constructs a zero-length ContinuousState.)""";
          // Source: drake/systems/framework/continuous_state.h
          const char* doc_4args_state_q_v_z =
R"""(Constructs a continuous state that exposes second-order structure,
with no particular constraints on the layout.

Precondition:
    The q, v, z are all views into the same storage as ``state``.

Parameter ``state``:
    The entire continuous state.

Parameter ``q``:
    The subset of state that is generalized position.

Parameter ``v``:
    The subset of state that is generalized velocity.

Parameter ``z``:
    The subset of state that is neither position nor velocity.)""";
        } ctor;
        // Symbol: drake::systems::ContinuousState::CopyToVector
        struct /* CopyToVector */ {
          // Source: drake/systems/framework/continuous_state.h
          const char* doc =
R"""(Returns a copy of the entire continuous state vector into an Eigen
vector.)""";
        } CopyToVector;
        // Symbol: drake::systems::ContinuousState::DoClone
        struct /* DoClone */ {
          // Source: drake/systems/framework/continuous_state.h
          const char* doc =
R"""(DiagramContinuousState must override this to maintain the necessary
internal substructure, and to perform a deep copy so that the result
owns all its own data. The default implementation here requires that
the full state is a BasicVector (that is, this is a leaf continuous
state). The BasicVector is cloned to preserve its concrete type and
contents, then the q, v, z Subvectors are created referencing it. The
implementation should not set_system_id on the result, the caller will
set an id on the state after this method returns.)""";
        } DoClone;
        // Symbol: drake::systems::ContinuousState::SetFrom
        struct /* SetFrom */ {
          // Source: drake/systems/framework/continuous_state.h
          const char* doc =
R"""(Copies the values from ``other`` into ``this``, converting the scalar
type as necessary.)""";
        } SetFrom;
        // Symbol: drake::systems::ContinuousState::SetFromVector
        struct /* SetFromVector */ {
          // Source: drake/systems/framework/continuous_state.h
          const char* doc =
R"""(Sets the entire continuous state vector from an Eigen expression.)""";
        } SetFromVector;
        // Symbol: drake::systems::ContinuousState::get_generalized_position
        struct /* get_generalized_position */ {
          // Source: drake/systems/framework/continuous_state.h
          const char* doc =
R"""(Returns a const reference to the subset of the state vector that is
generalized position ``q``. May be zero length.)""";
        } get_generalized_position;
        // Symbol: drake::systems::ContinuousState::get_generalized_velocity
        struct /* get_generalized_velocity */ {
          // Source: drake/systems/framework/continuous_state.h
          const char* doc =
R"""(Returns a const reference to the subset of the continuous state vector
that is generalized velocity ``v``. May be zero length.)""";
        } get_generalized_velocity;
        // Symbol: drake::systems::ContinuousState::get_misc_continuous_state
        struct /* get_misc_continuous_state */ {
          // Source: drake/systems/framework/continuous_state.h
          const char* doc =
R"""(Returns a const reference to the subset of the continuous state vector
that is other continuous state ``z``. May be zero length.)""";
        } get_misc_continuous_state;
        // Symbol: drake::systems::ContinuousState::get_mutable_generalized_position
        struct /* get_mutable_generalized_position */ {
          // Source: drake/systems/framework/continuous_state.h
          const char* doc =
R"""(Returns a mutable reference to the subset of the state vector that is
generalized position ``q``. May be zero length.)""";
        } get_mutable_generalized_position;
        // Symbol: drake::systems::ContinuousState::get_mutable_generalized_velocity
        struct /* get_mutable_generalized_velocity */ {
          // Source: drake/systems/framework/continuous_state.h
          const char* doc =
R"""(Returns a mutable reference to the subset of the continuous state
vector that is generalized velocity ``v``. May be zero length.)""";
        } get_mutable_generalized_velocity;
        // Symbol: drake::systems::ContinuousState::get_mutable_misc_continuous_state
        struct /* get_mutable_misc_continuous_state */ {
          // Source: drake/systems/framework/continuous_state.h
          const char* doc =
R"""(Returns a mutable reference to the subset of the continuous state
vector that is other continuous state ``z``. May be zero length.)""";
        } get_mutable_misc_continuous_state;
        // Symbol: drake::systems::ContinuousState::get_mutable_vector
        struct /* get_mutable_vector */ {
          // Source: drake/systems/framework/continuous_state.h
          const char* doc =
R"""(Returns a mutable reference to the entire continuous state vector.)""";
        } get_mutable_vector;
        // Symbol: drake::systems::ContinuousState::get_system_id
        struct /* get_system_id */ {
          // Source: drake/systems/framework/continuous_state.h
          const char* doc =
R"""((Internal) Gets the id of the subsystem that created this state.)""";
        } get_system_id;
        // Symbol: drake::systems::ContinuousState::get_vector
        struct /* get_vector */ {
          // Source: drake/systems/framework/continuous_state.h
          const char* doc =
R"""(Returns a reference to the entire continuous state vector.)""";
        } get_vector;
        // Symbol: drake::systems::ContinuousState::num_q
        struct /* num_q */ {
          // Source: drake/systems/framework/continuous_state.h
          const char* doc =
R"""(Returns the number of generalized positions q in this state vector.)""";
        } num_q;
        // Symbol: drake::systems::ContinuousState::num_v
        struct /* num_v */ {
          // Source: drake/systems/framework/continuous_state.h
          const char* doc =
R"""(Returns the number of generalized velocities v in this state vector.)""";
        } num_v;
        // Symbol: drake::systems::ContinuousState::num_z
        struct /* num_z */ {
          // Source: drake/systems/framework/continuous_state.h
          const char* doc =
R"""(Returns the number of miscellaneous continuous state variables z in
this state vector.)""";
        } num_z;
        // Symbol: drake::systems::ContinuousState::operator[]
        struct /* operator_array */ {
          // Source: drake/systems/framework/continuous_state.h
          const char* doc = R"""()""";
        } operator_array;
        // Symbol: drake::systems::ContinuousState::set_system_id
        struct /* set_system_id */ {
          // Source: drake/systems/framework/continuous_state.h
          const char* doc =
R"""((Internal) Records the id of the subsystem that created this state.)""";
        } set_system_id;
        // Symbol: drake::systems::ContinuousState::size
        struct /* size */ {
          // Source: drake/systems/framework/continuous_state.h
          const char* doc =
R"""(Returns the size of the entire continuous state vector, which is
necessarily ``num_q + num_v + num_z``.)""";
        } size;
      } ContinuousState;
      // Symbol: drake::systems::ContinuousStateIndex
      struct /* ContinuousStateIndex */ {
        // Source: drake/systems/framework/framework_common.h
        const char* doc =
R"""(Placeholder for future use. Currently, the only valid value is zero.)""";
      } ContinuousStateIndex;
      // Symbol: drake::systems::DependencyGraph
      struct /* DependencyGraph */ {
        // Source: drake/systems/framework/dependency_tracker.h
        const char* doc =
R"""(Represents the portion of the complete dependency graph that is a
subgraph centered on the owning subcontext, plus some edges leading to
other subcontexts. DependencyTracker objects are the nodes of the
graph, and maintain prerequisite/subscriber edges that interconnect
these nodes, and may also connect to nodes contained in dependency
graphs belonging to other subcontexts within the same complete context
tree. Dependencies on the parent (containing DiagramContext) and
children (contained subcontexts) typically arise from exported input
and output ports, while sibling dependencies arise from
output-to-input port connections.

A DependencyGraph creates and owns all the DependencyTracker objects
for a particular subcontext, organized to allow fast access using a
DependencyTicket as an index. Memory addresses of DependencyTracker
objects are stable once allocated, but DependencyTicket numbers are
stable even after a Context has been copied so should be preferred.

Because DependencyTrackers contain pointers, copying a DependencyGraph
must always be done as part of copying an entire Context tree. There
is a copy constructor here but it must be followed by a pointer-fixup
step so is for internal use only.)""";
        // Symbol: drake::systems::DependencyGraph::AppendToTrackerPointerMap
        struct /* AppendToTrackerPointerMap */ {
          // Source: drake/systems/framework/dependency_tracker.h
          const char* doc =
R"""((Internal use only) Create a mapping from the memory addresses of the
trackers contained here to the corresponding ones in ``clone``, which
must have exactly the same number of trackers. The mapping is appended
to the supplied map, which must not be null.)""";
        } AppendToTrackerPointerMap;
        // Symbol: drake::systems::DependencyGraph::CreateNewDependencyTracker
        struct /* CreateNewDependencyTracker */ {
          // Source: drake/systems/framework/dependency_tracker.h
          const char* doc_3args =
R"""(Allocates a new DependencyTracker with an already-known ticket number,
the given description and an optional cache value to be invalidated.
The new tracker has no prerequisites or subscribers yet. This may
leave gaps in the node numbering. Use has_tracker() if you need to
know whether there is a tracker for a particular ticket. We promise
that the returned DependencyTracker's location in memory will remain
unchanged once created in a particular Context, even as more trackers
are added. The DependencyTicket retains its meaning even after cloning
the Context, although of course the tracker has a new address in the
clone.

Precondition:
    The given ticket must be valid.

Precondition:
    No DependencyTracker is already using the given ticket.)""";
          // Source: drake/systems/framework/dependency_tracker.h
          const char* doc_2args =
R"""(Assigns a new ticket number and then allocates a new DependencyTracker
that can be accessed with that ticket. You may obtain the assigned
ticket from the returned tracker. See the other signature for details.)""";
        } CreateNewDependencyTracker;
        // Symbol: drake::systems::DependencyGraph::DependencyGraph
        struct /* ctor */ {
          // Source: drake/systems/framework/dependency_tracker.h
          const char* doc_move =
R"""(@name Does not allow move or assignment; copy constructor limited. The
copy constructor does not copy internal pointers so requires special
handling.)""";
          // Source: drake/systems/framework/dependency_tracker.h
          const char* doc =
R"""(Constructor creates an empty graph referencing the system pathname
service of its owning subcontext. The supplied pointer must not be
null.)""";
          // Source: drake/systems/framework/dependency_tracker.h
          const char* doc_copy =
R"""((Internal use only) Copy constructor partially duplicates the source
DependencyGraph object, with identical structure to the source but
with all internal pointers set to null, and all counters and
statistics set to their default-constructed values. Pointers must be
set properly using RepairTrackerPointers() once all the old-to-new
pointer mappings have been determined *for the whole Context*, not
just the containing subcontext. This should only be invoked by Context
code as part of copying an entire Context tree.

See also:
    AppendToTrackerPointerMap(), RepairTrackerPointers())""";
        } ctor;
        // Symbol: drake::systems::DependencyGraph::RepairTrackerPointers
        struct /* RepairTrackerPointers */ {
          // Source: drake/systems/framework/dependency_tracker.h
          const char* doc =
R"""((Internal use only) Assumes ``this`` DependencyGraph is a recent clone
whose trackers do not yet contain subscriber and prerequisite pointers
and sets the local pointers to point to the ``source``-corresponding
trackers in the new owning context, the appropriate cache entry values
in the new cache, and to the system name providing service of the new
owning Context for logging and error reporting. The supplied map
should map source pointers to their corresponding trackers. It is a
fatal error if any old pointer we encounter is not present in the map;
that would indicate a bug in the Context cloning code.)""";
        } RepairTrackerPointers;
        // Symbol: drake::systems::DependencyGraph::get_mutable_tracker
        struct /* get_mutable_tracker */ {
          // Source: drake/systems/framework/dependency_tracker.h
          const char* doc =
R"""(Returns a mutable DependencyTracker given a ticket. This is very fast.
Behavior is undefined if the ticket is out of range
[0..num_trackers()-1].)""";
        } get_mutable_tracker;
        // Symbol: drake::systems::DependencyGraph::get_tracker
        struct /* get_tracker */ {
          // Source: drake/systems/framework/dependency_tracker.h
          const char* doc =
R"""(Returns a const DependencyTracker given a ticket. This is very fast.
Behavior is undefined if the ticket is out of range
[0..num_trackers()-1].)""";
        } get_tracker;
        // Symbol: drake::systems::DependencyGraph::has_tracker
        struct /* has_tracker */ {
          // Source: drake/systems/framework/dependency_tracker.h
          const char* doc =
R"""(Returns true if there is a DependencyTracker in this graph that has
the given ticket number.)""";
        } has_tracker;
        // Symbol: drake::systems::DependencyGraph::trackers_size
        struct /* trackers_size */ {
          // Source: drake/systems/framework/dependency_tracker.h
          const char* doc =
R"""(Returns the current size of the DependencyTracker container, providing
for DependencyTicket numbers from ``0..trackers_size()-1``. Note that
it is possible to have empty slots in the container. Use has_tracker()
to determine if there is a tracker associated with a particular
ticket.)""";
        } trackers_size;
      } DependencyGraph;
      // Symbol: drake::systems::DependencyTicket
      struct /* DependencyTicket */ {
        // Source: drake/systems/framework/framework_common.h
        const char* doc =
R"""(Identifies a particular source value or computation for purposes of
declaring and managing dependencies. Unique only within a given
subsystem and its corresponding subcontext.)""";
      } DependencyTicket;
      // Symbol: drake::systems::DependencyTracker
      struct /* DependencyTracker */ {
        // Source: drake/systems/framework/dependency_tracker.h
        const char* doc = R"""()""";
        // Symbol: drake::systems::DependencyTracker::AddDownstreamSubscriber
        struct /* AddDownstreamSubscriber */ {
          // Source: drake/systems/framework/dependency_tracker.h
          const char* doc =
R"""(Adds a downstream subscriber to ``this`` DependencyTracker, which will
keep a pointer to the subscribing tracker. The subscriber will be
notified whenever this DependencyTracker is notified of a value or
prerequisite change.

Precondition:
    The subscriber has already recorded its dependency on this tracker
    in its prerequisite list.)""";
        } AddDownstreamSubscriber;
        // Symbol: drake::systems::DependencyTracker::DependencyTracker
        struct /* ctor */ {
          // Source: drake/systems/framework/dependency_tracker.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::systems::DependencyTracker::GetPathDescription
        struct /* GetPathDescription */ {
          // Source: drake/systems/framework/dependency_tracker.h
          const char* doc =
R"""(Returns the description, preceded by the full pathname of the
subsystem associated with the owning subcontext.)""";
        } GetPathDescription;
        // Symbol: drake::systems::DependencyTracker::HasPrerequisite
        struct /* HasPrerequisite */ {
          // Source: drake/systems/framework/dependency_tracker.h
          const char* doc =
R"""(Returns ``True`` if this tracker has already subscribed to
``prerequisite``. This is slow and should not be used in
performance-sensitive code.)""";
        } HasPrerequisite;
        // Symbol: drake::systems::DependencyTracker::HasSubscriber
        struct /* HasSubscriber */ {
          // Source: drake/systems/framework/dependency_tracker.h
          const char* doc =
R"""(Returns ``True`` if ``subscriber`` is one of this tracker's
subscribers. This is slow and should not be used in
performance-sensitive code.)""";
        } HasSubscriber;
        // Symbol: drake::systems::DependencyTracker::NoteValueChange
        struct /* NoteValueChange */ {
          // Source: drake/systems/framework/dependency_tracker.h
          const char* doc =
R"""(Notifies ``this`` DependencyTracker that its managed value was
directly modified or made available for mutable access. That is, this
is the *initiating* event of a value modification. All of our
downstream subscribers are notified but the associated cache entry (if
any) is *not* invalidated (see below for why). A unique, positive
``change_event`` should have been obtained from the owning Context and
supplied here.

Why don't we invalidate the cache entry? Recall that this method is
for *initiating* a change event, meaning that the quantity that this
tracker tracks is *initiating* an invalidation sweep, as opposed to
just reacting to prerequisite changes. Normally cache entries become
invalid because their prerequisites change; they are not usually the
first step in an invalidation sweep. So it is unusual for
NoteValueChange() to be called on a cache entry's dependency tracker.
But if it is called, that is likely to mean the cache entry was just
given a new value, and is therefore *valid*; invalidating it now would
be an error.)""";
        } NoteValueChange;
        // Symbol: drake::systems::DependencyTracker::PointerMap
        struct /* PointerMap */ {
          // Source: drake/systems/framework/dependency_tracker.h
          const char* doc = R"""((Internal use only))""";
        } PointerMap;
        // Symbol: drake::systems::DependencyTracker::RemoveDownstreamSubscriber
        struct /* RemoveDownstreamSubscriber */ {
          // Source: drake/systems/framework/dependency_tracker.h
          const char* doc =
R"""(Removes a downstream subscriber from ``this`` DependencyTracker.

Precondition:
    The subscriber has already removed the dependency on this tracker
    from its prerequisite list.)""";
        } RemoveDownstreamSubscriber;
        // Symbol: drake::systems::DependencyTracker::SubscribeToPrerequisite
        struct /* SubscribeToPrerequisite */ {
          // Source: drake/systems/framework/dependency_tracker.h
          const char* doc =
R"""(Subscribes ``this`` tracker to an upstream prerequisite's tracker. The
upstream tracker will keep a const pointer back to ``this`` tracker in
its subscriber list, and ``this`` tracker will keep a pointer to the
prerequisite tracker in its prerequisites list.)""";
        } SubscribeToPrerequisite;
        // Symbol: drake::systems::DependencyTracker::ThrowIfBadDependencyTracker
        struct /* ThrowIfBadDependencyTracker */ {
          // Source: drake/systems/framework/dependency_tracker.h
          const char* doc =
R"""(Throws an RuntimeError if there is something clearly wrong with this
DependencyTracker object. If the owning subcontext is known, provide a
pointer to it here and we'll check that this tracker agrees. If you
know which cache entry is supposed to be associated with this tracker,
supply a pointer to that and we'll check it (trackers that are not
associated with a real cache entry are still associated with the
CacheEntryValue∷dummy()). In addition we check for other internal
inconsistencies.

Raises:
    RuntimeError for anything that goes wrong, with an appropriate
    explanatory message.)""";
        } ThrowIfBadDependencyTracker;
        // Symbol: drake::systems::DependencyTracker::UnsubscribeFromPrerequisite
        struct /* UnsubscribeFromPrerequisite */ {
          // Source: drake/systems/framework/dependency_tracker.h
          const char* doc =
R"""(Unsubscribes ``this`` tracker from an upstream prerequisite tracker to
which we previously subscribed. Both the prerequisite list in ``this``
tracker and the subscriber list in ``prerequisite`` are modified.

Precondition:
    The supplied pointer must not be null.

Precondition:
    This tracker must already be subscribed to the given
    ``prerequisite``.)""";
        } UnsubscribeFromPrerequisite;
        // Symbol: drake::systems::DependencyTracker::cache_entry_value
        struct /* cache_entry_value */ {
          // Source: drake/systems/framework/dependency_tracker.h
          const char* doc =
R"""((Internal use only) Returns a pointer to the CacheEntryValue if this
tracker is a cache entry tracker, otherwise nullptr.)""";
        } cache_entry_value;
        // Symbol: drake::systems::DependencyTracker::description
        struct /* description */ {
          // Source: drake/systems/framework/dependency_tracker.h
          const char* doc =
R"""(Returns the human-readable description for this tracker.)""";
        } description;
        // Symbol: drake::systems::DependencyTracker::notifications_are_suppressed
        struct /* notifications_are_suppressed */ {
          // Source: drake/systems/framework/dependency_tracker.h
          const char* doc =
R"""(Returns true if suppress_notifications() has been called on this
tracker.)""";
        } notifications_are_suppressed;
        // Symbol: drake::systems::DependencyTracker::num_ignored_notifications
        struct /* num_ignored_notifications */ {
          // Source: drake/systems/framework/dependency_tracker.h
          const char* doc =
R"""(How many times did we receive a repeat notification for the same
change event that we ignored?)""";
        } num_ignored_notifications;
        // Symbol: drake::systems::DependencyTracker::num_notifications_received
        struct /* num_notifications_received */ {
          // Source: drake/systems/framework/dependency_tracker.h
          const char* doc =
R"""(What is the total number of notifications received by this tracker?
This is the sum of managed-value change event notifications and
prerequisite change notifications received.)""";
        } num_notifications_received;
        // Symbol: drake::systems::DependencyTracker::num_notifications_sent
        struct /* num_notifications_sent */ {
          // Source: drake/systems/framework/dependency_tracker.h
          const char* doc =
R"""(What is the total number of notifications sent to downstream
subscribers by this trackers?)""";
        } num_notifications_sent;
        // Symbol: drake::systems::DependencyTracker::num_prerequisite_change_events
        struct /* num_prerequisite_change_events */ {
          // Source: drake/systems/framework/dependency_tracker.h
          const char* doc =
R"""(How many times was this tracker notified of a change to one of its
value's prerequisites?)""";
        } num_prerequisite_change_events;
        // Symbol: drake::systems::DependencyTracker::num_prerequisites
        struct /* num_prerequisites */ {
          // Source: drake/systems/framework/dependency_tracker.h
          const char* doc =
R"""(Returns the total number of "depends-on" edges emanating from ``this``
tracker, pointing to its upstream prerequisites.)""";
        } num_prerequisites;
        // Symbol: drake::systems::DependencyTracker::num_subscribers
        struct /* num_subscribers */ {
          // Source: drake/systems/framework/dependency_tracker.h
          const char* doc =
R"""(Returns the total number of "is-prerequisite-of" edges emanating from
``this`` tracker, pointing to its downstream subscribers.)""";
        } num_subscribers;
        // Symbol: drake::systems::DependencyTracker::num_value_change_events
        struct /* num_value_change_events */ {
          // Source: drake/systems/framework/dependency_tracker.h
          const char* doc =
R"""(How many times was this tracker notified of a change event for a
direct change to a value it tracks?)""";
        } num_value_change_events;
        // Symbol: drake::systems::DependencyTracker::prerequisites
        struct /* prerequisites */ {
          // Source: drake/systems/framework/dependency_tracker.h
          const char* doc =
R"""(Returns a reference to the prerequisite trackers.)""";
        } prerequisites;
        // Symbol: drake::systems::DependencyTracker::set_cache_entry_value
        struct /* set_cache_entry_value */ {
          // Source: drake/systems/framework/dependency_tracker.h
          const char* doc =
R"""((Internal use only) Sets the cache entry value to be marked
out-of-date when this tracker's prerequisites change.

Precondition:
    The supplied cache entry value is non-null.

Precondition:
    No cache entry value has previously been assigned.)""";
        } set_cache_entry_value;
        // Symbol: drake::systems::DependencyTracker::subscribers
        struct /* subscribers */ {
          // Source: drake/systems/framework/dependency_tracker.h
          const char* doc =
R"""(Returns a reference to the subscribing trackers.)""";
        } subscribers;
        // Symbol: drake::systems::DependencyTracker::suppress_notifications
        struct /* suppress_notifications */ {
          // Source: drake/systems/framework/dependency_tracker.h
          const char* doc =
R"""((Internal use only) Instructs this tracker to suppress notifications
to downstream subscribers. This can be used by the framework during
Context allocation to disable built-in trackers that have nothing to
track. For example, if there are no q's we can improve performance and
avoid spurious notifications to q-subscribers like
configuration_tracker by disabling q's tracker.)""";
        } suppress_notifications;
        // Symbol: drake::systems::DependencyTracker::ticket
        struct /* ticket */ {
          // Source: drake/systems/framework/dependency_tracker.h
          const char* doc =
R"""(Returns the DependencyTicket for this DependencyTracker in its
containing DependencyGraph. The ticket is unique within the containing
subcontext.)""";
        } ticket;
      } DependencyTracker;
      // Symbol: drake::systems::Diagram
      struct /* Diagram */ {
        // Source: drake/systems/framework/diagram.h
        const char* doc =
R"""(Diagram is a System composed of one or more constituent Systems,
arranged in a directed graph where the vertices are the constituent
Systems themselves, and the edges connect the output of one
constituent System to the input of another. To construct a Diagram,
use a DiagramBuilder.

Each System in the Diagram must have a unique, non-empty name.)""";
        // Symbol: drake::systems::Diagram::Accept
        struct /* Accept */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc =
R"""(Implements a visitor pattern.

See also:
    SystemVisitor<T>.)""";
        } Accept;
        // Symbol: drake::systems::Diagram::AddTriggeredWitnessFunctionToCompositeEventCollection
        struct /* AddTriggeredWitnessFunctionToCompositeEventCollection */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc =
R"""(For the subsystem associated with ``witness_func``, gets its mutable
sub composite event collection from ``events``, and passes it to
``witness_func`'s AddEventToCollection method. This method also
modifies `event`` by updating the pointers to "diagram" continuous
state to point to the ContinuousState pointers for the associated
subsystem instead. Aborts if the subsystem is not part of this
Diagram.)""";
        } AddTriggeredWitnessFunctionToCompositeEventCollection;
        // Symbol: drake::systems::Diagram::AllocateDiscreteVariables
        struct /* AllocateDiscreteVariables */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc = R"""()""";
        } AllocateDiscreteVariables;
        // Symbol: drake::systems::Diagram::AllocateForcedDiscreteUpdateEventCollection
        struct /* AllocateForcedDiscreteUpdateEventCollection */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc = R"""()""";
        } AllocateForcedDiscreteUpdateEventCollection;
        // Symbol: drake::systems::Diagram::AllocateForcedPublishEventCollection
        struct /* AllocateForcedPublishEventCollection */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc = R"""()""";
        } AllocateForcedPublishEventCollection;
        // Symbol: drake::systems::Diagram::AllocateForcedUnrestrictedUpdateEventCollection
        struct /* AllocateForcedUnrestrictedUpdateEventCollection */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc = R"""()""";
        } AllocateForcedUnrestrictedUpdateEventCollection;
        // Symbol: drake::systems::Diagram::AllocateTimeDerivatives
        struct /* AllocateTimeDerivatives */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc = R"""()""";
        } AllocateTimeDerivatives;
        // Symbol: drake::systems::Diagram::AreConnected
        struct /* AreConnected */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc =
R"""(Reports if the indicated ``output`` is connected to the ``input``
port.

Precondition:
    the ports belong to systems that are direct children of this
    diagram.)""";
        } AreConnected;
        // Symbol: drake::systems::Diagram::Diagram<type-parameter-0-0>
        struct /* ctor */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc_1args_constDiagram =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
          // Source: drake/systems/framework/diagram.h
          const char* doc_0args =
R"""(Constructs an uninitialized Diagram. Subclasses that use this
constructor are obligated to call DiagramBuilder∷BuildInto(this).
Provides scalar- type conversion support only if every contained
subsystem provides the same support.)""";
          // Source: drake/systems/framework/diagram.h
          const char* doc_1args_converter =
R"""((Advanced) Constructs an uninitialized Diagram. Subclasses that use
this constructor are obligated to call DiagramBuilder∷BuildInto(this).

Declares scalar-type conversion support using ``converter``. Support
for a given pair of types ``T, U`` to convert from and to will be
enabled only if every contained subsystem supports that pair.

See system_scalar_conversion for detailed background and examples
related to scalar-type conversion support.)""";
          // Source: drake/systems/framework/diagram.h
          const char* doc_2args_SystemScalarConverter_constDiagram =
R"""((Advanced) Scalar-converting constructor, for used by derived classes
that are performing a conversion and also need to supply a
``converter`` that preserves subtypes for additional conversions.

See system_scalar_conversion for detailed background and examples
related to scalar-type conversion support.)""";
        } ctor;
        // Symbol: drake::systems::Diagram::DoCalcNextUpdateTime
        struct /* DoCalcNextUpdateTime */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc =
R"""(Computes the next update time based on the configured actions, for
scalar types that are arithmetic, or aborts for scalar types that are
not arithmetic.)""";
        } DoCalcNextUpdateTime;
        // Symbol: drake::systems::Diagram::DoCalcWitnessValue
        struct /* DoCalcWitnessValue */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc =
R"""(For the subsystem associated with ``witness_func``, gets its
subcontext from ``context``, passes the subcontext to
``witness_func``' Evaluate method and returns the result. Aborts if
the subsystem is not part of this Diagram.)""";
        } DoCalcWitnessValue;
        // Symbol: drake::systems::Diagram::DoGetGraphvizFragment
        struct /* DoGetGraphvizFragment */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc =
R"""(The NVI implementation of SystemBase∷GetGraphvizFragment() for
subclasses to override if desired. The default behavior should be
sufficient in most cases.)""";
        } DoGetGraphvizFragment;
        // Symbol: drake::systems::Diagram::DoGetMutableTargetSystemCompositeEventCollection
        struct /* DoGetMutableTargetSystemCompositeEventCollection */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc =
R"""(Returns a pointer to mutable composite event collection if
``target_system`` is a subsystem of this, nullptr is returned
otherwise.)""";
        } DoGetMutableTargetSystemCompositeEventCollection;
        // Symbol: drake::systems::Diagram::DoGetMutableTargetSystemState
        struct /* DoGetMutableTargetSystemState */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc =
R"""(Returns a pointer to mutable state if ``target_system`` is a subsystem
of this, nullptr is returned otherwise.)""";
        } DoGetMutableTargetSystemState;
        // Symbol: drake::systems::Diagram::DoGetTargetSystemCompositeEventCollection
        struct /* DoGetTargetSystemCompositeEventCollection */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc =
R"""(Returns a pointer to const composite event collection if
``target_system`` is a subsystem of this, nullptr is returned
otherwise.)""";
        } DoGetTargetSystemCompositeEventCollection;
        // Symbol: drake::systems::Diagram::DoGetTargetSystemContext
        struct /* DoGetTargetSystemContext */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc =
R"""(Returns a pointer to const context if ``target_system`` is a subsystem
of this, nullptr is returned otherwise.)""";
        } DoGetTargetSystemContext;
        // Symbol: drake::systems::Diagram::DoGetTargetSystemContinuousState
        struct /* DoGetTargetSystemContinuousState */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc =
R"""(Returns a pointer to const state if ``target_system`` is a subsystem
of this, nullptr is returned otherwise.)""";
        } DoGetTargetSystemContinuousState;
        // Symbol: drake::systems::Diagram::DoGetTargetSystemState
        struct /* DoGetTargetSystemState */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc =
R"""(Returns a pointer to const state if ``target_system`` is a subsystem
of this, nullptr is returned otherwise.)""";
        } DoGetTargetSystemState;
        // Symbol: drake::systems::Diagram::DoGetWitnessFunctions
        struct /* DoGetWitnessFunctions */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc =
R"""(Provides witness functions of subsystems that are active at the
beginning of a continuous time interval. The vector of witness
functions is not ordered in a particular manner.)""";
        } DoGetWitnessFunctions;
        // Symbol: drake::systems::Diagram::DoMapQDotToVelocity
        struct /* DoMapQDotToVelocity */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc =
R"""(The ``generalized_velocity`` vector must have the same size and
ordering as the generalized velocity in the ContinuousState that this
Diagram reserves in its context.)""";
        } DoMapQDotToVelocity;
        // Symbol: drake::systems::Diagram::DoMapVelocityToQDot
        struct /* DoMapVelocityToQDot */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc =
R"""(The ``generalized_velocity`` vector must have the same size and
ordering as the generalized velocity in the ContinuousState that this
Diagram reserves in its context.)""";
        } DoMapVelocityToQDot;
        // Symbol: drake::systems::Diagram::GetDirectFeedthroughs
        struct /* GetDirectFeedthroughs */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc = R"""()""";
        } GetDirectFeedthroughs;
        // Symbol: drake::systems::Diagram::GetDowncastSubsystemByName
        struct /* GetDowncastSubsystemByName */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc_was_unable_to_choose_unambiguous_names =
R"""(Retrieves a const reference to the subsystem with name ``name``
returned by get_name(), downcast to the type provided as a template
argument.

Template parameter ``MySystem``:
    is the downcast type, e.g., drake∷systems∷Adder

Raises:
    RuntimeError if a match cannot be found.

See also:
    GetSubsystemByName()

See also:
    System<T>∷get_name())""";
        } GetDowncastSubsystemByName;
        // Symbol: drake::systems::Diagram::GetInputPortLocators
        struct /* GetInputPortLocators */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc =
R"""(Returns the collection of "locators" for the subsystem input ports
that were exported or connected to the ``port_index`` input port for
the Diagram.)""";
        } GetInputPortLocators;
        // Symbol: drake::systems::Diagram::GetMutableSubsystemCompositeEventCollection
        struct /* GetMutableSubsystemCompositeEventCollection */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc =
R"""(Returns the mutable subsystem composite event collection that
corresponds to ``subsystem``. Aborts if ``subsystem`` is not a
subsystem of this diagram.)""";
        } GetMutableSubsystemCompositeEventCollection;
        // Symbol: drake::systems::Diagram::GetMutableSubsystemState
        struct /* GetMutableSubsystemState */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc_2args_subsystem_context =
R"""(Retrieves the state for a particular subsystem from the context for
the entire diagram. Invalidates all entries in that subsystem's cache
that depend on State. Aborts if ``subsystem`` is not actually a
subsystem of this diagram.)""";
          // Source: drake/systems/framework/diagram.h
          const char* doc_2args_subsystem_state =
R"""(Retrieves the state for a particular subsystem from the ``state`` for
the entire diagram. Aborts if ``subsystem`` is not actually a
subsystem of this diagram.)""";
        } GetMutableSubsystemState;
        // Symbol: drake::systems::Diagram::GetSubsystemByName
        struct /* GetSubsystemByName */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc =
R"""(Retrieves a const reference to the subsystem with name ``name``
returned by get_name().

Raises:
    RuntimeError if a match cannot be found.

See also:
    GetDowncastSubsystemByName()

See also:
    System<T>∷get_name())""";
        } GetSubsystemByName;
        // Symbol: drake::systems::Diagram::GetSubsystemCompositeEventCollection
        struct /* GetSubsystemCompositeEventCollection */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc =
R"""(Returns the const subsystem composite event collection from ``events``
that corresponds to ``subsystem``. Aborts if ``subsystem`` is not a
subsystem of this diagram.)""";
        } GetSubsystemCompositeEventCollection;
        // Symbol: drake::systems::Diagram::GetSubsystemDerivatives
        struct /* GetSubsystemDerivatives */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc =
R"""(Retrieves the state derivatives for a particular subsystem from the
derivatives for the entire diagram. Aborts if ``subsystem`` is not
actually a subsystem of this diagram. Returns a 0-length
ContinuousState if ``subsystem`` has none.)""";
        } GetSubsystemDerivatives;
        // Symbol: drake::systems::Diagram::GetSubsystemDiscreteValues
        struct /* GetSubsystemDiscreteValues */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc =
R"""(Retrieves the discrete state values for a particular subsystem from
the discrete values for the entire diagram. Aborts if ``subsystem`` is
not actually a subsystem of this diagram. Returns an empty
DiscreteValues if ``subsystem`` has none.)""";
        } GetSubsystemDiscreteValues;
        // Symbol: drake::systems::Diagram::GetSubsystemState
        struct /* GetSubsystemState */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc =
R"""(Retrieves the state for a particular subsystem from the ``state`` for
the entire diagram. Aborts if ``subsystem`` is not actually a
subsystem of this diagram.)""";
        } GetSubsystemState;
        // Symbol: drake::systems::Diagram::GetSystemIndexOrAbort
        struct /* GetSystemIndexOrAbort */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc =
R"""(Returns the index of the given ``sys`` in this diagram, or aborts if
``sys`` is not a member of the diagram.)""";
        } GetSystemIndexOrAbort;
        // Symbol: drake::systems::Diagram::GetSystems
        struct /* GetSystems */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc = R"""(Returns the list of contained Systems.)""";
        } GetSystems;
        // Symbol: drake::systems::Diagram::GetUnsupportedScalarConversionMessage
        struct /* GetUnsupportedScalarConversionMessage */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc = R"""()""";
        } GetUnsupportedScalarConversionMessage;
        // Symbol: drake::systems::Diagram::GraphvizFragment
        struct /* GraphvizFragment */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc = R"""()""";
        } GraphvizFragment;
        // Symbol: drake::systems::Diagram::GraphvizFragmentParams
        struct /* GraphvizFragmentParams */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc = R"""()""";
        } GraphvizFragmentParams;
        // Symbol: drake::systems::Diagram::HasSubsystemNamed
        struct /* HasSubsystemNamed */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc =
R"""(Returns true iff this contains a subsystem with the given name.

See also:
    GetSubsystemByName())""";
        } HasSubsystemNamed;
        // Symbol: drake::systems::Diagram::InputPortLocator
        struct /* InputPortLocator */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc =
R"""(A designator for a "system + input port" pair, to uniquely refer to
some input port on one of this diagram's subsystems.)""";
        } InputPortLocator;
        // Symbol: drake::systems::Diagram::OutputPortLocator
        struct /* OutputPortLocator */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc =
R"""(A designator for a "system + output port" pair, to uniquely refer to
some output port on one of this diagram's subsystems.)""";
        } OutputPortLocator;
        // Symbol: drake::systems::Diagram::SetDefaultParameters
        struct /* SetDefaultParameters */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc = R"""()""";
        } SetDefaultParameters;
        // Symbol: drake::systems::Diagram::SetDefaultState
        struct /* SetDefaultState */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc = R"""()""";
        } SetDefaultState;
        // Symbol: drake::systems::Diagram::SetRandomParameters
        struct /* SetRandomParameters */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc = R"""()""";
        } SetRandomParameters;
        // Symbol: drake::systems::Diagram::SetRandomState
        struct /* SetRandomState */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc = R"""()""";
        } SetRandomState;
        // Symbol: drake::systems::Diagram::connection_map
        struct /* connection_map */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc =
R"""(Returns a reference to the map of connections between Systems.)""";
        } connection_map;
        // Symbol: drake::systems::Diagram::get_output_port_locator
        struct /* get_output_port_locator */ {
          // Source: drake/systems/framework/diagram.h
          const char* doc =
R"""(Returns the "locator" for the subsystem output port that was exported
as the ``port_index`` output port for the Diagram.)""";
        } get_output_port_locator;
      } Diagram;
      // Symbol: drake::systems::DiagramBuilder
      struct /* DiagramBuilder */ {
        // Source: drake/systems/framework/diagram_builder.h
        const char* doc =
R"""(DiagramBuilder is a factory class for Diagram.

It is single use: after calling Build or BuildInto, DiagramBuilder
gives up ownership of the constituent systems, and should therefore be
discarded; all member functions will throw an exception after this
point.

When a Diagram (or DiagramBuilder) that owns systems is destroyed, the
systems will be destroyed in the reverse of the order they were added.

A system must be added to the DiagramBuilder with AddSystem or
AddNamedSystem before it can be wired up in any way. Every system must
have a unique, non-empty name.

Building large Diagrams
-----------------------

When building large Diagrams with many added systems and input-output
port connections, the runtime performance of DiagramBuilder∷Build()
might become relevant.

As part of its correctness checks, the DiagramBuilder∷Build() function
performs a graph search of the diagram's dependencies. In the graph,
the nodes are the child systems that have been added to the diagram,
and the edges are the diagram connections from one child's output port
to another child's input port(s). The builder must confirm that the
graph is acyclic; a cycle would imply an infinite loop in an output
calculation function. With a large graph, this check can be
computationally expensive. To speed it up, ensure that your output
ports do not gratuitously depend on irrelevant input ports.

The dependencies are supplied via the ``prerequisites_of_calc``
argument to DeclareOutputPort family of functions. If the default
value is used (i.e., when no prerequisites are provided), the default
is to assume the output port value is dependent on all possible
sources.

Refer to the DeclareLeafOutputPort_feedthrough "Direct feedthrough"
documentation for additional details and examples. In particular, the
SystemBase∷all_sources_except_input_ports_ticket() is a convenient
shortcut for outputs that do not depend on any inputs.)""";
        // Symbol: drake::systems::DiagramBuilder::AddNamedSystem
        struct /* AddNamedSystem */ {
          // Source: drake/systems/framework/diagram_builder.h
          const char* doc =
R"""(Takes ownership of ``system``, sets its name to ``name``, and adds it
to the builder. Returns a bare pointer to the System, which will
remain valid for the lifetime of the Diagram built by this builder.

Warning:
    a System may only be added to at most one DiagramBuilder. Multiple
    Diagram instances cannot share the same System.)""";
        } AddNamedSystem;
        // Symbol: drake::systems::DiagramBuilder::AddSystem
        struct /* AddSystem */ {
          // Source: drake/systems/framework/diagram_builder.h
          const char* doc =
R"""(Takes ownership of ``system`` and adds it to the builder. Returns a
bare pointer to the System, which will remain valid for the lifetime
of the Diagram built by this builder.

If the system's name is unset, sets it to System∷GetMemoryObjectName()
as a default in order to have unique names within the diagram.

Warning:
    a System may only be added to at most one DiagramBuilder. Multiple
    Diagram instances cannot share the same System.)""";
        } AddSystem;
        // Symbol: drake::systems::DiagramBuilder::Build
        struct /* Build */ {
          // Source: drake/systems/framework/diagram_builder.h
          const char* doc =
R"""(Builds the Diagram that has been described by the calls to Connect,
ExportInput, and ExportOutput.

Raises:
    RuntimeError if the graph is not buildable.

See DiagramBuilder_feedthrough "Building large Diagrams" for tips on
improving runtime performance of this function.)""";
        } Build;
        // Symbol: drake::systems::DiagramBuilder::BuildInto
        struct /* BuildInto */ {
          // Source: drake/systems/framework/diagram_builder.h
          const char* doc =
R"""(Configures ``target`` to have the topology that has been described by
the calls to Connect, ExportInput, and ExportOutput.

Raises:
    RuntimeError if the graph is not buildable.

Only Diagram subclasses should call this method. The target must not
already be initialized.)""";
        } BuildInto;
        // Symbol: drake::systems::DiagramBuilder::Cascade
        struct /* Cascade */ {
          // Source: drake/systems/framework/diagram_builder.h
          const char* doc =
R"""(Cascades ``src`` and ``dest``. The sole input port on the ``dest``
system is connected to sole output port on the ``src`` system.

Raises:
    RuntimeError if the sole-port precondition is not met (i.e., if
    ``dest`` has no input ports, or ``dest`` has more than one input
    port, or ``src`` has no output ports, or ``src`` has more than one
    output port).)""";
        } Cascade;
        // Symbol: drake::systems::DiagramBuilder::Connect
        struct /* Connect */ {
          // Source: drake/systems/framework/diagram_builder.h
          const char* doc =
R"""(Declares that input port ``dest`` is connected to output port ``src``.

Note:
    The connection created between ``src`` and ``dest`` via a call to
    this method can be effectively overridden by any subsequent call
    to InputPort∷FixValue(). That is, calling InputPort∷FixValue() on
    an already connected input port causes the resultant
    FixedInputPortValue to override any other value present on that
    port.)""";
        } Connect;
        // Symbol: drake::systems::DiagramBuilder::ConnectInput
        struct /* ConnectInput */ {
          // Source: drake/systems/framework/diagram_builder.h
          const char* doc_2args_diagram_port_name_input =
R"""(Connects an input to the entire Diagram, indicated by
``diagram_port_name``, to the given ``input`` port of a constituent
system.

Precondition:
    The Diagram input indicated by ``diagram_port_name`` must have
    been previously built via ExportInput().

Postcondition:
    ``input`` is connected to the indicated Diagram input port.)""";
          // Source: drake/systems/framework/diagram_builder.h
          const char* doc_2args_diagram_port_index_input =
R"""(Connects an input to the entire Diagram, indicated by
``diagram_port_index``, to the given ``input`` port of a constituent
system.

Precondition:
    The Diagram input indicated by ``diagram_port_index`` must have
    been previously built via ExportInput().

Postcondition:
    ``input`` is connected to the indicated Diagram input port.)""";
        } ConnectInput;
        // Symbol: drake::systems::DiagramBuilder::ConnectToSame
        struct /* ConnectToSame */ {
          // Source: drake/systems/framework/diagram_builder.h
          const char* doc =
R"""(Connects ``dest`` to the same source as ``exemplar`` is connected to.

If ``exemplar`` was connected to an output port, then ``dest`` is
connected to that same output. Or, if ``exemplar`` was exported as an
input of this diagram, then ``dest`` will be connected to that same
diagram input. Or, if ``exemplar`` was neither connected or exported,
then this function is a no-op.

Both ``exemplar`` and ``dest`` must be ports of constituent systems
that have already been added to this diagram.

Returns:
    True iff any connection or was made; or false when a no-op.)""";
        } ConnectToSame;
        // Symbol: drake::systems::DiagramBuilder::DiagramBuilder<T>
        struct /* ctor */ {
          // Source: drake/systems/framework/diagram_builder.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::systems::DiagramBuilder::Disconnect
        struct /* Disconnect */ {
          // Source: drake/systems/framework/diagram_builder.h
          const char* doc =
R"""(Undoes a Connect() by disconnecting the given subsystem ports.

See also:
    connection_map()

Raises:
    RuntimeError if the ports were not already connected.)""";
        } Disconnect;
        // Symbol: drake::systems::DiagramBuilder::ExportInput
        struct /* ExportInput */ {
          // Source: drake/systems/framework/diagram_builder.h
          const char* doc =
R"""(Declares that the given ``input`` port of a constituent system is
connected to a new input to the entire Diagram. ``name`` is an
optional name for the new input port; if it is unspecified, then a
default name will be provided.

Precondition:
    If supplied at all, ``name`` must not be empty.

Precondition:
    A port indicated by the resolution of ``name`` must not exist.

Postcondition:
    ``input`` is connected to the new exported input port.

Returns:
    The index of the exported input port of the entire diagram.)""";
        } ExportInput;
        // Symbol: drake::systems::DiagramBuilder::ExportOutput
        struct /* ExportOutput */ {
          // Source: drake/systems/framework/diagram_builder.h
          const char* doc =
R"""(Declares that the given ``output`` port of a constituent system is an
output of the entire diagram. ``name`` is an optional name for the
output port; if it is unspecified, then a default name will be
provided.

Precondition:
    If supplied at all, ``name`` must not be empty.

Returns:
    The index of the exported output port of the entire diagram.)""";
        } ExportOutput;
        // Symbol: drake::systems::DiagramBuilder::GetDowncastSubsystemByName
        struct /* GetDowncastSubsystemByName */ {
          // Source: drake/systems/framework/diagram_builder.h
          const char* doc =
R"""(Retrieves a const reference to the subsystem with name ``name``
returned by get_name(), downcast to the type provided as a template
argument.

Template parameter ``MySystem``:
    is the downcast type, e.g., drake∷systems∷Adder

Raises:
    RuntimeError if a unique match cannot be found.

See also:
    GetMutableDowncastSubsystemByName()

See also:
    GetSubsystemByName())""";
        } GetDowncastSubsystemByName;
        // Symbol: drake::systems::DiagramBuilder::GetMutableDowncastSubsystemByName
        struct /* GetMutableDowncastSubsystemByName */ {
          // Source: drake/systems/framework/diagram_builder.h
          const char* doc =
R"""(Retrieves a mutable reference to the subsystem with name ``name``
returned by get_name(), downcast to the type provided as a template
argument.

Template parameter ``MySystem``:
    is the downcast type, e.g., drake∷systems∷Adder

Raises:
    RuntimeError if a unique match cannot be found.

See also:
    GetDowncastSubsystemByName()

See also:
    GetMutableSubsystemByName())""";
        } GetMutableDowncastSubsystemByName;
        // Symbol: drake::systems::DiagramBuilder::GetMutableSubsystemByName
        struct /* GetMutableSubsystemByName */ {
          // Source: drake/systems/framework/diagram_builder.h
          const char* doc =
R"""(Retrieves a mutable reference to the subsystem with name ``name``
returned by get_name().

Raises:
    RuntimeError if a unique match cannot be found.

See also:
    System<T>∷get_name()

See also:
    GetSubsystemByName()

See also:
    GetMutableDowncastSubsystemByName())""";
        } GetMutableSubsystemByName;
        // Symbol: drake::systems::DiagramBuilder::GetMutableSystems
        struct /* GetMutableSystems */ {
          // Source: drake/systems/framework/diagram_builder.h
          const char* doc =
R"""(Returns the list of contained Systems.

See also:
    GetMutableSubsystemByName()

See also:
    GetSystems())""";
        } GetMutableSystems;
        // Symbol: drake::systems::DiagramBuilder::GetSubsystemByName
        struct /* GetSubsystemByName */ {
          // Source: drake/systems/framework/diagram_builder.h
          const char* doc =
R"""(Retrieves a const reference to the subsystem with name ``name``
returned by get_name().

Raises:
    RuntimeError if a unique match cannot be found.

See also:
    System<T>∷get_name()

See also:
    GetMutableSubsystemByName()

See also:
    GetDowncastSubsystemByName())""";
        } GetSubsystemByName;
        // Symbol: drake::systems::DiagramBuilder::GetSystems
        struct /* GetSystems */ {
          // Source: drake/systems/framework/diagram_builder.h
          const char* doc =
R"""(Returns the list of contained Systems.

See also:
    GetSubsystemByName()

See also:
    GetMutableSystems())""";
        } GetSystems;
        // Symbol: drake::systems::DiagramBuilder::HasSubsystemNamed
        struct /* HasSubsystemNamed */ {
          // Source: drake/systems/framework/diagram_builder.h
          const char* doc =
R"""(Returns true iff this contains a subsystem with the given name.

See also:
    GetSubsystemByName())""";
        } HasSubsystemNamed;
        // Symbol: drake::systems::DiagramBuilder::InputPortLocator
        struct /* InputPortLocator */ {
          // Source: drake/systems/framework/diagram_builder.h
          const char* doc =
R"""(A designator for a "system + input port" pair, to uniquely refer to
some input port on one of this builder's subsystems.)""";
        } InputPortLocator;
        // Symbol: drake::systems::DiagramBuilder::IsConnectedOrExported
        struct /* IsConnectedOrExported */ {
          // Source: drake/systems/framework/diagram_builder.h
          const char* doc =
R"""(Returns true iff the given input ``port`` of a constituent system is
either connected to another constituent system or exported as a
diagram input.)""";
        } IsConnectedOrExported;
        // Symbol: drake::systems::DiagramBuilder::OutputPortLocator
        struct /* OutputPortLocator */ {
          // Source: drake/systems/framework/diagram_builder.h
          const char* doc =
R"""(A designator for a "system + output port" pair, to uniquely refer to
some output port on one of this builder's subsystems.)""";
        } OutputPortLocator;
        // Symbol: drake::systems::DiagramBuilder::RemoveSystem
        struct /* RemoveSystem */ {
          // Source: drake/systems/framework/diagram_builder.h
          const char* doc =
R"""(Removes the given system from this builder and disconnects any
connections or exported ports associated with it.

Note that un-exporting this system's ports might have a ripple effect
on other exported port index assignments. The relative order will
remain intact, but any "holes" created by this removal will be filled
in by decrementing the indices of all higher-numbered ports that
remain.

Warning:
    Because a DiagramBuilder owns the objects it contains, the system
    will be deleted.)""";
        } RemoveSystem;
        // Symbol: drake::systems::DiagramBuilder::already_built
        struct /* already_built */ {
          // Source: drake/systems/framework/diagram_builder.h
          const char* doc =
R"""(Returns true iff Build() or BuildInto() has been called on this
Builder, in which case it's an error to call any member function other
than the the destructor.)""";
        } already_built;
        // Symbol: drake::systems::DiagramBuilder::connection_map
        struct /* connection_map */ {
          // Source: drake/systems/framework/diagram_builder.h
          const char* doc =
R"""((Advanced) Returns a reference to the map of connections between
Systems. The reference becomes invalid upon any call to Build or
BuildInto.)""";
        } connection_map;
        // Symbol: drake::systems::DiagramBuilder::empty
        struct /* empty */ {
          // Source: drake/systems/framework/diagram_builder.h
          const char* doc =
R"""(Returns whether any Systems have been added yet.)""";
        } empty;
        // Symbol: drake::systems::DiagramBuilder::get_mutable_life_support
        struct /* get_mutable_life_support */ {
          // Source: drake/systems/framework/diagram_builder.h
          const char* doc =
R"""((Internal use only). Returns a mutable reference to life support data
for the diagram. The data will be moved to the diagram at Build()
time. Data stored here will have a life-cycle that is the union of the
builder and the diagram.)""";
        } get_mutable_life_support;
        // Symbol: drake::systems::DiagramBuilder::num_input_ports
        struct /* num_input_ports */ {
          // Source: drake/systems/framework/diagram_builder.h
          const char* doc =
R"""(Returns the current number of diagram input ports. The count may
change as more ports are exported.)""";
        } num_input_ports;
        // Symbol: drake::systems::DiagramBuilder::num_output_ports
        struct /* num_output_ports */ {
          // Source: drake/systems/framework/diagram_builder.h
          const char* doc =
R"""(Returns the current number of diagram output outputs. The count may
change as more ports are exported.)""";
        } num_output_ports;
      } DiagramBuilder;
      // Symbol: drake::systems::DiagramCompositeEventCollection
      struct /* DiagramCompositeEventCollection */ {
        // Source: drake/systems/framework/event_collection.h
        const char* doc =
R"""(CompositeEventCollection for a Diagram.

End users should never need to use or know about this class. It is for
internal use only.)""";
        // Symbol: drake::systems::DiagramCompositeEventCollection::DiagramCompositeEventCollection<T>
        struct /* ctor */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Allocated CompositeEventCollection for all constituent subsystems are
passed in ``subevents`` (a vector of size of the number of subsystems
of the corresponding diagram), for which ownership is also transferred
to ``this``.)""";
        } ctor;
        // Symbol: drake::systems::DiagramCompositeEventCollection::get_mutable_subevent_collection
        struct /* get_mutable_subevent_collection */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc = R"""()""";
        } get_mutable_subevent_collection;
        // Symbol: drake::systems::DiagramCompositeEventCollection::get_subevent_collection
        struct /* get_subevent_collection */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc = R"""()""";
        } get_subevent_collection;
        // Symbol: drake::systems::DiagramCompositeEventCollection::num_subsystems
        struct /* num_subsystems */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Returns the number of subsystems for which this object contains event
collections.)""";
        } num_subsystems;
      } DiagramCompositeEventCollection;
      // Symbol: drake::systems::DiagramContext
      struct /* DiagramContext */ {
        // Source: drake/systems/framework/diagram_context.h
        const char* doc =
R"""(The DiagramContext is a container for all of the data necessary to
uniquely determine the computations performed by a Diagram.
Specifically, a DiagramContext contains Context objects for all its
constituent Systems.

See also:
    Context for more information.

In general, users should not need to interact with a DiagramContext
directly. Use the accessors on Diagram instead.)""";
        // Symbol: drake::systems::DiagramContext::AddSystem
        struct /* AddSystem */ {
          // Source: drake/systems/framework/diagram_context.h
          const char* doc =
R"""(Declares a new subsystem in the DiagramContext. Subsystems are
identified by number. If the subsystem has already been declared,
aborts.

User code should not call this method. It is for use during Diagram
context allocation only.)""";
        } AddSystem;
        // Symbol: drake::systems::DiagramContext::DiagramContext<T>
        struct /* ctor */ {
          // Source: drake/systems/framework/diagram_context.h
          const char* doc =
R"""(Constructs a DiagramContext with the given ``num_subcontexts``, which
is final: you cannot resize a DiagramContext after construction. The
number and ordering of subcontexts is identical to the number and
ordering of subsystems in the corresponding Diagram.)""";
          // Source: drake/systems/framework/diagram_context.h
          const char* doc_copy =
R"""(Protected copy constructor takes care of the local data members and
all base class members, but doesn't update base class pointers so is
not a complete copy.)""";
        } ctor;
        // Symbol: drake::systems::DiagramContext::GetMutableSubsystemContext
        struct /* GetMutableSubsystemContext */ {
          // Source: drake/systems/framework/diagram_context.h
          const char* doc =
R"""(Returns the context structure for a given subsystem ``index``. Aborts
if ``index`` is out of bounds, or if no system has been added to the
DiagramContext at that index.)""";
        } GetMutableSubsystemContext;
        // Symbol: drake::systems::DiagramContext::GetSubsystemContext
        struct /* GetSubsystemContext */ {
          // Source: drake/systems/framework/diagram_context.h
          const char* doc =
R"""(Returns the context structure for a given constituent system
``index``. Aborts if ``index`` is out of bounds, or if no system has
been added to the DiagramContext at that index.)""";
        } GetSubsystemContext;
        // Symbol: drake::systems::DiagramContext::InputPortIdentifier
        struct /* InputPortIdentifier */ {
          // Source: drake/systems/framework/diagram_context.h
          const char* doc =
R"""(Identifies a child subsystem's input port.)""";
        } InputPortIdentifier;
        // Symbol: drake::systems::DiagramContext::MakeParameters
        struct /* MakeParameters */ {
          // Source: drake/systems/framework/diagram_context.h
          const char* doc =
R"""((Internal use only) Generates the parameters for the entire diagram by
wrapping the parameters of all the constituent Systems. The wrapper
simply holds pointers to the parameters in the subsystem Contexts. It
does not make a copy, or take ownership.)""";
        } MakeParameters;
        // Symbol: drake::systems::DiagramContext::MakeState
        struct /* MakeState */ {
          // Source: drake/systems/framework/diagram_context.h
          const char* doc =
R"""((Internal use only) Generates the state vector for the entire diagram
by wrapping the states of all the constituent diagrams.)""";
        } MakeState;
        // Symbol: drake::systems::DiagramContext::OutputPortIdentifier
        struct /* OutputPortIdentifier */ {
          // Source: drake/systems/framework/diagram_context.h
          const char* doc =
R"""(Identifies a child subsystem's output port.)""";
        } OutputPortIdentifier;
        // Symbol: drake::systems::DiagramContext::SubscribeDiagramCompositeTrackersToChildrens
        struct /* SubscribeDiagramCompositeTrackersToChildrens */ {
          // Source: drake/systems/framework/diagram_context.h
          const char* doc =
R"""((Internal use only) Makes the diagram state, parameter, and composite
cache entry trackers subscribe to the corresponding constituent
trackers in the child subcontexts.)""";
        } SubscribeDiagramCompositeTrackersToChildrens;
        // Symbol: drake::systems::DiagramContext::SubscribeDiagramPortToExportedOutputPort
        struct /* SubscribeDiagramPortToExportedOutputPort */ {
          // Source: drake/systems/framework/diagram_context.h
          const char* doc =
R"""((Internal use only) Declares that a particular output port of this
diagram is simply forwarded from an output port of one of its child
subsystems. Sets up tracking of the diagram port's dependency on the
child port. Aborts if the subsystem has not been added to the
DiagramContext.

User code should not call this method. It is for use during Diagram
context allocation only.)""";
        } SubscribeDiagramPortToExportedOutputPort;
        // Symbol: drake::systems::DiagramContext::SubscribeExportedInputPortToDiagramPort
        struct /* SubscribeExportedInputPortToDiagramPort */ {
          // Source: drake/systems/framework/diagram_context.h
          const char* doc =
R"""((Internal use only) Declares that a particular input port of a child
subsystem is an input to the entire Diagram that allocates this
Context. Sets up tracking of the child port's dependency on the parent
port. Aborts if the subsystem has not been added to the
DiagramContext.

User code should not call this method. It is for use during Diagram
context allocation only.)""";
        } SubscribeExportedInputPortToDiagramPort;
        // Symbol: drake::systems::DiagramContext::SubscribeInputPortToOutputPort
        struct /* SubscribeInputPortToOutputPort */ {
          // Source: drake/systems/framework/diagram_context.h
          const char* doc =
R"""((Internal use only) Declares that a connection exists between a peer
output port and input port in this Diagram, and registers the input
port's dependency tracker with the output port's dependency tracker.
By "peer" we mean that both ports belong to immediate child subsystems
of this Diagram (it is also possible for both ports to belong to the
same subsystem).

User code should not call this method. It is for use during Diagram
context allocation only.)""";
        } SubscribeInputPortToOutputPort;
      } DiagramContext;
      // Symbol: drake::systems::DiagramContinuousState
      struct /* DiagramContinuousState */ {
        // Source: drake/systems/framework/diagram_continuous_state.h
        const char* doc =
R"""(%DiagramContinuousState is a ContinuousState consisting of
Supervectors xc, q, v, z over the corresponding entries in a set of
referenced ContinuousState objects, which may or may not be owned by
this DiagramContinuousState. This is done recursively since any of the
referenced ContinuousState objects could themselves be
DiagramContinuousState objects. The actual numerical data is always
contained in the leaf ContinuousState objects at the bottom of the
tree.

This object is used both for a Diagram's actual continuous state
variables xc (with partitions q, v, z) and for the time derivatives
xdot (qdot, vdot, zdot). Cloning a DiagramContinuousState results in
an object with identical structure, but which owns the referenced
ContinuousState objects, regardless of whether the original had
ownership.)""";
        // Symbol: drake::systems::DiagramContinuousState::Clone
        struct /* Clone */ {
          // Source: drake/systems/framework/diagram_continuous_state.h
          const char* doc =
R"""(Creates a deep copy of this DiagramContinuousState, with the same
substructure but with new, owned data. Intentionally shadows the
ContinuousState∷Clone() method but with a more-specific return type so
you don't have to downcast.)""";
        } Clone;
        // Symbol: drake::systems::DiagramContinuousState::DiagramContinuousState<T>
        struct /* ctor */ {
          // Source: drake/systems/framework/diagram_continuous_state.h
          const char* doc_was_unable_to_choose_unambiguous_names = R"""()""";
        } ctor;
        // Symbol: drake::systems::DiagramContinuousState::get_mutable_substate
        struct /* get_mutable_substate */ {
          // Source: drake/systems/framework/diagram_continuous_state.h
          const char* doc =
R"""(Returns the continuous state at the given ``index``. Aborts if
``index`` is out-of-bounds.)""";
        } get_mutable_substate;
        // Symbol: drake::systems::DiagramContinuousState::get_substate
        struct /* get_substate */ {
          // Source: drake/systems/framework/diagram_continuous_state.h
          const char* doc =
R"""(Returns the continuous state at the given ``index``. Aborts if
``index`` is out-of-bounds.)""";
        } get_substate;
        // Symbol: drake::systems::DiagramContinuousState::num_substates
        struct /* num_substates */ {
          // Source: drake/systems/framework/diagram_continuous_state.h
          const char* doc = R"""()""";
        } num_substates;
      } DiagramContinuousState;
      // Symbol: drake::systems::DiagramDiscreteValues
      struct /* DiagramDiscreteValues */ {
        // Source: drake/systems/framework/diagram_discrete_values.h
        const char* doc =
R"""(DiagramDiscreteValues is a DiscreteValues container comprised
recursively of a sequence of child DiscreteValues objects. The API
allows this to be treated as though it were a single DiscreteValues
object whose groups are the concatenation of the groups in each child.

The child objects may be owned or not. When this is used to aggregate
LeafSystem discrete values in a Diagram, the child objects are not
owned. When this is cloned, deep copies are made that are owned here.)""";
        // Symbol: drake::systems::DiagramDiscreteValues::Clone
        struct /* Clone */ {
          // Source: drake/systems/framework/diagram_discrete_values.h
          const char* doc =
R"""(Creates a deep copy of this DiagramDiscreteValues object, with the
same substructure but with new, owned data. Intentionally shadows the
DiscreteValues∷Clone() method but with a more-specific return type so
you don't have to downcast.)""";
        } Clone;
        // Symbol: drake::systems::DiagramDiscreteValues::DiagramDiscreteValues<T>
        struct /* ctor */ {
          // Source: drake/systems/framework/diagram_discrete_values.h
          const char* doc_1args_subdiscretes =
R"""(Constructs a DiagramDiscreteValues object that is composed of other
DiscreteValues, which are not owned by this object and must outlive
it.

The DiagramDiscreteValues vector xd = [xd₁ xd₂ ...] where each of the
xdᵢ is an array of BasicVector objects. These will have the same
ordering as the ``subdiscretes`` parameter, which should be the order
of the Diagram itself. That is, the substates should be indexed by
SubsystemIndex in the same order as the subsystems are.)""";
          // Source: drake/systems/framework/diagram_discrete_values.h
          const char* doc_1args_owned_subdiscretes =
R"""(Constructs a DiagramDiscreteValues object that is composed
(recursively) of other DiscreteValues objects, ownership of which is
transferred here.)""";
        } ctor;
        // Symbol: drake::systems::DiagramDiscreteValues::get_mutable_subdiscrete
        struct /* get_mutable_subdiscrete */ {
          // Source: drake/systems/framework/diagram_discrete_values.h
          const char* doc =
R"""(Returns a mutable reference to one of the referenced DiscreteValues
objects which may or may not be owned locally.)""";
        } get_mutable_subdiscrete;
        // Symbol: drake::systems::DiagramDiscreteValues::get_subdiscrete
        struct /* get_subdiscrete */ {
          // Source: drake/systems/framework/diagram_discrete_values.h
          const char* doc =
R"""(Returns a const reference to one of the referenced DiscreteValues
objects which may or may not be owned locally.)""";
        } get_subdiscrete;
        // Symbol: drake::systems::DiagramDiscreteValues::num_subdiscretes
        struct /* num_subdiscretes */ {
          // Source: drake/systems/framework/diagram_discrete_values.h
          const char* doc =
R"""(Returns the number of DiscreteValues objects referenced by this
DiagramDiscreteValues object, necessarily the same as the number of
subcontexts in the containing DiagramContext.)""";
        } num_subdiscretes;
      } DiagramDiscreteValues;
      // Symbol: drake::systems::DiagramEventCollection
      struct /* DiagramEventCollection */ {
        // Source: drake/systems/framework/event_collection.h
        const char* doc =
R"""(A concrete class that holds all simultaneous *homogeneous* events for
a Diagram. For each subsystem in the corresponding Diagram, a derived
EventCollection instance is maintained internally, thus effectively
holding the same recursive tree structure as the corresponding
Diagram.

This class uses an unusual paradigm for storing collections of events
corresponding to subsystems of the diagram ("subevent collections").
The class owns some subevent collections and maintains pointers to
other subevent collections. The technical reasoning is that the same
data may need to be referenced by multiple collections;
DiagramCompositeEventCollection maintains one collection for all
publish events and another for the events from each subsystem, but
maintains only a single copy of all of the event data.

End users should never need to use or know about this class. It is for
internal use only.)""";
        // Symbol: drake::systems::DiagramEventCollection::AddEvent
        struct /* AddEvent */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Throws if called, because no events should be added at the Diagram
level.)""";
        } AddEvent;
        // Symbol: drake::systems::DiagramEventCollection::Clear
        struct /* Clear */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc = R"""(Clears all subevent collections.)""";
        } Clear;
        // Symbol: drake::systems::DiagramEventCollection::DiagramEventCollection<EventType>
        struct /* ctor */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Note that this constructor only resizes the containers; it does not
allocate any derived EventCollection instances.

Parameter ``num_subsystems``:
    Number of subsystems in the corresponding Diagram.)""";
        } ctor;
        // Symbol: drake::systems::DiagramEventCollection::DoAddToEnd
        struct /* DoAddToEnd */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Goes through each subevent collection of ``this`` and adds the
corresponding one in ``other_collection`` to the subevent collection
in ``this``. Aborts if ``this`` does not have the same number of
subevent collections as ``other_collection``. In addition, this method
assumes that ``this`` and ``other_collection`` have the exact same
topology (i.e. both are created for the same Diagram.)

Raises:
    RuntimeError if ``other_collection`` is not an instance of
    DiagramEventCollection.)""";
        } DoAddToEnd;
        // Symbol: drake::systems::DiagramEventCollection::HasEvents
        struct /* HasEvents */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Returns ``True`` if and only if any of the subevent collections have
any events.)""";
        } HasEvents;
        // Symbol: drake::systems::DiagramEventCollection::get_mutable_subevent_collection
        struct /* get_mutable_subevent_collection */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Returns a mutable pointer to subsystem's EventCollection at ``index``.)""";
        } get_mutable_subevent_collection;
        // Symbol: drake::systems::DiagramEventCollection::get_subevent_collection
        struct /* get_subevent_collection */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Returns a const pointer to subsystem's EventCollection at ``index``.
Aborts if the 0-indexed ``index`` is greater than or equal to the
number of subsystems specified in this object's construction (see
DiagramEventCollection(int)) or if ``index`` is not in the range [0,
num_subsystems() - 1].)""";
        } get_subevent_collection;
        // Symbol: drake::systems::DiagramEventCollection::num_subsystems
        struct /* num_subsystems */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Returns the number of constituent EventCollection objects that
correspond to each subsystem in the Diagram.)""";
        } num_subsystems;
        // Symbol: drake::systems::DiagramEventCollection::set_and_own_subevent_collection
        struct /* set_and_own_subevent_collection */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Transfers ``subevent_collection`` ownership to ``this`` and associates
it with the subsystem identified by ``index``. Aborts if ``index`` is
not in the range [0, num_subsystems() - 1] or if
``subevent_collection`` is null.)""";
        } set_and_own_subevent_collection;
        // Symbol: drake::systems::DiagramEventCollection::set_subevent_collection
        struct /* set_subevent_collection */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Associate ``subevent_collection`` with subsystem identified by
``index``. Ownership of the object that ``subevent_collection`` is
maintained elsewhere, and its life span must be longer than this.
Aborts if ``index`` is not in the range [0, num_subsystems() - 1] or
if ``subevent_collection`` is null.)""";
        } set_subevent_collection;
      } DiagramEventCollection;
      // Symbol: drake::systems::DiagramOutputPort
      struct /* DiagramOutputPort */ {
        // Source: drake/systems/framework/diagram_output_port.h
        const char* doc =
R"""((Advanced.) Holds information about a subsystem output port that has
been exported to become one of this Diagram's output ports. The actual
methods for determining the port's value are supplied by the
LeafOutputPort that ultimately underlies the source port, although
that may be any number of levels down. This is intended for internal
use in implementing Diagram.)""";
        // Symbol: drake::systems::DiagramOutputPort::DiagramOutputPort<T>
        struct /* ctor */ {
          // Source: drake/systems/framework/diagram_output_port.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::systems::DiagramOutputPort::get_source_output_port
        struct /* get_source_output_port */ {
          // Source: drake/systems/framework/diagram_output_port.h
          const char* doc =
R"""(Obtains a reference to the subsystem output port that was exported to
create this diagram port. Note that the source may itself be a diagram
output port.)""";
        } get_source_output_port;
      } DiagramOutputPort;
      // Symbol: drake::systems::DiagramState
      struct /* DiagramState */ {
        // Source: drake/systems/framework/diagram_state.h
        const char* doc =
R"""(DiagramState is a State, annotated with pointers to all the mutable
substates that it spans.)""";
        // Symbol: drake::systems::DiagramState::DiagramState<T>
        struct /* ctor */ {
          // Source: drake/systems/framework/diagram_state.h
          const char* doc =
R"""(Constructs a DiagramState consisting of ``size`` substates.)""";
        } ctor;
        // Symbol: drake::systems::DiagramState::Finalize
        struct /* Finalize */ {
          // Source: drake/systems/framework/diagram_state.h
          const char* doc =
R"""(Finalizes this state as a span of all the constituent substates.)""";
        } Finalize;
        // Symbol: drake::systems::DiagramState::get_mutable_substate
        struct /* get_mutable_substate */ {
          // Source: drake/systems/framework/diagram_state.h
          const char* doc = R"""(Returns the substate at ``index``.)""";
        } get_mutable_substate;
        // Symbol: drake::systems::DiagramState::get_substate
        struct /* get_substate */ {
          // Source: drake/systems/framework/diagram_state.h
          const char* doc = R"""(Returns the substate at ``index``.)""";
        } get_substate;
        // Symbol: drake::systems::DiagramState::set_and_own_substate
        struct /* set_and_own_substate */ {
          // Source: drake/systems/framework/diagram_state.h
          const char* doc =
R"""(Sets the substate at ``index`` to ``substate``, or aborts if ``index``
is out of bounds.)""";
        } set_and_own_substate;
        // Symbol: drake::systems::DiagramState::set_substate
        struct /* set_substate */ {
          // Source: drake/systems/framework/diagram_state.h
          const char* doc =
R"""(Sets the substate at ``index`` to ``substate``, or aborts if ``index``
is out of bounds. Does not take ownership of ``substate``, which must
live as long as this object.)""";
        } set_substate;
      } DiagramState;
      // Symbol: drake::systems::DiscreteStateIndex
      struct /* DiscreteStateIndex */ {
        // Source: drake/systems/framework/framework_common.h
        const char* doc =
R"""(Serves as the local index for discrete state groups within a given
System and its corresponding Context.)""";
      } DiscreteStateIndex;
      // Symbol: drake::systems::DiscreteUpdateEvent
      struct /* DiscreteUpdateEvent */ {
        // Source: drake/systems/framework/event.h
        const char* doc =
R"""(This class represents a discrete update event. It has an optional
callback function to do custom handling of this event, and that can
write updates to a mutable, non-null DiscreteValues object.

See also:
    LeafSystem for more convenient interfaces to discrete update
    events via the Declare*DiscreteUpdateEvent() methods.)""";
        // Symbol: drake::systems::DiscreteUpdateEvent::DiscreteUpdateEvent<T>
        struct /* ctor */ {
          // Source: drake/systems/framework/event.h
          const char* doc_0args =
R"""(Constructs an empty DiscreteUpdateEvent.)""";
          // Source: drake/systems/framework/event.h
          const char* doc_1args =
R"""(Constructs a DiscreteUpdateEvent with the given callback function.)""";
        } ctor;
        // Symbol: drake::systems::DiscreteUpdateEvent::handle
        struct /* handle */ {
          // Source: drake/systems/framework/event.h
          const char* doc =
R"""(Calls the optional callback function, if one exists, with ``system``,
``context``, ``this`` and ``discrete_state``.)""";
        } handle;
        // Symbol: drake::systems::DiscreteUpdateEvent::is_discrete_update
        struct /* is_discrete_update */ {
          // Source: drake/systems/framework/event.h
          const char* doc = R"""()""";
        } is_discrete_update;
      } DiscreteUpdateEvent;
      // Symbol: drake::systems::DiscreteValues
      struct /* DiscreteValues */ {
        // Source: drake/systems/framework/discrete_values.h
        const char* doc =
R"""(%DiscreteValues is a container for numerical but non-continuous state
and parameters. It may own its underlying data, for use with leaf
Systems, or not, for use with Diagrams.

DiscreteValues is an ordered collection xd of BasicVector "groups" so
xd = [xd₀, xd₁...], where each group xdᵢ is a contiguous vector.
Requesting a specific group index from this collection is the most
granular way to retrieve discrete values from the Context, and thus is
the unit of cache invalidation. System authors are encouraged to
partition their DiscreteValues such that each cacheable computation
within the System may depend on only the elements of DiscreteValues
that it needs.

None of the contained vectors (groups) may be null, although any of
them may be zero-length.)""";
        // Symbol: drake::systems::DiscreteValues::AppendGroup
        struct /* AppendGroup */ {
          // Source: drake/systems/framework/discrete_values.h
          const char* doc =
R"""(Adds an additional group that owns the given ``datum``, which must be
non-null. Returns the assigned group number, counting up from 0 for
the first group.

Warning:
    Do not use this method to add groups to a DiscreteValues object
    that is owned by an existing Context.)""";
        } AppendGroup;
        // Symbol: drake::systems::DiscreteValues::Clone
        struct /* Clone */ {
          // Source: drake/systems/framework/discrete_values.h
          const char* doc =
R"""(Creates a deep copy of this object with the same substructure but with
all data owned by the copy. That is, if the original was a
DiagramDiscreteValues object that maintains a tree of sub-objects, the
clone will not include any references to the original sub-objects and
is thus decoupled from the Context containing the original. The
concrete type of the BasicVector underlying each leaf DiscreteValue is
preserved.)""";
        } Clone;
        // Symbol: drake::systems::DiscreteValues::DiscreteValues<T>
        struct /* ctor */ {
          // Source: drake/systems/framework/discrete_values.h
          const char* doc_0args =
R"""(Constructs an empty DiscreteValues object containing no groups.)""";
          // Source: drake/systems/framework/discrete_values.h
          const char* doc_1args_data =
R"""(Constructs a DiscreteValues that owns the underlying ``data``. Every
entry must be non-null.)""";
          // Source: drake/systems/framework/discrete_values.h
          const char* doc_1args_datum =
R"""(Constructs a one-group DiscreteValues object that owns a single
``datum`` vector which may not be null.)""";
        } ctor;
        // Symbol: drake::systems::DiscreteValues::SetFrom
        struct /* SetFrom */ {
          // Source: drake/systems/framework/discrete_values.h
          const char* doc =
R"""(Resets the values in this DiscreteValues from the values in ``other``,
possibly writing through to unowned data. Throws if the dimensions
don't match.)""";
        } SetFrom;
        // Symbol: drake::systems::DiscreteValues::get_data
        struct /* get_data */ {
          // Source: drake/systems/framework/discrete_values.h
          const char* doc = R"""()""";
        } get_data;
        // Symbol: drake::systems::DiscreteValues::get_mutable_value
        struct /* get_mutable_value */ {
          // Source: drake/systems/framework/discrete_values.h
          const char* doc_0args =
R"""(Returns the entire vector for the *only* group as a mutable
Eigen∷VectorBlock, which allows mutation of the values, but does not
allow resize() to be called on the vector.)""";
          // Source: drake/systems/framework/discrete_values.h
          const char* doc_1args =
R"""(Returns the entire vector for the indicated group as a mutable
Eigen∷VectorBlock, which allows mutation of the values, but does not
allow resize() to be called on the vector.)""";
        } get_mutable_value;
        // Symbol: drake::systems::DiscreteValues::get_mutable_vector
        struct /* get_mutable_vector */ {
          // Source: drake/systems/framework/discrete_values.h
          const char* doc_0args =
R"""((Advanced) Returns a mutable reference to the BasicVector containing
the values for the *only* group. Prefer ``get_mutable_value()`` to get
the underlying Eigen object.)""";
          // Source: drake/systems/framework/discrete_values.h
          const char* doc_1args =
R"""((Advanced) Returns a mutable reference to the BasicVector holding data
for the indicated group. Prefer ``get_mutable_value()`` to get the
underlying Eigen object unless you really need the BasicVector
wrapping it.)""";
        } get_mutable_vector;
        // Symbol: drake::systems::DiscreteValues::get_system_id
        struct /* get_system_id */ {
          // Source: drake/systems/framework/discrete_values.h
          const char* doc =
R"""((Internal use only) Gets the id of the subsystem that created this
object.)""";
        } get_system_id;
        // Symbol: drake::systems::DiscreteValues::get_value
        struct /* get_value */ {
          // Source: drake/systems/framework/discrete_values.h
          const char* doc_0args =
R"""((Don't use this in new code) Returns the entire vector as a const
Eigen∷VectorBlock for the *only* group. Prefer ``value()`` which
returns direct access to the underlying VectorX rather than wrapping
it in a VectorBlock.)""";
          // Source: drake/systems/framework/discrete_values.h
          const char* doc_1args =
R"""((Don't use this in new code) Returns the entire vector as a const
Eigen∷VectorBlock for the indicated group. Prefer ``value()`` which
returns direct access to the underlying VectorX rather than wrapping
it in a VectorBlock.)""";
        } get_value;
        // Symbol: drake::systems::DiscreteValues::get_vector
        struct /* get_vector */ {
          // Source: drake/systems/framework/discrete_values.h
          const char* doc_0args =
R"""((Advanced) Returns a const reference to the BasicVector containing the
values for the *only* group. Prefer ``value()`` to get the underlying
VectorX directly unless you really need the BasicVector wrapping it.)""";
          // Source: drake/systems/framework/discrete_values.h
          const char* doc_1args =
R"""((Advanced) Returns a const reference to the BasicVector holding data
for the indicated group. Prefer ``value(index)`` to get the underlying
VectorX directly unless you really need the BasicVector wrapping it.)""";
        } get_vector;
        // Symbol: drake::systems::DiscreteValues::num_groups
        struct /* num_groups */ {
          // Source: drake/systems/framework/discrete_values.h
          const char* doc = R"""()""";
        } num_groups;
        // Symbol: drake::systems::DiscreteValues::operator[]
        struct /* operator_array */ {
          // Source: drake/systems/framework/discrete_values.h
          const char* doc_1args_idx_nonconst =
R"""(Returns a mutable reference to an element in the *only* group.)""";
          // Source: drake/systems/framework/discrete_values.h
          const char* doc_1args_idx_const =
R"""(Returns a const reference to an element in the *only* group.)""";
        } operator_array;
        // Symbol: drake::systems::DiscreteValues::set_system_id
        struct /* set_system_id */ {
          // Source: drake/systems/framework/discrete_values.h
          const char* doc =
R"""((Internal use only) Records the id of the subsystem that created this
object.)""";
        } set_system_id;
        // Symbol: drake::systems::DiscreteValues::set_value
        struct /* set_value */ {
          // Source: drake/systems/framework/discrete_values.h
          const char* doc_1args =
R"""(Sets the vector to the given value for the *only* group.)""";
          // Source: drake/systems/framework/discrete_values.h
          const char* doc_2args =
R"""(Sets the vector to the given value for the indicated group.)""";
        } set_value;
        // Symbol: drake::systems::DiscreteValues::size
        struct /* size */ {
          // Source: drake/systems/framework/discrete_values.h
          const char* doc =
R"""(Returns the number of elements in the only DiscreteValues group.)""";
        } size;
        // Symbol: drake::systems::DiscreteValues::value
        struct /* value */ {
          // Source: drake/systems/framework/discrete_values.h
          const char* doc_0args =
R"""(Returns a const reference to the underlying VectorX containing the
values for the *only* group. This is the preferred method for
examining the value of the only group.)""";
          // Source: drake/systems/framework/discrete_values.h
          const char* doc_1args =
R"""(Returns a const reference to the underlying VectorX containing the
values for the indicated group. This is the preferred method for
examining the value of a group.)""";
        } value;
      } DiscreteValues;
      // Symbol: drake::systems::Event
      struct /* Event */ {
        // Source: drake/systems/framework/event.h
        const char* doc =
R"""(Abstract base class that represents an event. The base event contains
two main pieces of information: an enum trigger type and an optional
attribute that can be used to explain why the event is triggered.

Concrete derived classes contain a function pointer to an optional
callback that handles the event. No-op is the default handling
behavior. The System framework supports three concrete event types:
PublishEvent, DiscreteUpdateEvent, and UnrestrictedUpdateEvent
distinguished by their callback functions' write access level to the
State.

The most common and convenient use of events and callbacks will happen
via the LeafSystem Declare*Event() methods. To that end, the callback
signature always passes the ``const System<T>&`` as the first
argument, so that LeafSystem does not need to capture ``this`` into
the lambda; typically only a pointer-to-member- function is captured.
Capturing any more than that would defeat std∷function's small buffer
optimization and cause heap allocations when scheduling events.

Event handling occurs during a simulation of a system. The logic that
describes when particular event types are handled is described in the
class documentation for Simulator.)""";
        // Symbol: drake::systems::Event::AddToComposite
        struct /* AddToComposite */ {
          // Source: drake/systems/framework/event.h
          const char* doc_2args =
R"""(Adds a clone of ``this`` event to the event collection ``events``,
with the given trigger type. If ``this`` event has an unknown trigger
type, then any trigger type is acceptable. Otherwise the given trigger
type must match the trigger type stored in ``this`` event.

Precondition:
    ``trigger_type`` must match the current trigger type unless that
    is unknown.

Precondition:
    ``events`` must not be null.)""";
          // Source: drake/systems/framework/event.h
          const char* doc_1args =
R"""(Provides an alternate signature for adding an Event that already has
the correct trigger type set. Must not have an unknown trigger type.)""";
        } AddToComposite;
        // Symbol: drake::systems::Event::Clone
        struct /* Clone */ {
          // Source: drake/systems/framework/event.h
          const char* doc = R"""(Clones this instance.)""";
        } Clone;
        // Symbol: drake::systems::Event::DoAddToComposite
        struct /* DoAddToComposite */ {
          // Source: drake/systems/framework/event.h
          const char* doc =
R"""(Derived classes must implement this to add a clone of this Event to
the event collection and unconditionally set its trigger type.)""";
        } DoAddToComposite;
        // Symbol: drake::systems::Event::DoClone
        struct /* DoClone */ {
          // Source: drake/systems/framework/event.h
          const char* doc =
R"""(Derived classes must implement this method to clone themselves. Any
Event-specific data is cloned using the Clone() method. Data specific
to the class derived from Event must be cloned by the implementation.)""";
        } DoClone;
        // Symbol: drake::systems::Event::Event<T>
        struct /* ctor */ {
          // Source: drake/systems/framework/event.h
          const char* doc = R"""(Constructs an empty Event.)""";
        } ctor;
        // Symbol: drake::systems::Event::TriggerType
        struct /* TriggerType */ {
          // Source: drake/systems/framework/event.h
          const char* doc = R"""()""";
        } TriggerType;
        // Symbol: drake::systems::Event::get_event_data
        struct /* get_event_data */ {
          // Source: drake/systems/framework/event.h
          const char* doc =
R"""(Returns a const pointer to the event data. The returned value can be
nullptr, which means this event does not have any associated data of
the given ``EventDataType``.

Template parameter ``EventDataType``:
    the expected data type for an event that has this trigger type
    (PeriodicEventData or WitnessTriggeredEventData).)""";
        } get_event_data;
        // Symbol: drake::systems::Event::get_mutable_event_data
        struct /* get_mutable_event_data */ {
          // Source: drake/systems/framework/event.h
          const char* doc =
R"""(Returns a mutable pointer to the event data. The returned value can be
nullptr, which means this event does not have any associated data of
the given ``EventDataType``.

Template parameter ``EventDataType``:
    the expected data type for an event that has this trigger type
    (PeriodicEventData or WitnessTriggeredEventData).)""";
        } get_mutable_event_data;
        // Symbol: drake::systems::Event::get_trigger_type
        struct /* get_trigger_type */ {
          // Source: drake/systems/framework/event.h
          const char* doc = R"""(Returns the trigger type.)""";
        } get_trigger_type;
        // Symbol: drake::systems::Event::has_event_data
        struct /* has_event_data */ {
          // Source: drake/systems/framework/event.h
          const char* doc =
R"""(Returns true if this event has associated data of the given
``EventDataType``.

Template parameter ``EventDataType``:
    the expected data type for an event that has this trigger type
    (PeriodicEventData or WitnessTriggeredEventData).)""";
        } has_event_data;
        // Symbol: drake::systems::Event::is_discrete_update
        struct /* is_discrete_update */ {
          // Source: drake/systems/framework/event.h
          const char* doc =
R"""(Returns ``True`` if this is a DiscreteUpdateEvent.)""";
        } is_discrete_update;
        // Symbol: drake::systems::Event::set_event_data
        struct /* set_event_data */ {
          // Source: drake/systems/framework/event.h
          const char* doc = R"""()""";
        } set_event_data;
        // Symbol: drake::systems::Event::set_trigger_type
        struct /* set_trigger_type */ {
          // Source: drake/systems/framework/event.h
          const char* doc = R"""()""";
        } set_trigger_type;
      } Event;
      // Symbol: drake::systems::EventCollection
      struct /* EventCollection */ {
        // Source: drake/systems/framework/event_collection.h
        const char* doc =
R"""(There are three concrete event types for any System: publish, discrete
state update, and unrestricted state update, listed in order of
increasing ability to change the state (i.e., zero to all).
EventCollection is an abstract base class that stores simultaneous
events *of the same type* that occur *at the same time* (i.e.,
simultaneous events).

The Simulator promises that for each set of simultaneous events of the
same type, the public event handling method (e.g.,
System∷Publish(context, publish_events)) will be invoked exactly once.

The System API provides several functions for customizable event
generation such as System∷DoCalcNextUpdateTime() or
System∷DoGetPerStepEvents(). These functions can return any number of
events of arbitrary types, and the resulting events are stored in
separate CompositeEventCollection instances. Before calling the event
handlers, all of these CompositeEventCollection objects must be merged
to generate a complete set of simultaneous events. Then, only events
of the appropriate type are passed to the event handlers. e.g.
sys.Publish(context, combined_event_collection.get_publish_events()).
For example, the Simulator executes this collation process when it is
applied to simulate a system.

Here is a complete example. For some LeafSystem ``sys`` at time ``t``,
its System∷DoCalcNextUpdateTime() generates the following
CompositeEventCollection (``events1``):


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    PublishEvent: {event1(kPeriodic, callback1)}
      DiscreteUpdateEvent: {event2(kPeriodic, callback2)}
      UnrestrictedUpdateEvent: {}

.. raw:: html

    </details>

This LeafSystem also desires per-step event processing(``events2``),
generated by its implementation of System∷DoGetPerStepEvents():


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    PublishEvent: {event3(kPerStep, callback3)}
      DiscreteUpdateEvent: {}
      UnrestrictedUpdateEvent: {event4(kPerStep,callback4)}

.. raw:: html

    </details>

These collections of "simultaneous" events, ``events1`` and
``events2``, are then merged into the composite event collection
``all_events``:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    PublishEvent: {event1, event3}
      DiscreteUpdateEvent: {event2}
      UnrestrictedUpdateEvent: {event4}

.. raw:: html

    </details>

This heterogeneous event collection can be processed by calling the
appropriate handler on the appropriate homogeneous subcollection:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    sys.CalcUnrestrictedUpdate(context,
          all_events.get_unrestricted_update_events(), state);
      sys.CalcDiscreteVariableUpdate(context,
          all_events.get_discrete_update_events(), discrete_state);
      sys.Publish(context, all_events.get_publish_events())

.. raw:: html

    </details>

Template parameter ``EventType``:
    a concrete derived type of Event (e.g., PublishEvent).)""";
        // Symbol: drake::systems::EventCollection::AddEvent
        struct /* AddEvent */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Adds an event to this collection, or throws if the concrete collection
does not permit adding new events. Derived classes must implement this
method to add the specified event to the homogeneous event collection.)""";
        } AddEvent;
        // Symbol: drake::systems::EventCollection::AddToEnd
        struct /* AddToEnd */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Adds all of ``other`'s events to the end of `this``.)""";
        } AddToEnd;
        // Symbol: drake::systems::EventCollection::Clear
        struct /* Clear */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Removes all events from this collection.)""";
        } Clear;
        // Symbol: drake::systems::EventCollection::DoAddToEnd
        struct /* DoAddToEnd */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc = R"""()""";
        } DoAddToEnd;
        // Symbol: drake::systems::EventCollection::EventCollection<EventType>
        struct /* ctor */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Constructor only accessible by derived class.)""";
        } ctor;
        // Symbol: drake::systems::EventCollection::HasEvents
        struct /* HasEvents */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Returns ``False`` if and only if this collection contains no events.)""";
        } HasEvents;
        // Symbol: drake::systems::EventCollection::SetFrom
        struct /* SetFrom */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Clears all the events maintained by ``this`` then adds all of the
events in ``other`` to this.)""";
        } SetFrom;
      } EventCollection;
      // Symbol: drake::systems::EventStatus
      struct /* EventStatus */ {
        // Source: drake/systems/framework/event_status.h
        const char* doc =
R"""(Holds the return status from execution of an event handler function,
or the effective status after a series of handler executions due to
dispatching of simultaneous events. Drake API users will typically use
only the four factory methods below to return status, and optionally a
human-readable message, from their event handlers.)""";
        // Symbol: drake::systems::EventStatus::DidNothing
        struct /* DidNothing */ {
          // Source: drake/systems/framework/event_status.h
          const char* doc =
R"""(Returns "did nothing" status, with no message.)""";
        } DidNothing;
        // Symbol: drake::systems::EventStatus::EventStatus
        struct /* ctor */ {
          // Source: drake/systems/framework/event_status.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::systems::EventStatus::Failed
        struct /* Failed */ {
          // Source: drake/systems/framework/event_status.h
          const char* doc =
R"""(Returns "failed" status, with a message explaining why.)""";
        } Failed;
        // Symbol: drake::systems::EventStatus::KeepMoreSevere
        struct /* KeepMoreSevere */ {
          // Source: drake/systems/framework/event_status.h
          const char* doc =
R"""((Advanced) Replaces the contents of ``this`` with the more-severe
status if ``candidate`` is a more severe status than ``this`` one.
Does nothing if ``candidate`` severity is less than or equal to
``this`` severity. This method is for use in event dispatchers for
accumulating status returns from a series of event handlers for a set
of simultaneous events.)""";
        } KeepMoreSevere;
        // Symbol: drake::systems::EventStatus::ReachedTermination
        struct /* ReachedTermination */ {
          // Source: drake/systems/framework/event_status.h
          const char* doc =
R"""(Returns "reached termination" status, with a message explaining why.)""";
        } ReachedTermination;
        // Symbol: drake::systems::EventStatus::Severity
        struct /* Severity */ {
          // Source: drake/systems/framework/event_status.h
          const char* doc =
R"""(The numerical values are ordered, with did_nothing < success <
terminate < fatal.)""";
          // Symbol: drake::systems::EventStatus::Severity::kDidNothing
          struct /* kDidNothing */ {
            // Source: drake/systems/framework/event_status.h
            const char* doc =
R"""(Successful, but nothing happened; no state update needed.)""";
          } kDidNothing;
          // Symbol: drake::systems::EventStatus::Severity::kFailed
          struct /* kFailed */ {
            // Source: drake/systems/framework/event_status.h
            const char* doc =
R"""(Handler was unable to perform its job (has message).)""";
          } kFailed;
          // Symbol: drake::systems::EventStatus::Severity::kReachedTermination
          struct /* kReachedTermination */ {
            // Source: drake/systems/framework/event_status.h
            const char* doc =
R"""(Handler succeeded but detected a normal termination condition (has
message). Intended primarily for internal use by the Simulator.)""";
          } kReachedTermination;
          // Symbol: drake::systems::EventStatus::Severity::kSucceeded
          struct /* kSucceeded */ {
            // Source: drake/systems/framework/event_status.h
            const char* doc =
R"""(Handler executed successfully; state may have been updated.)""";
          } kSucceeded;
        } Severity;
        // Symbol: drake::systems::EventStatus::Succeeded
        struct /* Succeeded */ {
          // Source: drake/systems/framework/event_status.h
          const char* doc =
R"""(Returns "succeeded" status, with no message.)""";
        } Succeeded;
        // Symbol: drake::systems::EventStatus::ThrowOnFailure
        struct /* ThrowOnFailure */ {
          // Source: drake/systems/framework/event_status.h
          const char* doc =
R"""(If failed(), throws an RuntimeError with a human-readable message.

Parameter ``function_name``:
    The name of the user-callable API that encountered the failure.
    Don't include "()".)""";
        } ThrowOnFailure;
        // Symbol: drake::systems::EventStatus::did_nothing
        struct /* did_nothing */ {
          // Source: drake/systems/framework/event_status.h
          const char* doc =
R"""(Returns ``True`` if the status is DidNothing.)""";
        } did_nothing;
        // Symbol: drake::systems::EventStatus::failed
        struct /* failed */ {
          // Source: drake/systems/framework/event_status.h
          const char* doc =
R"""(Returns ``True`` if the status is Failed. There will also be a
message() with more detail.)""";
        } failed;
        // Symbol: drake::systems::EventStatus::message
        struct /* message */ {
          // Source: drake/systems/framework/event_status.h
          const char* doc =
R"""(Returns the optionally-provided human-readable message supplied by the
event handler that produced the current status. Returns an empty
string if no message was provided.)""";
        } message;
        // Symbol: drake::systems::EventStatus::reached_termination
        struct /* reached_termination */ {
          // Source: drake/systems/framework/event_status.h
          const char* doc =
R"""(Returns ``True`` if the status is ReachedTermination. There will also
be a message() with more detail.)""";
        } reached_termination;
        // Symbol: drake::systems::EventStatus::severity
        struct /* severity */ {
          // Source: drake/systems/framework/event_status.h
          const char* doc =
R"""(Returns the severity of the current status.)""";
        } severity;
        // Symbol: drake::systems::EventStatus::succeeded
        struct /* succeeded */ {
          // Source: drake/systems/framework/event_status.h
          const char* doc =
R"""(Returns ``True`` if the status is Succeeded. "Did nothing" can also be
viewed as successful but you have to check for that separately.)""";
        } succeeded;
        // Symbol: drake::systems::EventStatus::system
        struct /* system */ {
          // Source: drake/systems/framework/event_status.h
          const char* doc =
R"""(Returns the optionally-provided subsystem that generated a status
return that can include a message (reached termination or failed).
Returns nullptr if no subsystem was provided.)""";
        } system;
      } EventStatus;
      // Symbol: drake::systems::ExternalSystemConstraint
      struct /* ExternalSystemConstraint */ {
        // Source: drake/systems/framework/system_constraint.h
        const char* doc =
R"""(An "external" constraint on a System. This class is intended for use
by applications that are examining a System by adding additional
constraints based on their particular situation (e.g., that a velocity
state element has an upper bound); it is not intended for declaring
intrinsic constraints that some particular System subclass might
always impose on itself (e.g., that a mass parameter is non-negative).)""";
        // Symbol: drake::systems::ExternalSystemConstraint::ExternalSystemConstraint
        struct /* ctor */ {
          // Source: drake/systems/framework/system_constraint.h
          const char* doc_0args = R"""(Creates an empty constraint.)""";
          // Source: drake/systems/framework/system_constraint.h
          const char* doc_5args =
R"""(Creates a constraint with the given arguments. The calc functions
(other than calc_double) may be omitted.)""";
        } ctor;
        // Symbol: drake::systems::ExternalSystemConstraint::MakeForAllScalars
        struct /* MakeForAllScalars */ {
          // Source: drake/systems/framework/system_constraint.h
          const char* doc =
R"""(Creates a constraint based on generic lambda. This constraint will
supply Calc functions for Drake's default scalar types.)""";
        } MakeForAllScalars;
        // Symbol: drake::systems::ExternalSystemConstraint::MakeForNonsymbolicScalars
        struct /* MakeForNonsymbolicScalars */ {
          // Source: drake/systems/framework/system_constraint.h
          const char* doc =
R"""(Creates a constraint based on generic lambda. This constraint will
supply Calc functions for Drake's non-symbolic default scalar types.)""";
        } MakeForNonsymbolicScalars;
        // Symbol: drake::systems::ExternalSystemConstraint::bounds
        struct /* bounds */ {
          // Source: drake/systems/framework/system_constraint.h
          const char* doc =
R"""(Returns the bounds of this constraint (and whether it is an equality
or inequality constraint.))""";
        } bounds;
        // Symbol: drake::systems::ExternalSystemConstraint::description
        struct /* description */ {
          // Source: drake/systems/framework/system_constraint.h
          const char* doc =
R"""(Returns a human-readable description of this constraint.)""";
        } description;
        // Symbol: drake::systems::ExternalSystemConstraint::get_calc
        struct /* get_calc */ {
          // Source: drake/systems/framework/system_constraint.h
          const char* doc =
R"""(Retrieves the evaluation function ``value = f(system, context)`` for
this constraint. The result may be a default-constructed (missing)
function, if the scalar type T is not supported by this constraint
instance.

Template parameter ``T``:
    denotes the scalar type of the System<T>.)""";
        } get_calc;
      } ExternalSystemConstraint;
      // Symbol: drake::systems::FixedInputPortValue
      struct /* FixedInputPortValue */ {
        // Source: drake/systems/framework/fixed_input_port_value.h
        const char* doc =
R"""(A FixedInputPortValue encapsulates a vector or abstract value for use
as an internal value source for one of a System's input ports. The
semantics are identical to a Parameter. We assign a DependencyTracker
to this object and subscribe the InputPort to it when that port is
fixed. Any modification to the value here issues a notification to its
dependent, and increments a serial number kept here.)""";
        // Symbol: drake::systems::FixedInputPortValue::FixedInputPortValue
        struct /* ctor */ {
          // Source: drake/systems/framework/fixed_input_port_value.h
          const char* doc_move =
R"""(@name Does not allow move or assignment; copy is private.)""";
        } ctor;
        // Symbol: drake::systems::FixedInputPortValue::GetMutableData
        struct /* GetMutableData */ {
          // Source: drake/systems/framework/fixed_input_port_value.h
          const char* doc =
R"""(Returns a pointer to the data inside this FixedInputPortValue, and
notifies the dependent input port that the value has changed.

To ensure invalidation notifications are delivered, callers should
call this method every time they wish to update the stored value. In
particular, callers MUST NOT write through the returned pointer if
there is any possibility this FixedInputPortValue has been accessed
since the last time this method was called.)""";
        } GetMutableData;
        // Symbol: drake::systems::FixedInputPortValue::GetMutableVectorData
        struct /* GetMutableVectorData */ {
          // Source: drake/systems/framework/fixed_input_port_value.h
          const char* doc =
R"""(Returns a pointer to the data inside this FixedInputPortValue, and
notifies the dependent input port that the value has changed,
invalidating downstream computations.

Raises:
    RuntimeError if the data is not vector data.

To ensure invalidation notifications are delivered, callers should
call this method every time they wish to update the stored value. In
particular, callers MUST NOT write through the returned pointer if
there is any possibility this FixedInputPortValue has been accessed
since the last time this method was called.

Template parameter ``T``:
    Scalar type of the input port's vector value. Must match the type
    associated with this port.)""";
        } GetMutableVectorData;
        // Symbol: drake::systems::FixedInputPortValue::get_owning_context
        struct /* get_owning_context */ {
          // Source: drake/systems/framework/fixed_input_port_value.h
          const char* doc =
R"""(Returns a const reference to the context that owns this object.)""";
        } get_owning_context;
        // Symbol: drake::systems::FixedInputPortValue::get_value
        struct /* get_value */ {
          // Source: drake/systems/framework/fixed_input_port_value.h
          const char* doc =
R"""(Returns a reference to the contained abstract value.)""";
        } get_value;
        // Symbol: drake::systems::FixedInputPortValue::get_vector_value
        struct /* get_vector_value */ {
          // Source: drake/systems/framework/fixed_input_port_value.h
          const char* doc =
R"""(Returns a reference to the contained ``BasicVector<T>`` or throws an
exception if this doesn't contain an object of that type.)""";
        } get_vector_value;
        // Symbol: drake::systems::FixedInputPortValue::serial_number
        struct /* serial_number */ {
          // Source: drake/systems/framework/fixed_input_port_value.h
          const char* doc =
R"""(Returns the serial number of the contained value. This counts up every
time the contained value changes, or when mutable access is granted.)""";
        } serial_number;
        // Symbol: drake::systems::FixedInputPortValue::ticket
        struct /* ticket */ {
          // Source: drake/systems/framework/fixed_input_port_value.h
          const char* doc =
R"""(Returns the ticket used to find the associated DependencyTracker.)""";
        } ticket;
      } FixedInputPortValue;
      // Symbol: drake::systems::InputPort
      struct /* InputPort */ {
        // Source: drake/systems/framework/input_port.h
        const char* doc =
R"""(An InputPort is a System resource that describes the kind of input a
System accepts, on a given port. It does not directly contain any
runtime input port data; that is always contained in a Context. The
actual value will be either the value of an OutputPort to which this
is connected, or a fixed value set in a Context.)""";
        // Symbol: drake::systems::InputPort::Eval
        struct /* Eval */ {
          // Source: drake/systems/framework/input_port.h
          const char* doc = R"""()""";
        } Eval;
        // Symbol: drake::systems::InputPort::FixValue
        struct /* FixValue */ {
          // Source: drake/systems/framework/input_port.h
          const char* doc =
R"""(Provides a fixed value for this InputPort in the given Context. If the
port is already connected, this value will override the connected
source value. (By "connected" we mean that the port appeared in a
DiagramBuilder∷Connect() call.)

For vector-valued input ports, you can provide an Eigen vector
expression, a BasicVector object, or a scalar (treated as a Vector1).
In each of these cases the value is copied into a
``Value<BasicVector>``. If the original value was a
BasicVector-derived object, its concrete type is maintained although
the stored type is still ``Value<BasicVector>``. The supplied vector
must have the right size for the vector port or an RuntimeError is
thrown.

For abstract-valued input ports, you can provide any ValueType that is
compatible with the model type provided when the port was declared. If
the type has a copy constructor it will be copied into a
``Value<ValueType>`` object for storage. Otherwise it must have an
accessible ``Clone()`` method and it is stored using the type returned
by that method, which must be ValueType or a base class of ValueType.
Eigen objects and expressions are not accepted directly, but you can
store then in abstract ports by providing an already-abstract object
like ``Value<MatrixXd>(your_matrix)``.

The returned FixedInputPortValue reference may be used to modify the
input port's value subsequently using the appropriate
FixedInputPortValue method, which will ensure that cache invalidation
notifications are delivered.

Template parameter ``ValueType``:
    The type of the supplied ``value`` object. This will be inferred
    so no template argument need be specified. The type must be copy
    constructible or have an accessible ``Clone()`` method.

Parameter ``context``:
    A Context that is compatible with the System that owns this port.

Parameter ``value``:
    The fixed value for this port. Must be convertible to the input
    port's data type.

Returns:
    a reference to the FixedInputPortValue object in the Context that
    contains this port's value.

Precondition:
    ``context`` is compatible with the System that owns this
    InputPort.

Precondition:
    ``value`` is compatible with this InputPort's data type.)""";
        } FixValue;
        // Symbol: drake::systems::InputPort::HasValue
        struct /* HasValue */ {
          // Source: drake/systems/framework/input_port.h
          const char* doc =
R"""(Returns true iff this port is connected or has had a fixed value
provided in the given Context. Beware that at the moment, this could
be an expensive operation, because the value is brought up-to-date as
part of this operation.)""";
        } HasValue;
        // Symbol: drake::systems::InputPort::InputPort<T>
        struct /* ctor */ {
          // Source: drake/systems/framework/input_port.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::systems::InputPort::get_system
        struct /* get_system */ {
          // Source: drake/systems/framework/input_port.h
          const char* doc =
R"""(Returns a reference to the System that owns this input port. Note that
for a Diagram input port this will be the Diagram, not the LeafSystem
whose input port was exported.)""";
        } get_system;
      } InputPort;
      // Symbol: drake::systems::InputPortBase
      struct /* InputPortBase */ {
        // Source: drake/systems/framework/input_port_base.h
        const char* doc =
R"""(An InputPort is a System resource that describes the kind of input a
System accepts, on a given port. It does not directly contain any
runtime input port data; that is always contained in a Context. The
actual value will be either the value of an OutputPort to which this
is connected, or a fixed value set in a Context.

InputPortBase is the scalar type-independent part of an InputPort.)""";
        // Symbol: drake::systems::InputPortBase::Allocate
        struct /* Allocate */ {
          // Source: drake/systems/framework/input_port_base.h
          const char* doc =
R"""(Allocates a concrete object suitable for holding the value to be
provided by this input port, and returns that as an AbstractValue. The
returned object will never be null.)""";
        } Allocate;
        // Symbol: drake::systems::InputPortBase::DoEvalOptional
        struct /* DoEvalOptional */ {
          // Source: drake/systems/framework/input_port_base.h
          const char* doc =
R"""(Evaluate this port; returns nullptr if the port is not connected.)""";
        } DoEvalOptional;
        // Symbol: drake::systems::InputPortBase::DoEvalRequired
        struct /* DoEvalRequired */ {
          // Source: drake/systems/framework/input_port_base.h
          const char* doc =
R"""(Evaluate this port; throws an exception if the port is not connected.)""";
        } DoEvalRequired;
        // Symbol: drake::systems::InputPortBase::EvalAbstractCallback
        struct /* EvalAbstractCallback */ {
          // Source: drake/systems/framework/input_port_base.h
          const char* doc =
R"""(Signature of a function suitable for returning the cached value of a
particular input port. Will return nullptr if the port is not
connected.)""";
        } EvalAbstractCallback;
        // Symbol: drake::systems::InputPortBase::InputPortBase
        struct /* ctor */ {
          // Source: drake/systems/framework/input_port_base.h
          const char* doc =
R"""((Internal use only) Provides derived classes the ability to set the
base class members at construction.

Parameter ``owning_system``:
    The System that owns this input port.

Parameter ``owning_system_id``:
    The ID of owning_system.

Parameter ``name``:
    A name for the port. Input port names should be non-empty and
    unique within a single System.

Parameter ``index``:
    The index to be assigned to this InputPort.

Parameter ``ticket``:
    The DependencyTicket to be assigned to this InputPort.

Parameter ``data_type``:
    Whether the port described is vector- or abstract-valued.

Parameter ``size``:
    If the port described is vector-valued, the number of elements.
    Ignored for abstract-valued ports.

Parameter ``random_type``:
    Input ports may optionally be labeled as random, if the port is
    intended to model a random-source "noise" or "disturbance" input.)""";
        } ctor;
        // Symbol: drake::systems::InputPortBase::ThrowRequiredMissing
        struct /* ThrowRequiredMissing */ {
          // Source: drake/systems/framework/input_port_base.h
          const char* doc =
R"""(Throws an exception that this port is not connected, but was expected
to be connected (i.e., an Eval caller expected that it was always
connected).)""";
        } ThrowRequiredMissing;
        // Symbol: drake::systems::InputPortBase::get_index
        struct /* get_index */ {
          // Source: drake/systems/framework/input_port_base.h
          const char* doc =
R"""(Returns the index of this input port within the owning System. For a
Diagram, this will be the index within the Diagram, *not* the index
within a LeafSystem whose input port was exported.)""";
        } get_index;
        // Symbol: drake::systems::InputPortBase::get_random_type
        struct /* get_random_type */ {
          // Source: drake/systems/framework/input_port_base.h
          const char* doc =
R"""(Returns the RandomDistribution if this is a random port.)""";
        } get_random_type;
        // Symbol: drake::systems::InputPortBase::is_random
        struct /* is_random */ {
          // Source: drake/systems/framework/input_port_base.h
          const char* doc = R"""(Returns true if this is a random port.)""";
        } is_random;
      } InputPortBase;
      // Symbol: drake::systems::InputPortIndex
      struct /* InputPortIndex */ {
        // Source: drake/systems/framework/framework_common.h
        const char* doc =
R"""(Serves as the local index for the input ports of a given System. The
indexes used by a subsystem and its corresponding subcontext are the
same.)""";
      } InputPortIndex;
      // Symbol: drake::systems::InputPortSelection
      struct /* InputPortSelection */ {
        // Source: drake/systems/framework/framework_common.h
        const char* doc =
R"""(Intended for use in e.g. variant<InputPortSelection, InputPortIndex>
for algorithms that support optional and/or default port indices.)""";
        // Symbol: drake::systems::InputPortSelection::kNoInput
        struct /* kNoInput */ {
          // Source: drake/systems/framework/framework_common.h
          const char* doc = R"""()""";
        } kNoInput;
        // Symbol: drake::systems::InputPortSelection::kUseFirstInputIfItExists
        struct /* kUseFirstInputIfItExists */ {
          // Source: drake/systems/framework/framework_common.h
          const char* doc = R"""()""";
        } kUseFirstInputIfItExists;
      } InputPortSelection;
      // Symbol: drake::systems::LeafCompositeEventCollection
      struct /* LeafCompositeEventCollection */ {
        // Source: drake/systems/framework/event_collection.h
        const char* doc =
R"""(A CompositeEventCollection for a LeafSystem. i.e.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    PublishEvent<T>: {event1i, ...}
      DiscreteUpdateEvent<T>: {event2i, ...}
      UnrestrictedUpdateEvent<T>: {event3i, ...}

.. raw:: html

    </details>)""";
        // Symbol: drake::systems::LeafCompositeEventCollection::LeafCompositeEventCollection<T>
        struct /* ctor */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::systems::LeafCompositeEventCollection::get_discrete_update_events
        struct /* get_discrete_update_events */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Returns a const reference to the collection of discrete update events.)""";
        } get_discrete_update_events;
        // Symbol: drake::systems::LeafCompositeEventCollection::get_publish_events
        struct /* get_publish_events */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Returns a const reference to the collection of publish events.)""";
        } get_publish_events;
        // Symbol: drake::systems::LeafCompositeEventCollection::get_unrestricted_update_events
        struct /* get_unrestricted_update_events */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Returns a const reference to the collection of unrestricted update
events.)""";
        } get_unrestricted_update_events;
      } LeafCompositeEventCollection;
      // Symbol: drake::systems::LeafContext
      struct /* LeafContext */ {
        // Source: drake/systems/framework/leaf_context.h
        const char* doc =
R"""(LeafContext contains all prerequisite data necessary to uniquely
determine the results of computations performed by the associated
LeafSystem.

See also:
    Context for more information.)""";
        // Symbol: drake::systems::LeafContext::DoCloneState
        struct /* DoCloneState */ {
          // Source: drake/systems/framework/leaf_context.h
          const char* doc = R"""()""";
        } DoCloneState;
        // Symbol: drake::systems::LeafContext::DoCloneWithoutPointers
        struct /* DoCloneWithoutPointers */ {
          // Source: drake/systems/framework/leaf_context.h
          const char* doc =
R"""(Derived classes should reimplement and replace this; don't recursively
invoke it.)""";
        } DoCloneWithoutPointers;
        // Symbol: drake::systems::LeafContext::LeafContext<T>
        struct /* ctor */ {
          // Source: drake/systems/framework/leaf_context.h
          const char* doc_copy =
R"""(Protected copy constructor takes care of the local data members and
all base class members, but doesn't update base class pointers so is
not a complete copy.)""";
        } ctor;
      } LeafContext;
      // Symbol: drake::systems::LeafEventCollection
      struct /* LeafEventCollection */ {
        // Source: drake/systems/framework/event_collection.h
        const char* doc =
R"""(A concrete class that holds all simultaneous *homogeneous* events for
a LeafSystem.

End users should never need to use or know about this class. It is for
internal use only.)""";
        // Symbol: drake::systems::LeafEventCollection::AddEvent
        struct /* AddEvent */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Add ``event`` to the existing collection.)""";
        } AddEvent;
        // Symbol: drake::systems::LeafEventCollection::Clear
        struct /* Clear */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Removes all events from this collection.)""";
        } Clear;
        // Symbol: drake::systems::LeafEventCollection::DoAddToEnd
        struct /* DoAddToEnd */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(All events in ``other_collection`` are concatenated to this.

Here is an example. Suppose this collection stores the following
events:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    EventType: {event1, event2, event3}

.. raw:: html

    </details>

``other_collection`` has:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    EventType: {event4}

.. raw:: html

    </details>

After calling DoAddToEnd(other_collection), ``this`` stores:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    EventType: {event1, event2, event3, event4}

.. raw:: html

    </details>

Raises:
    RuntimeError if ``other_collection`` is not an instance of
    LeafEventCollection.)""";
        } DoAddToEnd;
        // Symbol: drake::systems::LeafEventCollection::HasEvents
        struct /* HasEvents */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Returns ``True`` if and only if this collection is nonempty.)""";
        } HasEvents;
        // Symbol: drake::systems::LeafEventCollection::LeafEventCollection<EventType>
        struct /* ctor */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc = R"""(Constructor.)""";
        } ctor;
        // Symbol: drake::systems::LeafEventCollection::MakeForcedEventCollection
        struct /* MakeForcedEventCollection */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Static method that generates a LeafEventCollection with exactly one
event with no optional attribute, data or callback, and trigger type
kForced.)""";
        } MakeForcedEventCollection;
        // Symbol: drake::systems::LeafEventCollection::Reserve
        struct /* Reserve */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Reserve storage for at least ``capacity`` events. At construction,
there will be at least ``kDefaultCapacity``; use this method to
reserve more.)""";
        } Reserve;
        // Symbol: drake::systems::LeafEventCollection::get_events
        struct /* get_events */ {
          // Source: drake/systems/framework/event_collection.h
          const char* doc =
R"""(Returns a const reference to the vector of const pointers to all of
the events.)""";
        } get_events;
      } LeafEventCollection;
      // Symbol: drake::systems::LeafOutputPort
      struct /* LeafOutputPort */ {
        // Source: drake/systems/framework/leaf_output_port.h
        const char* doc =
R"""((Advanced.) Implements an output port whose value is managed by a
cache entry in the same LeafSystem as the port. This is intended for
internal use in implementing the DeclareOutputPort() variants in
LeafSystem.)""";
        // Symbol: drake::systems::LeafOutputPort::AllocCallback
        struct /* AllocCallback */ {
          // Source: drake/systems/framework/leaf_output_port.h
          const char* doc =
R"""(Signature of a function suitable for allocating an object that can
hold a value of a particular output port. The result is returned as an
AbstractValue even if this is a vector-valued port.)""";
        } AllocCallback;
        // Symbol: drake::systems::LeafOutputPort::CalcCallback
        struct /* CalcCallback */ {
          // Source: drake/systems/framework/leaf_output_port.h
          const char* doc =
R"""(Signature of a function suitable for calculating a value of a
particular output port, given a place to put the value.)""";
        } CalcCallback;
        // Symbol: drake::systems::LeafOutputPort::CalcVectorCallback
        struct /* CalcVectorCallback */ {
          // Source: drake/systems/framework/leaf_output_port.h
          const char* doc =
R"""(Signature of a function suitable for calculating a value of a
particular vector-valued output port, given a place to put the value.)""";
        } CalcVectorCallback;
        // Symbol: drake::systems::LeafOutputPort::LeafOutputPort<T>
        struct /* ctor */ {
          // Source: drake/systems/framework/leaf_output_port.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::systems::LeafOutputPort::cache_entry
        struct /* cache_entry */ {
          // Source: drake/systems/framework/leaf_output_port.h
          const char* doc =
R"""(Returns the cache entry associated with this output port.)""";
        } cache_entry;
        // Symbol: drake::systems::LeafOutputPort::disable_caching_by_default
        struct /* disable_caching_by_default */ {
          // Source: drake/systems/framework/leaf_output_port.h
          const char* doc =
R"""((Debugging) Specifies that caching should be disabled for this output
port when a Context is first allocated. This is useful if you have
observed different behavior with caching on or off and would like to
determine if the problem is caused by this port.

See also:
    CacheEntry∷disable_caching_by_default())""";
        } disable_caching_by_default;
      } LeafOutputPort;
      // Symbol: drake::systems::LeafSystem
      struct /* LeafSystem */ {
        // Source: drake/systems/framework/leaf_system.h
        const char* doc =
R"""(A superclass template that extends System with some convenience
utilities that are not applicable to Diagrams.)""";
        // Symbol: drake::systems::LeafSystem::AddTriggeredWitnessFunctionToCompositeEventCollection
        struct /* AddTriggeredWitnessFunctionToCompositeEventCollection */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc = R"""()""";
        } AddTriggeredWitnessFunctionToCompositeEventCollection;
        // Symbol: drake::systems::LeafSystem::AllocateAbstractState
        struct /* AllocateAbstractState */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc =
R"""(Returns a copy of the states declared in DeclareAbstractState() calls.)""";
        } AllocateAbstractState;
        // Symbol: drake::systems::LeafSystem::AllocateContext
        struct /* AllocateContext */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc =
R"""(Shadows System<T>∷AllocateContext to provide a more concrete return
type LeafContext<T>.)""";
        } AllocateContext;
        // Symbol: drake::systems::LeafSystem::AllocateContinuousState
        struct /* AllocateContinuousState */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc =
R"""(Returns a copy of the state declared in the most recent
DeclareContinuousState() call, or else a zero-sized state if that
method has never been called.)""";
        } AllocateContinuousState;
        // Symbol: drake::systems::LeafSystem::AllocateDiscreteState
        struct /* AllocateDiscreteState */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc =
R"""(Returns a copy of the states declared in DeclareDiscreteState() calls.)""";
        } AllocateDiscreteState;
        // Symbol: drake::systems::LeafSystem::AllocateDiscreteVariables
        struct /* AllocateDiscreteVariables */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc = R"""()""";
        } AllocateDiscreteVariables;
        // Symbol: drake::systems::LeafSystem::AllocateForcedDiscreteUpdateEventCollection
        struct /* AllocateForcedDiscreteUpdateEventCollection */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc = R"""()""";
        } AllocateForcedDiscreteUpdateEventCollection;
        // Symbol: drake::systems::LeafSystem::AllocateForcedPublishEventCollection
        struct /* AllocateForcedPublishEventCollection */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc = R"""()""";
        } AllocateForcedPublishEventCollection;
        // Symbol: drake::systems::LeafSystem::AllocateForcedUnrestrictedUpdateEventCollection
        struct /* AllocateForcedUnrestrictedUpdateEventCollection */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc = R"""()""";
        } AllocateForcedUnrestrictedUpdateEventCollection;
        // Symbol: drake::systems::LeafSystem::AllocateParameters
        struct /* AllocateParameters */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc =
R"""(Returns a copy of the parameters declared in DeclareNumericParameter()
and DeclareAbstractParameter() calls.)""";
        } AllocateParameters;
        // Symbol: drake::systems::LeafSystem::AllocateTimeDerivatives
        struct /* AllocateTimeDerivatives */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc = R"""()""";
        } AllocateTimeDerivatives;
        // Symbol: drake::systems::LeafSystem::DeclareAbstractInputPort
        struct /* DeclareAbstractInputPort */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc =
R"""(Declares an abstract-valued input port using the given
``model_value``. This is the best way to declare LeafSystem abstract
input ports.

Any port connected to this input, and any call to FixValue for this
input, must provide for values whose type matches this
``model_value``.

See also:
    System∷DeclareInputPort() for more information.)""";
        } DeclareAbstractInputPort;
        // Symbol: drake::systems::LeafSystem::DeclareAbstractOutputPort
        struct /* DeclareAbstractOutputPort */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc_4args_stdvariant_constOutputType_voidMySystemconstContextOutputTypeconst_stdset =
R"""(Declares an abstract-valued output port by specifying a model value of
concrete type ``OutputType`` and a calculator function that is a class
member function (method) with signature:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    void MySystem∷CalcOutputValue(const Context<T>&, OutputType*) const;

.. raw:: html

    </details>

where ``MySystem`` must be a class derived from ``LeafSystem<T>``.
`OutputType` must be such that ``Value<OutputType>`` is permitted.
Template arguments will be deduced and do not need to be specified.

See also:
    drake∷Value)""";
          // Source: drake/systems/framework/leaf_system.h
          const char* doc_3args_stdvariant_voidMySystemconstContextOutputTypeconst_stdset =
R"""(Declares an abstract-valued output port by specifying only a
calculator function that is a class member function (method) with
signature:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    void MySystem∷CalcOutputValue(const Context<T>&, OutputType*) const;

.. raw:: html

    </details>

where ``MySystem`` is a class derived from ``LeafSystem<T>``.
`OutputType` is a concrete type such that ``Value<OutputType>`` is
permitted, and must be default constructible, so that we can create a
model value using ``Value<OutputType>{}`` (value initialized so
numerical types will be zeroed in the model). Template arguments will
be deduced and do not need to be specified.

Note:
    The default constructor will be called once immediately, and
    subsequent allocations will just copy the model value without
    invoking the constructor again. If you want the constructor
    invoked again at each allocation (not common), use one of the
    other signatures to explicitly provide a method for the allocator
    to call; that method can then invoke the ``OutputType`` default
    constructor.

See also:
    drake∷Value)""";
          // Source: drake/systems/framework/leaf_system.h
          const char* doc_4args_name_alloc_calc_prerequisites_of_calc =
R"""((Advanced) Declares an abstract-valued output port using the given
allocator and calculator functions provided in their most generic
forms. If you have a member function available use one of the other
signatures.

Parameter ``alloc``:
    Callback function that allocates storage for the value. It takes
    no arguments and must return an AbstractValue.

Parameter ``calc``:
    Callback function that computes the value. It takes two arguments
    (context, value) and does not return anything; instead, it should
    mutate the AbstractValue object pointed to by ``value`` with the
    new result.

See also:
    LeafOutputPort∷AllocCallback, LeafOutputPort∷CalcCallback)""";
        } DeclareAbstractOutputPort;
        // Symbol: drake::systems::LeafSystem::DeclareAbstractParameter
        struct /* DeclareAbstractParameter */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc =
R"""(Declares an abstract parameter using the given ``model_value``.
LeafSystem's default implementation of SetDefaultParameters() will
reset parameters to their model values. Returns the index of the new
parameter.)""";
        } DeclareAbstractParameter;
        // Symbol: drake::systems::LeafSystem::DeclareAbstractState
        struct /* DeclareAbstractState */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc =
R"""(Declares an abstract state variable and provides a model value for it.
A Context obtained with CreateDefaultContext() will contain this
abstract state variable initially set to a clone of the
``model_value`` given here. The actual concrete type is always
preserved.

Parameter ``model_value``:
    The abstract state model value to be cloned as needed.

Returns:
    index of the declared abstract state variable.)""";
        } DeclareAbstractState;
        // Symbol: drake::systems::LeafSystem::DeclareContinuousState
        struct /* DeclareContinuousState */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc_1args_num_state_variables =
R"""(Declares that this System should reserve continuous state with
``num_state_variables`` state variables, which have no second-order
structure.

Returns:
    index of the declared state (currently always zero).)""";
          // Source: drake/systems/framework/leaf_system.h
          const char* doc_3args_num_q_num_v_num_z =
R"""(Declares that this System should reserve continuous state with
``num_q`` generalized positions, ``num_v`` generalized velocities, and
``num_z`` miscellaneous state variables.

Returns:
    index of the declared state (currently always zero).)""";
          // Source: drake/systems/framework/leaf_system.h
          const char* doc_1args_model_vector =
R"""(Declares that this System should reserve continuous state with
``model_vector``.size() miscellaneous state variables, stored in a
vector cloned from ``model_vector``.

Returns:
    index of the declared state (currently always zero).)""";
          // Source: drake/systems/framework/leaf_system.h
          const char* doc_4args_model_vector_num_q_num_v_num_z =
R"""(Declares that this System should reserve continuous state with
``num_q`` generalized positions, ``num_v`` generalized velocities, and
``num_z`` miscellaneous state variables, stored in a vector cloned
from ``model_vector``. Aborts if ``model_vector`` has the wrong size.
If the ``model_vector`` declares any VectorBase∷GetElementBounds()
constraints, they will be re-declared as inequality constraints on
this system (see DeclareInequalityConstraint()).

Returns:
    index of the declared state (currently always zero).)""";
        } DeclareContinuousState;
        // Symbol: drake::systems::LeafSystem::DeclareDiscreteState
        struct /* DeclareDiscreteState */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc_1args_model_vector =
R"""(Declares a discrete state group with ``model_vector``.size() state
variables, stored in a vector cloned from ``model_vector`` (preserving
the concrete type and value).)""";
          // Source: drake/systems/framework/leaf_system.h
          const char* doc_1args_vector =
R"""(Declares a discrete state group with ``vector``.size() state
variables, stored in a BasicVector initialized with the contents of
``vector``.)""";
          // Source: drake/systems/framework/leaf_system.h
          const char* doc_1args_num_state_variables =
R"""(Declares a discrete state group with ``num_state_variables`` state
variables, stored in a BasicVector initialized to be all-zero. If you
want non-zero initial values, use an alternate DeclareDiscreteState()
signature that accepts a ``model_vector`` parameter.

Precondition:
    ``num_state_variables`` must be non-negative.)""";
        } DeclareDiscreteState;
        // Symbol: drake::systems::LeafSystem::DeclareEqualityConstraint
        struct /* DeclareEqualityConstraint */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc_3args_voidMySystemconstContextconst_int_stdstring =
R"""(Declares a system constraint of the form f(context) = 0 by specifying
a member function to use to calculate the (VectorX) constraint value
with a signature:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    void MySystem∷CalcConstraint(const Context<T>&, VectorX<T>*) const;

.. raw:: html

    </details>

Parameter ``count``:
    is the dimension of the VectorX output.

Parameter ``description``:
    should be a human-readable phrase.

Returns:
    The index of the constraint. Template arguments will be deduced
    and do not need to be specified.

See also:
    SystemConstraint<T> for more information about the meaning of
    these constraints.)""";
          // Source: drake/systems/framework/leaf_system.h
          const char* doc_3args_calc_count_description =
R"""(Declares a system constraint of the form f(context) = 0 by specifying
a std∷function to use to calculate the (Vector) constraint value with
a signature:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    void CalcConstraint(const Context<T>&, VectorX<T>*);

.. raw:: html

    </details>

Parameter ``count``:
    is the dimension of the VectorX output.

Parameter ``description``:
    should be a human-readable phrase.

Returns:
    The index of the constraint.

See also:
    SystemConstraint<T> for more information about the meaning of
    these constraints.)""";
        } DeclareEqualityConstraint;
        // Symbol: drake::systems::LeafSystem::DeclareForcedDiscreteUpdateEvent
        struct /* DeclareForcedDiscreteUpdateEvent */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc =
R"""(Declares a function that is called whenever a user directly calls
CalcForcedDiscreteVariableUpdate(const Context&, DiscreteValues<T>*).
Multiple calls to DeclareForcedDiscreteUpdateEvent() will cause
multiple handlers to be called upon a call to
CalcForcedDiscreteVariableUpdate(); these handlers will be called with
the same const Context in arbitrary order. The handler should be a
class member function (method) with this signature:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    EventStatus MySystem∷MyDiscreteVariableUpdates(const Context<T>&,
    DiscreteValues<T>*);

.. raw:: html

    </details>

where ``MySystem`` is a class derived from ``LeafSystem<T>`` and the
method name is arbitrary.

See declare_forced_events "Declare forced events" for more
information.

Precondition:
    ``this`` must be dynamic_cast-able to MySystem.

Precondition:
    ``update`` must not be null.)""";
        } DeclareForcedDiscreteUpdateEvent;
        // Symbol: drake::systems::LeafSystem::DeclareForcedPublishEvent
        struct /* DeclareForcedPublishEvent */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc =
R"""(Declares a function that is called whenever a user directly calls
ForcedPublish(const Context&). Multiple calls to
DeclareForcedPublishEvent() will cause multiple handlers to be called
upon a call to ForcedPublish(); these handlers which will be called
with the same const Context in arbitrary order. The handler should be
a class member function (method) with this signature:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    EventStatus MySystem∷MyPublish(const Context<T>&) const;

.. raw:: html

    </details>

where ``MySystem`` is a class derived from ``LeafSystem<T>`` and the
method name is arbitrary.

See declare_forced_events "Declare forced events" for more
information.

Precondition:
    ``this`` must be dynamic_cast-able to MySystem.

Precondition:
    ``publish`` must not be null.)""";
        } DeclareForcedPublishEvent;
        // Symbol: drake::systems::LeafSystem::DeclareForcedUnrestrictedUpdateEvent
        struct /* DeclareForcedUnrestrictedUpdateEvent */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc =
R"""(Declares a function that is called whenever a user directly calls
CalcForcedUnrestrictedUpdate(const Context&, State<T>*). Multiple
calls to DeclareForcedUnrestrictedUpdateEvent() will cause multiple
handlers to be called upon a call to CalcForcedUnrestrictedUpdate();
these handlers which will be called with the same const Context in
arbitrary order.The handler should be a class member function (method)
with this signature:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    EventStatus MySystem∷MyUnrestrictedUpdates(const Context<T>&,
    State<T>*);

.. raw:: html

    </details>

where ``MySystem`` is a class derived from ``LeafSystem<T>`` and the
method name is arbitrary.

See declare_forced_events "Declare forced events" for more
information.

Precondition:
    ``this`` must be dynamic_cast-able to MySystem.

Precondition:
    ``update`` must not be null.)""";
        } DeclareForcedUnrestrictedUpdateEvent;
        // Symbol: drake::systems::LeafSystem::DeclareImplicitTimeDerivativesResidualSize
        struct /* DeclareImplicitTimeDerivativesResidualSize */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc =
R"""((Advanced) Overrides the default size for the implicit time
derivatives residual. If no value is set, the default size is
n=num_continuous_states().

Parameter ``n``:
    The size of the residual vector output argument of
    System∷CalcImplicitTimeDerivativesResidual(). If n <= 0 restore to
    the default, num_continuous_states().

See also:
    implicit_time_derivatives_residual_size()

See also:
    System∷CalcImplicitTimeDerivativesResidual())""";
        } DeclareImplicitTimeDerivativesResidualSize;
        // Symbol: drake::systems::LeafSystem::DeclareInequalityConstraint
        struct /* DeclareInequalityConstraint */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc_3args_voidMySystemconstContextconst_SystemConstraintBounds_stdstring =
R"""(Declares a system constraint of the form bounds.lower() <=
calc(context) <= bounds.upper() by specifying a member function to use
to calculate the (VectorX) constraint value with a signature:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    void MySystem∷CalcConstraint(const Context<T>&, VectorX<T>*) const;

.. raw:: html

    </details>

Parameter ``description``:
    should be a human-readable phrase.

Returns:
    The index of the constraint. Template arguments will be deduced
    and do not need to be specified.

See also:
    SystemConstraint<T> for more information about the meaning of
    these constraints.)""";
          // Source: drake/systems/framework/leaf_system.h
          const char* doc_3args_calc_bounds_description =
R"""(Declares a system constraint of the form bounds.lower() <=
calc(context) <= bounds.upper() by specifying a std∷function to use to
calculate the (Vector) constraint value with a signature:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    void CalcConstraint(const Context<T>&, VectorX<T>*);

.. raw:: html

    </details>

Parameter ``description``:
    should be a human-readable phrase.

Returns:
    The index of the constraint.

See also:
    SystemConstraint<T> for more information about the meaning of
    these constraints.)""";
        } DeclareInequalityConstraint;
        // Symbol: drake::systems::LeafSystem::DeclareInitializationDiscreteUpdateEvent
        struct /* DeclareInitializationDiscreteUpdateEvent */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc =
R"""(Declares that a DiscreteUpdate event should occur at initialization
and that it should invoke the given event handler method. The handler
should be a class member function (method) with this signature:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    EventStatus MySystem∷MyUpdate(const Context<T>&,
    DiscreteValues<T>*) const;

.. raw:: html

    </details>

where ``MySystem`` is a class derived from ``LeafSystem<T>`` and the
method name is arbitrary.

See declare_initialization_events "Declare initialization events" for
more information.

Precondition:
    ``this`` must be dynamic_cast-able to MySystem.

Precondition:
    ``update`` must not be null.

See also:
    DeclareInitializationPublishEvent()

See also:
    DeclareInitializationUnrestrictedUpdateEvent()

See also:
    DeclareInitializationEvent())""";
        } DeclareInitializationDiscreteUpdateEvent;
        // Symbol: drake::systems::LeafSystem::DeclareInitializationEvent
        struct /* DeclareInitializationEvent */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc =
R"""((Advanced) Declares that a particular Event object should be
dispatched at initialization. This is the most general form for
declaring initialization events and most users should use one of the
other methods in this group instead.

See also:
    DeclareInitializationPublishEvent()

See also:
    DeclareInitializationDiscreteUpdateEvent()

See also:
    DeclareInitializationUnrestrictedUpdateEvent()

See declare_initialization_events "Declare initialization events" for
more information.

Depending on the type of ``event``, on initialization it will be
passed to the Publish, DiscreteUpdate, or UnrestrictedUpdate event
dispatcher. If the ``event`` object contains a handler function,
Drake's default dispatchers will invoke that handler. If not, then no
further action is taken. Thus an ``event`` with no handler has no
effect unless its dispatcher has been overridden. We strongly
recommend that you *do not* override the dispatcher and instead *do*
supply a handler.

The given ``event`` object is deep-copied (cloned), and the copy is
stored internally so you do not need to keep the object around after
this call.

Precondition:
    `event`'s associated trigger type must be TriggerType∷kUnknown or
    already set to TriggerType∷kInitialization.)""";
        } DeclareInitializationEvent;
        // Symbol: drake::systems::LeafSystem::DeclareInitializationPublishEvent
        struct /* DeclareInitializationPublishEvent */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc =
R"""(Declares that a Publish event should occur at initialization and that
it should invoke the given event handler method. The handler should be
a class member function (method) with this signature:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    EventStatus MySystem∷MyPublish(const Context<T>&) const;

.. raw:: html

    </details>

where ``MySystem`` is a class derived from ``LeafSystem<T>`` and the
method name is arbitrary.

See declare_initialization_events "Declare initialization events" for
more information.

Precondition:
    ``this`` must be dynamic_cast-able to MySystem.

Precondition:
    ``publish`` must not be null.

See also:
    DeclareInitializationDiscreteUpdateEvent()

See also:
    DeclareInitializationUnrestrictedUpdateEvent()

See also:
    DeclareInitializationEvent())""";
        } DeclareInitializationPublishEvent;
        // Symbol: drake::systems::LeafSystem::DeclareInitializationUnrestrictedUpdateEvent
        struct /* DeclareInitializationUnrestrictedUpdateEvent */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc =
R"""(Declares that an UnrestrictedUpdate event should occur at
initialization and that it should invoke the given event handler
method. The handler should be a class member function (method) with
this signature:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    EventStatus MySystem∷MyUpdate(const Context<T>&,
    State<T>*) const;

.. raw:: html

    </details>

where ``MySystem`` is a class derived from ``LeafSystem<T>`` and the
method name is arbitrary.

See declare_initialization_events "Declare initialization events" for
more information.

Precondition:
    ``this`` must be dynamic_cast-able to MySystem.

Precondition:
    ``update`` must not be null.

See also:
    DeclareInitializationPublishEvent()

See also:
    DeclareInitializationDiscreteUpdateEvent()

See also:
    DeclareInitializationEvent())""";
        } DeclareInitializationUnrestrictedUpdateEvent;
        // Symbol: drake::systems::LeafSystem::DeclareNumericParameter
        struct /* DeclareNumericParameter */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc =
R"""(Declares a numeric parameter using the given ``model_vector``.
LeafSystem's default implementation of SetDefaultParameters() will
reset parameters to their model vectors. If the ``model_vector``
declares any VectorBase∷GetElementBounds() constraints, they will be
re-declared as inequality constraints on this system (see
DeclareInequalityConstraint()). Returns the index of the new
parameter.)""";
        } DeclareNumericParameter;
        // Symbol: drake::systems::LeafSystem::DeclarePerStepDiscreteUpdateEvent
        struct /* DeclarePerStepDiscreteUpdateEvent */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc =
R"""(Declares that a DiscreteUpdate event should occur at the start of
every trajectory-advancing step and that it should invoke the given
event handler method. The handler should be a class member function
(method) with this signature:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    EventStatus MySystem∷MyUpdate(const Context<T>&,
    DiscreteValues<T>*) const;

.. raw:: html

    </details>

where ``MySystem`` is a class derived from ``LeafSystem<T>`` and the
method name is arbitrary.

See declare_per-step_events "Declare per-step events" for more
information.

Precondition:
    ``this`` must be dynamic_cast-able to MySystem.

Precondition:
    ``update`` must not be null.

See also:
    DeclarePerStepPublishEvent()

See also:
    DeclarePerStepUnrestrictedUpdateEvent()

See also:
    DeclarePerStepEvent())""";
        } DeclarePerStepDiscreteUpdateEvent;
        // Symbol: drake::systems::LeafSystem::DeclarePerStepEvent
        struct /* DeclarePerStepEvent */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc =
R"""((Advanced) Declares that a particular Event object should be
dispatched at every trajectory-advancing step. Publish events are
dispatched at the end of initialization and at the end of each step.
Discrete- and unrestricted update events are dispatched at the start
of each step. This is the most general form for declaring per-step
events and most users should use one of the other methods in this
group instead.

See also:
    DeclarePerStepPublishEvent()

See also:
    DeclarePerStepDiscreteUpdateEvent()

See also:
    DeclarePerStepUnrestrictedUpdateEvent()

See declare_per-step_events "Declare per-step events" for more
information.

Depending on the type of ``event``, at each step it will be passed to
the Publish, DiscreteUpdate, or UnrestrictedUpdate event dispatcher.
If the ``event`` object contains a handler function, Drake's default
dispatchers will invoke that handler. If not, then no further action
is taken. Thus an ``event`` with no handler has no effect unless its
dispatcher has been overridden. We strongly recommend that you *do
not* override the dispatcher and instead *do* supply a handler.

The given ``event`` object is deep-copied (cloned), and the copy is
stored internally so you do not need to keep the object around after
this call.

Precondition:
    `event`'s associated trigger type must be TriggerType∷kUnknown or
    already set to TriggerType∷kPerStep.)""";
        } DeclarePerStepEvent;
        // Symbol: drake::systems::LeafSystem::DeclarePerStepPublishEvent
        struct /* DeclarePerStepPublishEvent */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc =
R"""(Declares that a Publish event should occur at initialization and at
the end of every trajectory-advancing step and that it should invoke
the given event handler method. The handler should be a class member
function (method) with this signature:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    EventStatus MySystem∷MyPublish(const Context<T>&) const;

.. raw:: html

    </details>

where ``MySystem`` is a class derived from ``LeafSystem<T>`` and the
method name is arbitrary.

Warning:
    These per-step publish events are independent of the Simulator's
    optional "publish every time step" and "publish at initialization"
    features. Generally if you are declaring per-step publish events
    yourself you should turn off those Simulation options.

See declare_per-step_events "Declare per-step events" for more
information.

Precondition:
    ``this`` must be dynamic_cast-able to MySystem.

Precondition:
    ``publish`` must not be null.

See also:
    DeclarePerStepDiscreteUpdateEvent()

See also:
    DeclarePerStepUnrestrictedUpdateEvent()

See also:
    DeclarePerStepEvent()

See also:
    Simulator∷set_publish_at_initialization()

See also:
    Simulator∷set_publish_every_time_step())""";
        } DeclarePerStepPublishEvent;
        // Symbol: drake::systems::LeafSystem::DeclarePerStepUnrestrictedUpdateEvent
        struct /* DeclarePerStepUnrestrictedUpdateEvent */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc =
R"""(Declares that an UnrestrictedUpdate event should occur at the start of
every trajectory-advancing step and that it should invoke the given
event handler method. The handler should be a class member function
(method) with this signature:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    EventStatus MySystem∷MyUpdate(const Context<T>&,
    State<T>*) const;

.. raw:: html

    </details>

where ``MySystem`` is a class derived from ``LeafSystem<T>`` and the
method name is arbitrary.

See declare_per-step_events "Declare per-step events" for more
information.

Precondition:
    ``this`` must be dynamic_cast-able to MySystem.

Precondition:
    ``update`` must not be null.

See also:
    DeclarePerStepPublishEvent()

See also:
    DeclarePerStepDiscreteUpdateEvent()

See also:
    DeclarePerStepEvent())""";
        } DeclarePerStepUnrestrictedUpdateEvent;
        // Symbol: drake::systems::LeafSystem::DeclarePeriodicDiscreteUpdateEvent
        struct /* DeclarePeriodicDiscreteUpdateEvent */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc =
R"""(Declares that a DiscreteUpdate event should occur periodically and
that it should invoke the given event handler method. The handler
should be a class member function (method) with this signature:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    EventStatus MySystem∷MyUpdate(const Context<T>&,
    DiscreteValues<T>*) const;

.. raw:: html

    </details>

where ``MySystem`` is a class derived from ``LeafSystem<T>`` and the
method name is arbitrary.

See declare_periodic_events "Declare periodic events" for more
information.

Precondition:
    ``this`` must be dynamic_cast-able to MySystem.

Precondition:
    ``update`` must not be null.

See also:
    DeclarePeriodicPublishEvent()

See also:
    DeclarePeriodicUnrestrictedUpdateEvent()

See also:
    DeclarePeriodicEvent())""";
        } DeclarePeriodicDiscreteUpdateEvent;
        // Symbol: drake::systems::LeafSystem::DeclarePeriodicEvent
        struct /* DeclarePeriodicEvent */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc =
R"""((Advanced) Declares that a particular Event object should be
dispatched periodically. This is the most general form for declaring
periodic events and most users should use one of the other methods in
this group instead.

See also:
    DeclarePeriodicPublishEvent()

See also:
    DeclarePeriodicDiscreteUpdateEvent()

See also:
    DeclarePeriodicUnrestrictedUpdateEvent()

See declare_periodic_events "Declare periodic events" for more
information.

Depending on the type of ``event``, when triggered it will be passed
to the Publish, DiscreteUpdate, or UnrestrictedUpdate event
dispatcher. If the ``event`` object contains a handler function,
Drake's default dispatchers will invoke that handler. If not, then no
further action is taken. Thus an ``event`` with no handler has no
effect unless its dispatcher has been overridden. We strongly
recommend that you *do not* override the dispatcher and instead *do*
supply a handler.

The given ``event`` object is deep-copied (cloned), and the copy is
stored internally so you do not need to keep the object around after
this call.

Precondition:
    `event`'s associated trigger type must be TriggerType∷kUnknown or
    already set to TriggerType∷kPeriodic.)""";
        } DeclarePeriodicEvent;
        // Symbol: drake::systems::LeafSystem::DeclarePeriodicPublishEvent
        struct /* DeclarePeriodicPublishEvent */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc =
R"""(Declares that a Publish event should occur periodically and that it
should invoke the given event handler method. The handler should be a
class member function (method) with this signature:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    EventStatus MySystem∷MyPublish(const Context<T>&) const;

.. raw:: html

    </details>

where ``MySystem`` is a class derived from ``LeafSystem<T>`` and the
method name is arbitrary.

See declare_periodic_events "Declare periodic events" for more
information.

Precondition:
    ``this`` must be dynamic_cast-able to MySystem.

Precondition:
    ``publish`` must not be null.

See also:
    DeclarePeriodicDiscreteUpdateEvent()

See also:
    DeclarePeriodicUnrestrictedUpdateEvent()

See also:
    DeclarePeriodicEvent())""";
        } DeclarePeriodicPublishEvent;
        // Symbol: drake::systems::LeafSystem::DeclarePeriodicUnrestrictedUpdateEvent
        struct /* DeclarePeriodicUnrestrictedUpdateEvent */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc =
R"""(Declares that an UnrestrictedUpdate event should occur periodically
and that it should invoke the given event handler method. The handler
should be a class member function (method) with this signature:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    EventStatus MySystem∷MyUpdate(const Context<T>&, State<T>*) const;

.. raw:: html

    </details>

where ``MySystem`` is a class derived from ``LeafSystem<T>`` and the
method name is arbitrary.

See declare_periodic_events "Declare periodic events" for more
information.

Precondition:
    ``this`` must be dynamic_cast-able to MySystem.

Precondition:
    ``update`` must not be null.

See also:
    DeclarePeriodicPublishEvent()

See also:
    DeclarePeriodicDiscreteUpdateEvent()

See also:
    DeclarePeriodicEvent())""";
        } DeclarePeriodicUnrestrictedUpdateEvent;
        // Symbol: drake::systems::LeafSystem::DeclareStateOutputPort
        struct /* DeclareStateOutputPort */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc_continuous =
R"""(Declares a vector-valued output port whose value is the continuous
state of this system.

Parameter ``state_index``:
    must be ContinuousStateIndex(0) for now, since LeafSystem only
    supports a single continuous state group at the moment.)""";
          // Source: drake/systems/framework/leaf_system.h
          const char* doc_discrete =
R"""(Declares a vector-valued output port whose value is the given discrete
state group of this system.)""";
          // Source: drake/systems/framework/leaf_system.h
          const char* doc_abstract =
R"""(Declares an abstract-valued output port whose value is the given
abstract state of this system.)""";
        } DeclareStateOutputPort;
        // Symbol: drake::systems::LeafSystem::DeclareVectorInputPort
        struct /* DeclareVectorInputPort */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc_3args_model_vector =
R"""(Declares a vector-valued input port using the given ``model_vector``.
This is the best way to declare LeafSystem input ports that require
subclasses of BasicVector. The port's size and type will be the same
as model_vector. If the port is intended to model a random noise or
disturbance input, ``random_type`` can (optionally) be used to label
it as such. If the ``model_vector`` declares any
VectorBase∷GetElementBounds() constraints, they will be re-declared as
inequality constraints on this system (see
DeclareInequalityConstraint()).

See also:
    System∷DeclareInputPort() for more information.)""";
          // Source: drake/systems/framework/leaf_system.h
          const char* doc_3args_size =
R"""(Declares a vector-valued input port with type BasicVector and size
``size``. If the port is intended to model a random noise or
disturbance input, ``random_type`` can (optionally) be used to label
it as such.

See also:
    System∷DeclareInputPort() for more information.)""";
        } DeclareVectorInputPort;
        // Symbol: drake::systems::LeafSystem::DeclareVectorOutputPort
        struct /* DeclareVectorOutputPort */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc_4args_model_vector =
R"""((Advanced) Declares a vector-valued output port using the given
``model_vector`` and a function for calculating the port's value at
runtime. The port's size will be model_vector.size(), and the default
allocator for the port will be model_vector.Clone(). Note that this
takes the calculator function in its most generic form; if you have a
member function available use one of the other signatures.

See also:
    LeafOutputPort∷CalcVectorCallback)""";
          // Source: drake/systems/framework/leaf_system.h
          const char* doc_4args_size =
R"""((Advanced) Declares a vector-valued output port with type
BasicVector<T> and size ``size``, using the drake∷dummy_value<T>,
which is NaN when T = double. ``vector_calc_function`` is a function
for calculating the port's value at runtime. Note that this takes the
calculator function in its most generic form; if you have a member
function available use one of the other signatures.

See also:
    LeafOutputPort∷CalcVectorCallback)""";
        } DeclareVectorOutputPort;
        // Symbol: drake::systems::LeafSystem::DeprecateInputPort
        struct /* DeprecateInputPort */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc =
R"""(Flags an already-declared input port as deprecated. The first attempt
to use the port in a program will log a warning message. This function
may be called at most once for any given port.)""";
        } DeprecateInputPort;
        // Symbol: drake::systems::LeafSystem::DeprecateOutputPort
        struct /* DeprecateOutputPort */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc =
R"""(Flags an already-declared output port as deprecated. The first attempt
to use the port in a program will log a warning message. This function
may be called at most once for any given port.)""";
        } DeprecateOutputPort;
        // Symbol: drake::systems::LeafSystem::DoAllocateContext
        struct /* DoAllocateContext */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc = R"""()""";
        } DoAllocateContext;
        // Symbol: drake::systems::LeafSystem::DoCalcNextUpdateTime
        struct /* DoCalcNextUpdateTime */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc =
R"""(Computes the next update time based on the configured periodic events,
for scalar types that are arithmetic, or aborts for scalar types that
are not arithmetic. Subclasses that require aperiodic events should
override, but be sure to invoke the parent class implementation at the
start of the override if you want periodic events to continue to be
handled.

Postcondition:
    ``time`` is set to a value greater than or equal to
    ``context.get_time()`` on return.

Warning:
    If you override this method, think carefully before setting
    ``time`` to ``context.get_time()`` on return, which can
    inadvertently cause simulations of systems derived from LeafSystem
    to loop interminably. Such a loop will occur if, for example, the
    event(s) does not modify the state.)""";
        } DoCalcNextUpdateTime;
        // Symbol: drake::systems::LeafSystem::DoCalcWitnessValue
        struct /* DoCalcWitnessValue */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc = R"""()""";
        } DoCalcWitnessValue;
        // Symbol: drake::systems::LeafSystem::DoMakeLeafContext
        struct /* DoMakeLeafContext */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc =
R"""(Provides a new instance of the leaf context for this system. Derived
leaf systems with custom derived leaf system contexts should override
this to provide a context of the appropriate type. The returned
context should be "empty"; invoked by AllocateContext(), the caller
will take the responsibility to initialize the core LeafContext data.
The default implementation provides a default-constructed
``LeafContext<T>``.)""";
        } DoMakeLeafContext;
        // Symbol: drake::systems::LeafSystem::DoValidateAllocatedLeafContext
        struct /* DoValidateAllocatedLeafContext */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc =
R"""(Derived classes that impose restrictions on what resources are
permitted should check those restrictions by implementing this. For
example, a derived class might require a single input and single
output. Note that the supplied Context will be complete except that
input and output dependencies on peer and parent subcontexts will not
yet have been set up, so you may not consider them for validation. The
default implementation does nothing.)""";
        } DoValidateAllocatedLeafContext;
        // Symbol: drake::systems::LeafSystem::GetDirectFeedthroughs
        struct /* GetDirectFeedthroughs */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc = R"""()""";
        } GetDirectFeedthroughs;
        // Symbol: drake::systems::LeafSystem::GetMutableNumericParameter
        struct /* GetMutableNumericParameter */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc =
R"""(Extracts the numeric parameters of type U from the ``context`` at
``index``. Asserts if the context is not a LeafContext, or if it does
not have a vector-valued parameter of type U at ``index``.)""";
        } GetMutableNumericParameter;
        // Symbol: drake::systems::LeafSystem::GetNumericParameter
        struct /* GetNumericParameter */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc =
R"""(Extracts the numeric parameters of type U from the ``context`` at
``index``. Asserts if the context is not a LeafContext, or if it does
not have a vector-valued parameter of type U at ``index``.)""";
        } GetNumericParameter;
        // Symbol: drake::systems::LeafSystem::LeafSystem<T>
        struct /* ctor */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc_0args =
R"""(Default constructor that declares no inputs, outputs, state,
parameters, events, nor scalar-type conversion support (AutoDiff,
etc.). To enable AutoDiff support, use the SystemScalarConverter-based
constructor.)""";
          // Source: drake/systems/framework/leaf_system.h
          const char* doc_1args =
R"""(Constructor that declares no inputs, outputs, state, parameters, or
events, but allows subclasses to declare scalar-type conversion
support (AutoDiff, etc.).

The scalar-type conversion support will use ``converter``. To enable
scalar-type conversion support, pass a ``SystemTypeTag<S>{}`` where
``S`` must be the exact class of ``this`` being constructed.

See system_scalar_conversion for detailed background and examples
related to scalar-type conversion support.)""";
        } ctor;
        // Symbol: drake::systems::LeafSystem::MakeWitnessFunction
        struct /* MakeWitnessFunction */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc_3args =
R"""(Constructs the witness function with the given description (used
primarily for debugging and logging), direction type, and calculator
function; and with no event object.

Note:
    In order for the witness function to be used, you MUST overload
    System∷DoGetWitnessFunctions().)""";
          // Source: drake/systems/framework/leaf_system.h
          const char* doc_4args =
R"""(Constructs the witness function with the given description (used
primarily for debugging and logging), direction type, and calculator
function, and with an object corresponding to the event that is to be
dispatched when this witness function triggers. Example types of event
objects are publish, discrete variable update, unrestricted update
events. A clone of the event will be owned by the newly constructed
WitnessFunction.

Note:
    In order for the witness function to be used, you MUST overload
    System∷DoGetWitnessFunctions().)""";
        } MakeWitnessFunction;
        // Symbol: drake::systems::LeafSystem::SetDefaultParameters
        struct /* SetDefaultParameters */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc =
R"""(Default implementation: sets all numeric parameters to the model
vector given to DeclareNumericParameter, or else if no model was
provided sets the numeric parameter to one. It sets all abstract
parameters to the model value given to DeclareAbstractParameter.
Overrides must not change the number of parameters.)""";
        } SetDefaultParameters;
        // Symbol: drake::systems::LeafSystem::SetDefaultState
        struct /* SetDefaultState */ {
          // Source: drake/systems/framework/leaf_system.h
          const char* doc =
R"""(Default implementation: sets all continuous state to the model vector
given in DeclareContinuousState (or zero if no model vector was given)
and discrete states to zero. Overrides must not change the number of
state variables.)""";
        } SetDefaultState;
      } LeafSystem;
      // Symbol: drake::systems::NumericParameterIndex
      struct /* NumericParameterIndex */ {
        // Source: drake/systems/framework/framework_common.h
        const char* doc =
R"""(Serves as the local index for numeric parameter groups within a given
System and its corresponding Context.)""";
      } NumericParameterIndex;
      // Symbol: drake::systems::OutputPort
      struct /* OutputPort */ {
        // Source: drake/systems/framework/output_port.h
        const char* doc =
R"""(An OutputPort belongs to a System and represents the properties of one
of that System's output ports. OutputPort objects are assigned
OutputPortIndex values in the order they are declared; these are
unique within a single System.

An output port can be considered a "window" into a System that permits
controlled exposure of one of the values contained in that System's
Context at run time. Input ports of other subsystems may be connected
to an output port to construct system diagrams with carefully managed
interdependencies.

The exposed value may be the result of an output computation, or it
may simply expose some other value contained in the Context, such as
the values of state variables. The Context handles caching of output
port values and tracks dependencies to ensure that the values are
valid with respect to their prerequisites. Leaf systems provide for
the production of output port values, by computation or forwarding
from other values within the associated leaf context. A diagram's
output ports, on the other hand, are exported from output ports of its
contained subsystems.

An output port's value is always stored as an AbstractValue, but we
also provide special handling for output ports known to have numeric
(vector) values. Vector-valued ports may specify a particular vector
length, or may leave that to be determined at runtime.

OutputPort objects support three important operations:

- Allocate() returns an object that can hold the port's value.
- Calc() unconditionally computes the port's value.
- Eval() updates a cached value if necessary.)""";
        // Symbol: drake::systems::OutputPort::Allocate
        struct /* Allocate */ {
          // Source: drake/systems/framework/output_port.h
          const char* doc =
R"""(Allocates a concrete object suitable for holding the value to be
exposed by this output port, and returns that as an AbstractValue. The
returned object will never be null. If Drake assertions are enabled
(typically only in Debug builds), validates for a vector-valued port
that the returned AbstractValue is actually a BasicVector-derived type
and that it has an acceptable size.

Note:
    If this is a vector-valued port, the underlying type is
    ``Value<BasicVector<T>>``; downcast to ``BasicVector<T>`` before
    downcasting to the specific ``BasicVector`` subclass.)""";
        } Allocate;
        // Symbol: drake::systems::OutputPort::Calc
        struct /* Calc */ {
          // Source: drake/systems/framework/output_port.h
          const char* doc =
R"""(Unconditionally computes the value of this output port with respect to
the given context, into an already-allocated AbstractValue object
whose concrete type must be exactly the same as the type returned by
this port's allocator. If Drake assertions are enabled (typically only
in Debug builds), validates that the given ``value`` has exactly the
same concrete type as is returned by the Allocate() method.)""";
        } Calc;
        // Symbol: drake::systems::OutputPort::DoAllocate
        struct /* DoAllocate */ {
          // Source: drake/systems/framework/output_port.h
          const char* doc =
R"""(A concrete OutputPort must provide a way to allocate a suitable object
for holding the runtime value of this output port. The particulars may
depend on values and types of objects in the given Context.

Returns:
    A unique_ptr to the new value-holding object as an AbstractValue.)""";
        } DoAllocate;
        // Symbol: drake::systems::OutputPort::DoCalc
        struct /* DoCalc */ {
          // Source: drake/systems/framework/output_port.h
          const char* doc =
R"""(A concrete OutputPort must implement this method to calculate the
value this output port should have, given the supplied Context. The
value may be determined by computation or by copying from a source
value in the Context.

Parameter ``context``:
    A Context that has already been validated as compatible with the
    System whose output port this is.

Parameter ``value``:
    A pointer that has already be validated as non-null and pointing
    to an object of the right type to hold a value of this output
    port.)""";
        } DoCalc;
        // Symbol: drake::systems::OutputPort::DoEval
        struct /* DoEval */ {
          // Source: drake/systems/framework/output_port.h
          const char* doc =
R"""(A concrete OutputPort must provide access to the current value of this
output port stored within the given Context. If the value is already
up to date with respect to its prerequisites in ``context``, no
computation should be performed. Otherwise, the implementation should
arrange for the value to be computed, typically but not necessarily by
invoking DoCalc().

Parameter ``context``:
    A Context that has already been validated as compatible with the
    System whose output port this is.)""";
        } DoEval;
        // Symbol: drake::systems::OutputPort::Eval
        struct /* Eval */ {
          // Source: drake/systems/framework/output_port.h
          const char* doc = R"""()""";
        } Eval;
        // Symbol: drake::systems::OutputPort::OutputPort<T>
        struct /* ctor */ {
          // Source: drake/systems/framework/output_port.h
          const char* doc =
R"""(Provides derived classes the ability to set the base class members at
construction. See OutputPortBase∷OutputPortBase() for the meaning of
these parameters.

Precondition:
    The ``name`` must not be empty.

Precondition:
    The ``system`` parameter must be the same object as the
    ``system_interface`` parameter.)""";
        } ctor;
        // Symbol: drake::systems::OutputPort::ThrowIfInvalidPortValueType
        struct /* ThrowIfInvalidPortValueType */ {
          // Source: drake/systems/framework/output_port.h
          const char* doc_2args =
R"""(Check that an AbstractValue provided to Calc() is suitable for this
port. Derived classes should throw a helpful message if not (see
LeafOutputPort for an example). It is OK for this to be an expensive
check because we will only call it in Debug builds.)""";
          // Source: drake/systems/framework/output_port.h
          const char* doc_3args =
R"""(Static method allows DiagramOutputPort to call this recursively.)""";
        } ThrowIfInvalidPortValueType;
        // Symbol: drake::systems::OutputPort::get_system
        struct /* get_system */ {
          // Source: drake/systems/framework/output_port.h
          const char* doc =
R"""(Returns a reference to the System that owns this output port. Note
that for a diagram output port this will be the diagram, not the leaf
system whose output port was forwarded.)""";
        } get_system;
      } OutputPort;
      // Symbol: drake::systems::OutputPortBase
      struct /* OutputPortBase */ {
        // Source: drake/systems/framework/output_port_base.h
        const char* doc =
R"""(OutputPortBase handles the scalar type-independent aspects of an
OutputPort. An OutputPort belongs to a System and represents the
properties of one of that System's output ports.)""";
        // Symbol: drake::systems::OutputPortBase::DoGetPrerequisite
        struct /* DoGetPrerequisite */ {
          // Source: drake/systems/framework/output_port_base.h
          const char* doc =
R"""(Concrete output ports must implement this to return the prerequisite
dependency ticket for this port, which may be in the current System or
one of its immediate child subsystems.)""";
        } DoGetPrerequisite;
        // Symbol: drake::systems::OutputPortBase::GetPrerequisite
        struct /* GetPrerequisite */ {
          // Source: drake/systems/framework/output_port_base.h
          const char* doc = R"""()""";
        } GetPrerequisite;
        // Symbol: drake::systems::OutputPortBase::OutputPortBase
        struct /* ctor */ {
          // Source: drake/systems/framework/output_port_base.h
          const char* doc =
R"""(Provides derived classes the ability to set the base class members at
construction.

Parameter ``owning_system``:
    The System that owns this output port.

Parameter ``owning_system_id``:
    The ID of owning_system.

Parameter ``name``:
    A name for the port. Must not be empty. Output port names should
    be unique within a single System.

Parameter ``index``:
    The index to be assigned to this OutputPort.

Parameter ``ticket``:
    The DependencyTicket to be assigned to this OutputPort.

Parameter ``data_type``:
    Whether the port described is vector or abstract valued.

Parameter ``size``:
    If the port described is vector-valued, the number of elements
    expected, otherwise ignored.)""";
        } ctor;
        // Symbol: drake::systems::OutputPortBase::get_index
        struct /* get_index */ {
          // Source: drake/systems/framework/output_port_base.h
          const char* doc =
R"""(Returns the index of this output port within the owning System. For a
Diagram, this will be the index within the Diagram, *not* the index
within the LeafSystem whose output port was forwarded.)""";
        } get_index;
      } OutputPortBase;
      // Symbol: drake::systems::OutputPortIndex
      struct /* OutputPortIndex */ {
        // Source: drake/systems/framework/framework_common.h
        const char* doc =
R"""(Serves as the local index for the output ports of a given System. The
indexes used by a subsystem and its corresponding subcontext are the
same.)""";
      } OutputPortIndex;
      // Symbol: drake::systems::OutputPortSelection
      struct /* OutputPortSelection */ {
        // Source: drake/systems/framework/framework_common.h
        const char* doc =
R"""(Intended for use in e.g. variant<OutputPortSelection, OutputPortIndex>
for algorithms that support optional and/or default port indices.)""";
        // Symbol: drake::systems::OutputPortSelection::kNoOutput
        struct /* kNoOutput */ {
          // Source: drake/systems/framework/framework_common.h
          const char* doc = R"""()""";
        } kNoOutput;
        // Symbol: drake::systems::OutputPortSelection::kUseFirstOutputIfItExists
        struct /* kUseFirstOutputIfItExists */ {
          // Source: drake/systems/framework/framework_common.h
          const char* doc = R"""()""";
        } kUseFirstOutputIfItExists;
      } OutputPortSelection;
      // Symbol: drake::systems::Parameters
      struct /* Parameters */ {
        // Source: drake/systems/framework/parameters.h
        const char* doc =
R"""(Parameters is a container for variables that parameterize a System so
that it can represent a family of related models. Parameters are
members of the Context. Parameters are not Inputs because they do not
flow from upstream Systems, and they are not State because the System
does not define update functions for them. If Parameters are modified,
they are modified by application-specific logic, extrinsic to the
System framework and to the flow of simulation time.

The Parameters include both vector-valued and abstract-valued
elements.)""";
        // Symbol: drake::systems::Parameters::Clone
        struct /* Clone */ {
          // Source: drake/systems/framework/parameters.h
          const char* doc = R"""(Returns a deep copy of the Parameters.)""";
        } Clone;
        // Symbol: drake::systems::Parameters::Parameters<T>
        struct /* ctor */ {
          // Source: drake/systems/framework/parameters.h
          const char* doc_0args = R"""(Constructs an empty Parameters.)""";
          // Source: drake/systems/framework/parameters.h
          const char* doc_2args_numeric_abstract =
R"""(Constructs Parameters both ``numeric`` and ``abstract``.)""";
          // Source: drake/systems/framework/parameters.h
          const char* doc_1args_numeric =
R"""(Constructs Parameters that are purely ``numeric``.)""";
          // Source: drake/systems/framework/parameters.h
          const char* doc_1args_abstract =
R"""(Constructs Parameters that are purely ``abstract``.)""";
          // Source: drake/systems/framework/parameters.h
          const char* doc_1args_vec =
R"""(Constructs Parameters in the common case where the parameters consist
of exactly one numeric vector.)""";
          // Source: drake/systems/framework/parameters.h
          const char* doc_1args_value =
R"""(Constructs Parameters in the common case where the parameters consist
of exactly one abstract value.)""";
        } ctor;
        // Symbol: drake::systems::Parameters::SetFrom
        struct /* SetFrom */ {
          // Source: drake/systems/framework/parameters.h
          const char* doc = R"""(Initializes this object from ``other``.)""";
        } SetFrom;
        // Symbol: drake::systems::Parameters::get_abstract_parameter
        struct /* get_abstract_parameter */ {
          // Source: drake/systems/framework/parameters.h
          const char* doc_1args_index =
R"""(Returns the abstract-valued parameter at ``index``. Asserts if the
index is out of bounds.)""";
          // Source: drake/systems/framework/parameters.h
          const char* doc_1args_int =
R"""(Returns the abstract-valued parameter at ``index``. Asserts if the
index is out of bounds, and throws if the parameter is not of type V.)""";
        } get_abstract_parameter;
        // Symbol: drake::systems::Parameters::get_abstract_parameters
        struct /* get_abstract_parameters */ {
          // Source: drake/systems/framework/parameters.h
          const char* doc = R"""()""";
        } get_abstract_parameters;
        // Symbol: drake::systems::Parameters::get_mutable_abstract_parameter
        struct /* get_mutable_abstract_parameter */ {
          // Source: drake/systems/framework/parameters.h
          const char* doc_1args_index =
R"""(Returns the abstract-valued parameter at ``index``. Asserts if the
index is out of bounds.)""";
          // Source: drake/systems/framework/parameters.h
          const char* doc_1args_int =
R"""(Returns the abstract-valued parameter at ``index``. Asserts if the
index is out of bounds, and throws if the parameter is not of type V.)""";
        } get_mutable_abstract_parameter;
        // Symbol: drake::systems::Parameters::get_mutable_numeric_parameter
        struct /* get_mutable_numeric_parameter */ {
          // Source: drake/systems/framework/parameters.h
          const char* doc =
R"""(Returns the vector-valued parameter at ``index``. Asserts if the index
is out of bounds.)""";
        } get_mutable_numeric_parameter;
        // Symbol: drake::systems::Parameters::get_numeric_parameter
        struct /* get_numeric_parameter */ {
          // Source: drake/systems/framework/parameters.h
          const char* doc =
R"""(Returns the vector-valued parameter at ``index``. Asserts if the index
is out of bounds.)""";
        } get_numeric_parameter;
        // Symbol: drake::systems::Parameters::get_numeric_parameters
        struct /* get_numeric_parameters */ {
          // Source: drake/systems/framework/parameters.h
          const char* doc = R"""()""";
        } get_numeric_parameters;
        // Symbol: drake::systems::Parameters::get_system_id
        struct /* get_system_id */ {
          // Source: drake/systems/framework/parameters.h
          const char* doc =
R"""((Internal use only) Gets the id of the subsystem that created this
object.)""";
        } get_system_id;
        // Symbol: drake::systems::Parameters::num_abstract_parameters
        struct /* num_abstract_parameters */ {
          // Source: drake/systems/framework/parameters.h
          const char* doc = R"""()""";
        } num_abstract_parameters;
        // Symbol: drake::systems::Parameters::num_numeric_parameter_groups
        struct /* num_numeric_parameter_groups */ {
          // Source: drake/systems/framework/parameters.h
          const char* doc = R"""()""";
        } num_numeric_parameter_groups;
        // Symbol: drake::systems::Parameters::set_abstract_parameters
        struct /* set_abstract_parameters */ {
          // Source: drake/systems/framework/parameters.h
          const char* doc = R"""()""";
        } set_abstract_parameters;
        // Symbol: drake::systems::Parameters::set_numeric_parameters
        struct /* set_numeric_parameters */ {
          // Source: drake/systems/framework/parameters.h
          const char* doc = R"""()""";
        } set_numeric_parameters;
        // Symbol: drake::systems::Parameters::set_system_id
        struct /* set_system_id */ {
          // Source: drake/systems/framework/parameters.h
          const char* doc =
R"""((Internal use only) Records the id of the subsystem that created this
object.)""";
        } set_system_id;
      } Parameters;
      // Symbol: drake::systems::PeriodicEventData
      struct /* PeriodicEventData */ {
        // Source: drake/systems/framework/event.h
        const char* doc =
R"""(An event data variant describing an event that recurs on a fixed
period. The events are triggered at time = offset_sec + i *
period_sec, where i is a non-negative integer.)""";
        // Symbol: drake::systems::PeriodicEventData::PeriodicEventData
        struct /* ctor */ {
          // Source: drake/systems/framework/event.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::systems::PeriodicEventData::offset_sec
        struct /* offset_sec */ {
          // Source: drake/systems/framework/event.h
          const char* doc =
R"""(Gets the time after zero when this event should first occur.)""";
        } offset_sec;
        // Symbol: drake::systems::PeriodicEventData::period_sec
        struct /* period_sec */ {
          // Source: drake/systems/framework/event.h
          const char* doc =
R"""(Gets the period with which this event should recur.)""";
        } period_sec;
        // Symbol: drake::systems::PeriodicEventData::set_offset_sec
        struct /* set_offset_sec */ {
          // Source: drake/systems/framework/event.h
          const char* doc =
R"""(Sets the time after zero when this event should first occur.)""";
        } set_offset_sec;
        // Symbol: drake::systems::PeriodicEventData::set_period_sec
        struct /* set_period_sec */ {
          // Source: drake/systems/framework/event.h
          const char* doc =
R"""(Sets the period with which this event should recur.)""";
        } set_period_sec;
      } PeriodicEventData;
      // Symbol: drake::systems::PeriodicEventDataComparator
      struct /* PeriodicEventDataComparator */ {
        // Source: drake/systems/framework/event.h
        const char* doc =
R"""(Structure for comparing two PeriodicEventData objects for use in a map
container, using an arbitrary comparison method.)""";
        // Symbol: drake::systems::PeriodicEventDataComparator::operator()
        struct /* operator_call */ {
          // Source: drake/systems/framework/event.h
          const char* doc = R"""()""";
        } operator_call;
      } PeriodicEventDataComparator;
      // Symbol: drake::systems::PortBase
      struct /* PortBase */ {
        // Source: drake/systems/framework/port_base.h
        const char* doc =
R"""(A PortBase is base class for System ports; users will typically use
the InputPort<T> or OutputPort<T> types, not this base class.)""";
        // Symbol: drake::systems::PortBase::GetFullDescription
        struct /* GetFullDescription */ {
          // Source: drake/systems/framework/port_base.h
          const char* doc =
R"""(Returns a verbose human-readable description of port. This is useful
for error messages or debugging.)""";
        } GetFullDescription;
        // Symbol: drake::systems::PortBase::PortBase
        struct /* ctor */ {
          // Source: drake/systems/framework/port_base.h
          const char* doc =
R"""(Provides derived classes the ability to set the base class members at
construction.

Parameter ``kind_string``:
    Either "Input" or "Output", depending on the kind of subclass.

Parameter ``owning_system``:
    The System that owns this port.

Parameter ``owning_system_id``:
    The ID of owning_system.

Parameter ``name``:
    A name for the port. Port names should be non-empty and unique
    within a single System.

Parameter ``index``:
    The index to be assigned to this port. Input ports and output
    ports each have their own pool of indices (InputPortIndex and
    OutputPortIndex); this is just that TypeSafeIndex passed as a bare
    int.

Parameter ``ticket``:
    The DependencyTicket to be assigned to this port.

Parameter ``data_type``:
    Whether the port described is vector- or abstract-valued.

Parameter ``size``:
    If the port described is vector-valued, the number of elements.
    Ignored for abstract-valued ports.)""";
        } ctor;
        // Symbol: drake::systems::PortBase::PortEvalCast
        struct /* PortEvalCast */ {
          // Source: drake/systems/framework/port_base.h
          const char* doc_1args_constAbstractValue =
R"""(Pull a value of a given type from an abstract value or issue a nice
message if the type is not correct.)""";
          // Source: drake/systems/framework/port_base.h
          const char* doc_1args_constBasicVector =
R"""(Downcast a basic vector to a more specific subclass, or else issue a
nice message if the type is not correct.)""";
        } PortEvalCast;
        // Symbol: drake::systems::PortBase::ThrowBadCast
        struct /* ThrowBadCast */ {
          // Source: drake/systems/framework/port_base.h
          const char* doc_1args =
R"""(Reports that the user provided a bad ValueType argument to Eval.)""";
          // Source: drake/systems/framework/port_base.h
          const char* doc_2args =
R"""(Reports that the user provided a bad ValueType argument to Eval. The
value_typename is the type of the port's current value; the
eval_typename is the type the user asked for.)""";
        } ThrowBadCast;
        // Symbol: drake::systems::PortBase::ThrowValidateContextMismatch
        struct /* ThrowValidateContextMismatch */ {
          // Source: drake/systems/framework/port_base.h
          const char* doc =
R"""((Internal use only) Throws RuntimeError with a message that the sanity
check(s) related to ValidateContext have failed.)""";
        } ThrowValidateContextMismatch;
        // Symbol: drake::systems::PortBase::ValidateSystemId
        struct /* ValidateSystemId */ {
          // Source: drake/systems/framework/port_base.h
          const char* doc =
R"""((Internal use only) Checks whether the given id (nominally obtained
from a Context passed to this port) was created for the system that
owns this port.

This is similar in spirit to SystemBase∷ValidateContext, but ports
cannot use SystemBase.

Note:
    This method is sufficiently fast for performance sensitive code.)""";
        } ValidateSystemId;
        // Symbol: drake::systems::PortBase::get_data_type
        struct /* get_data_type */ {
          // Source: drake/systems/framework/port_base.h
          const char* doc = R"""(Returns the port data type.)""";
        } get_data_type;
        // Symbol: drake::systems::PortBase::get_deprecation
        struct /* get_deprecation */ {
          // Source: drake/systems/framework/port_base.h
          const char* doc =
R"""(When this port is deprecated, returns non-null with a (possibly empty)
deprecation message; when this port is not deprecated, returns null.)""";
        } get_deprecation;
        // Symbol: drake::systems::PortBase::get_int_index
        struct /* get_int_index */ {
          // Source: drake/systems/framework/port_base.h
          const char* doc =
R"""(Returns the index of this port within the owning System (i.e., an
InputPortIndex or OutputPortIndex, but as a bare integer). For a
Diagram, this will be the index within the Diagram, *not* the index
within the LeafSystem whose output port was forwarded.)""";
        } get_int_index;
        // Symbol: drake::systems::PortBase::get_mutable_system_interface
        struct /* get_mutable_system_interface */ {
          // Source: drake/systems/framework/port_base.h
          const char* doc =
R"""(Returns get_system_interface(), but without the const.)""";
        } get_mutable_system_interface;
        // Symbol: drake::systems::PortBase::get_name
        struct /* get_name */ {
          // Source: drake/systems/framework/port_base.h
          const char* doc = R"""(Get port name.)""";
        } get_name;
        // Symbol: drake::systems::PortBase::get_system_interface
        struct /* get_system_interface */ {
          // Source: drake/systems/framework/port_base.h
          const char* doc =
R"""(Returns a reference to the system that owns this port. Note that for a
diagram port this will be the diagram, not the leaf system whose port
was exported.)""";
        } get_system_interface;
        // Symbol: drake::systems::PortBase::set_deprecation
        struct /* set_deprecation */ {
          // Source: drake/systems/framework/port_base.h
          const char* doc =
R"""(Sets whether this port is deprecated (and if so, the message).)""";
        } set_deprecation;
        // Symbol: drake::systems::PortBase::size
        struct /* size */ {
          // Source: drake/systems/framework/port_base.h
          const char* doc =
R"""(Returns the fixed size expected for a vector-valued port. Not
meaningful for abstract-valued ports.)""";
        } size;
        // Symbol: drake::systems::PortBase::ticket
        struct /* ticket */ {
          // Source: drake/systems/framework/port_base.h
          const char* doc =
R"""((Advanced.) Returns the DependencyTicket for this port within the
owning System.)""";
        } ticket;
      } PortBase;
      // Symbol: drake::systems::PortDataType
      struct /* PortDataType */ {
        // Source: drake/systems/framework/framework_common.h
        const char* doc =
R"""(All system ports are either vectors of Eigen scalars, or black-box
AbstractValues which may contain any type.)""";
        // Symbol: drake::systems::PortDataType::kAbstractValued
        struct /* kAbstractValued */ {
          // Source: drake/systems/framework/framework_common.h
          const char* doc = R"""()""";
        } kAbstractValued;
        // Symbol: drake::systems::PortDataType::kVectorValued
        struct /* kVectorValued */ {
          // Source: drake/systems/framework/framework_common.h
          const char* doc = R"""()""";
        } kVectorValued;
      } PortDataType;
      // Symbol: drake::systems::PublishEvent
      struct /* PublishEvent */ {
        // Source: drake/systems/framework/event.h
        const char* doc =
R"""(This class represents a publish event. It has an optional callback
function to do custom handling of this event.

See also:
    System∷Publish for more details.

See also:
    LeafSystem for more convenient interfaces to publish events via
    the Declare*PublishEvent() methods.)""";
        // Symbol: drake::systems::PublishEvent::PublishEvent<T>
        struct /* ctor */ {
          // Source: drake/systems/framework/event.h
          const char* doc_0args = R"""(Constructs an empty PublishEvent.)""";
          // Source: drake/systems/framework/event.h
          const char* doc_1args =
R"""(Constructs a PublishEvent with the given callback function.)""";
        } ctor;
        // Symbol: drake::systems::PublishEvent::handle
        struct /* handle */ {
          // Source: drake/systems/framework/event.h
          const char* doc =
R"""(Calls the optional callback or system callback function, if one
exists, with ``system``, ``context``, and ``this``.)""";
        } handle;
        // Symbol: drake::systems::PublishEvent::is_discrete_update
        struct /* is_discrete_update */ {
          // Source: drake/systems/framework/event.h
          const char* doc = R"""()""";
        } is_discrete_update;
      } PublishEvent;
      // Symbol: drake::systems::SingleOutputVectorSource
      struct /* SingleOutputVectorSource */ {
        // Source: drake/systems/framework/single_output_vector_source.h
        const char* doc =
R"""(A base class that specializes LeafSystem for use with no input ports,
and only a single, vector output port. Subclasses should override the
protected method


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    void DoCalcOutput(const Context<T>&, Eigen∷VectorBlock<VectorX<T>>*) const;

.. raw:: html

    </details>

.. pydrake_system::

    name: SingleOutputVectorSource
    output_ports:
    - y0)""";
        // Symbol: drake::systems::SingleOutputVectorSource::DoCalcVectorOutput
        struct /* DoCalcVectorOutput */ {
          // Source: drake/systems/framework/single_output_vector_source.h
          const char* doc =
R"""(Provides a convenience method for SingleOutputVectorSource subclasses.
This method performs the same logical operation as System∷DoCalcOutput
but provides the single output's VectorBlock instead. Subclasses
should override this method, and not the base class method (which is
``final``).)""";
        } DoCalcVectorOutput;
        // Symbol: drake::systems::SingleOutputVectorSource::SingleOutputVectorSource<T>
        struct /* ctor */ {
          // Source: drake/systems/framework/single_output_vector_source.h
          const char* doc_0args =
R"""(Deleted default constructor. Child classes must either supply the
vector size to the single-argument constructor of ``int``, or supply a
model vector to the single-argument constructor of ``const
BasicVector<T>&``.)""";
          // Source: drake/systems/framework/single_output_vector_source.h
          const char* doc_1args_size =
R"""(Creates a source with the given sole output port configuration.

Note:
    Objects created using this constructor overload do not support
    system scalar conversion. See system_scalar_conversion. Use a
    different constructor overload if such conversion is desired.)""";
          // Source: drake/systems/framework/single_output_vector_source.h
          const char* doc_1args_model_vector =
R"""(Creates a source with output type and dimension of the
``model_vector``.

Note:
    Objects created using this constructor overload do not support
    system scalar conversion. See system_scalar_conversion. Use a
    different constructor overload if such conversion is desired.)""";
          // Source: drake/systems/framework/single_output_vector_source.h
          const char* doc_2args_converter_size =
R"""(Creates a source with the given sole output port configuration.

Note:
    objects created using this constructor may support system scalar
    conversion. See system_scalar_conversion.

Parameter ``converter``:
    is per LeafSystem∷LeafSystem constructor documentation; see that
    function documentation for details.)""";
          // Source: drake/systems/framework/single_output_vector_source.h
          const char* doc_2args_converter_model_vector =
R"""(Creates a source with output type and dimension of the
``model_vector``.

Note:
    objects created using this constructor may support system scalar
    conversion. See system_scalar_conversion.

Parameter ``converter``:
    is per LeafSystem∷LeafSystem constructor documentation; see that
    function documentation for details.)""";
        } ctor;
        // Symbol: drake::systems::SingleOutputVectorSource::get_output_port
        struct /* get_output_port */ {
          // Source: drake/systems/framework/single_output_vector_source.h
          const char* doc = R"""(Returns the sole output port.)""";
        } get_output_port;
      } SingleOutputVectorSource;
      // Symbol: drake::systems::State
      struct /* State */ {
        // Source: drake/systems/framework/state.h
        const char* doc =
R"""(%State is a container for all the data comprising the complete state
of a particular System at a particular moment. Any field in State may
be empty if it is not applicable to the System in question. A System
may not maintain state in any place other than a State object.

A State ``x`` contains three types of state variables:

- Continuous state ``xc``
- Discrete state   ``xd``
- Abstract state   ``xa``)""";
        // Symbol: drake::systems::State::SetFrom
        struct /* SetFrom */ {
          // Source: drake/systems/framework/state.h
          const char* doc = R"""(Initializes this state from a State<U>.)""";
        } SetFrom;
        // Symbol: drake::systems::State::State<T>
        struct /* ctor */ {
          // Source: drake/systems/framework/state.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::systems::State::get_abstract_state
        struct /* get_abstract_state */ {
          // Source: drake/systems/framework/state.h
          const char* doc =
R"""(Returns a const pointer to the abstract component of the state at
``index``. Asserts if ``index`` doesn't exist.)""";
        } get_abstract_state;
        // Symbol: drake::systems::State::get_continuous_state
        struct /* get_continuous_state */ {
          // Source: drake/systems/framework/state.h
          const char* doc = R"""()""";
        } get_continuous_state;
        // Symbol: drake::systems::State::get_discrete_state
        struct /* get_discrete_state */ {
          // Source: drake/systems/framework/state.h
          const char* doc = R"""()""";
        } get_discrete_state;
        // Symbol: drake::systems::State::get_mutable_abstract_state
        struct /* get_mutable_abstract_state */ {
          // Source: drake/systems/framework/state.h
          const char* doc_0args =
R"""(Returns a mutable reference to the abstract component of the state,
which may be of size zero.)""";
          // Source: drake/systems/framework/state.h
          const char* doc_1args =
R"""(Returns a mutable pointer to element ``index`` of the abstract state.
Asserts if ``index`` doesn't exist.)""";
        } get_mutable_abstract_state;
        // Symbol: drake::systems::State::get_mutable_continuous_state
        struct /* get_mutable_continuous_state */ {
          // Source: drake/systems/framework/state.h
          const char* doc = R"""()""";
        } get_mutable_continuous_state;
        // Symbol: drake::systems::State::get_mutable_discrete_state
        struct /* get_mutable_discrete_state */ {
          // Source: drake/systems/framework/state.h
          const char* doc = R"""()""";
        } get_mutable_discrete_state;
        // Symbol: drake::systems::State::get_system_id
        struct /* get_system_id */ {
          // Source: drake/systems/framework/state.h
          const char* doc =
R"""((Internal use only) Gets the id of the subsystem that created this
state.)""";
        } get_system_id;
        // Symbol: drake::systems::State::set_abstract_state
        struct /* set_abstract_state */ {
          // Source: drake/systems/framework/state.h
          const char* doc =
R"""((Advanced) Defines the abstract state variables for this State.

Warning:
    Do not use this function to resize or change types of abstract
    state variables in a State that is owned by an existing Context.)""";
        } set_abstract_state;
        // Symbol: drake::systems::State::set_continuous_state
        struct /* set_continuous_state */ {
          // Source: drake/systems/framework/state.h
          const char* doc =
R"""((Advanced) Defines the continuous state variables for this State.

Warning:
    Do not use this function to resize continuous state in a State
    that is owned by an existing Context.)""";
        } set_continuous_state;
        // Symbol: drake::systems::State::set_discrete_state
        struct /* set_discrete_state */ {
          // Source: drake/systems/framework/state.h
          const char* doc =
R"""((Advanced) Defines the discrete state variables for this State.

Warning:
    Do not use this function to resize discrete state in a State that
    is owned by an existing Context.)""";
        } set_discrete_state;
        // Symbol: drake::systems::State::set_system_id
        struct /* set_system_id */ {
          // Source: drake/systems/framework/state.h
          const char* doc =
R"""((Internal use only) Records the id of the subsystem that created this
state.)""";
        } set_system_id;
      } State;
      // Symbol: drake::systems::SubsystemIndex
      struct /* SubsystemIndex */ {
        // Source: drake/systems/framework/framework_common.h
        const char* doc =
R"""(Serves as a local index for a child subsystem within a parent Diagram,
or a child subcontext within a parent DiagramContext. A subsystem and
its matching subcontext have the same SubsystemIndex. Unique only
within a given subsystem or subcontext.)""";
      } SubsystemIndex;
      // Symbol: drake::systems::Subvector
      struct /* Subvector */ {
        // Source: drake/systems/framework/subvector.h
        const char* doc =
R"""(Subvector is a concrete class template that implements VectorBase by
providing a sliced view of a VectorBase.)""";
        // Symbol: drake::systems::Subvector::Subvector<T>
        struct /* ctor */ {
          // Source: drake/systems/framework/subvector.h
          const char* doc =
R"""(Constructs a subvector of vector that consists of num_elements
starting at first_element.

Parameter ``vector``:
    The vector to slice. Must not be nullptr. Must remain valid for
    the lifetime of this object.)""";
        } ctor;
        // Symbol: drake::systems::Subvector::size
        struct /* size */ {
          // Source: drake/systems/framework/subvector.h
          const char* doc = R"""()""";
        } size;
      } Subvector;
      // Symbol: drake::systems::Supervector
      struct /* Supervector */ {
        // Source: drake/systems/framework/supervector.h
        const char* doc =
R"""(Supervector is a concrete class template that implements VectorBase by
concatenating multiple VectorBases, which it does not own.)""";
        // Symbol: drake::systems::Supervector::Supervector<T>
        struct /* ctor */ {
          // Source: drake/systems/framework/supervector.h
          const char* doc =
R"""(Constructs a supervector consisting of all the vectors in subvectors,
which must live at least as long as this supervector.)""";
        } ctor;
        // Symbol: drake::systems::Supervector::size
        struct /* size */ {
          // Source: drake/systems/framework/supervector.h
          const char* doc = R"""()""";
        } size;
      } Supervector;
      // Symbol: drake::systems::System
      struct /* System */ {
        // Source: drake/systems/framework/system.h
        const char* doc =
R"""(Base class for all System functionality that is dependent on the
templatized scalar type T for input, state, parameters, and outputs.)""";
        // Symbol: drake::systems::System::Accept
        struct /* Accept */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Implements a visitor pattern.

See also:
    SystemVisitor<T>.)""";
        } Accept;
        // Symbol: drake::systems::System::AddConstraint
        struct /* AddConstraint */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Adds an already-created constraint to the list of constraints for this
System. Ownership of the SystemConstraint is transferred to this
system.)""";
        } AddConstraint;
        // Symbol: drake::systems::System::AddExternalConstraint
        struct /* AddExternalConstraint */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Adds an "external" constraint to this System.

This method is intended for use by applications that are examining
this System to add additional constraints based on their particular
situation (e.g., that a velocity state element has an upper bound); it
is not intended for declaring intrinsic constraints that some
particular System subclass might always impose on itself (e.g., that a
mass parameter is non-negative). To that end, this method should not
be called by subclasses of ``this`` during their constructor.

The ``constraint`` will automatically persist across system scalar
conversion.)""";
        } AddExternalConstraint;
        // Symbol: drake::systems::System::AddTriggeredWitnessFunctionToCompositeEventCollection
        struct /* AddTriggeredWitnessFunctionToCompositeEventCollection */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Add ``event`` to ``events`` due to a witness function triggering.
``events`` should be allocated with this system's
AllocateCompositeEventCollection. Neither ``event`` nor ``events`` can
be nullptr. Additionally, ``event`` must contain event data
(event->get_event_data() must not be nullptr) and the type of that
data must be WitnessTriggeredEventData.)""";
        } AddTriggeredWitnessFunctionToCompositeEventCollection;
        // Symbol: drake::systems::System::AllocateCompositeEventCollection
        struct /* AllocateCompositeEventCollection */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Allocates a CompositeEventCollection for this system. The allocated
instance is used for populating collections of triggered events; for
example, Simulator passes this object to System∷CalcNextUpdateTime()
to allow the system to identify and handle upcoming events.)""";
        } AllocateCompositeEventCollection;
        // Symbol: drake::systems::System::AllocateContext
        struct /* AllocateContext */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""((Advanced) Returns an **uninitialized** Context<T> suitable for use
with this System<T>. Most users should use CreateDefaultContext(),
instead.

Warning:
    The returned context is uninitialized (contains invalid data). It
    is useful for pre-allocating storage which will later be
    overwritten (e.g., by SetDefaultContext() or
    Context<T>∷SetTimeStateAndParametersFrom()) and **must not** be
    used for any calculations until it's been overwritten.)""";
        } AllocateContext;
        // Symbol: drake::systems::System::AllocateDiscreteVariables
        struct /* AllocateDiscreteVariables */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Returns a DiscreteValues of the same dimensions as the discrete_state
allocated in CreateDefaultContext. The simulator will provide this
state as the output argument to Update.)""";
        } AllocateDiscreteVariables;
        // Symbol: drake::systems::System::AllocateFixedInputs
        struct /* AllocateFixedInputs */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(For each input port, allocates a fixed input of the concrete type that
this System requires, and binds it to the port, disconnecting any
prior input. Does not assign any values to the fixed inputs.)""";
        } AllocateFixedInputs;
        // Symbol: drake::systems::System::AllocateForcedDiscreteUpdateEventCollection
        struct /* AllocateForcedDiscreteUpdateEventCollection */ {
          // Source: drake/systems/framework/system.h
          const char* doc = R"""()""";
        } AllocateForcedDiscreteUpdateEventCollection;
        // Symbol: drake::systems::System::AllocateForcedPublishEventCollection
        struct /* AllocateForcedPublishEventCollection */ {
          // Source: drake/systems/framework/system.h
          const char* doc = R"""()""";
        } AllocateForcedPublishEventCollection;
        // Symbol: drake::systems::System::AllocateForcedUnrestrictedUpdateEventCollection
        struct /* AllocateForcedUnrestrictedUpdateEventCollection */ {
          // Source: drake/systems/framework/system.h
          const char* doc = R"""()""";
        } AllocateForcedUnrestrictedUpdateEventCollection;
        // Symbol: drake::systems::System::AllocateImplicitTimeDerivativesResidual
        struct /* AllocateImplicitTimeDerivativesResidual */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Returns an Eigen VectorX suitable for use as the output argument to
the CalcImplicitTimeDerivativesResidual() method. The returned VectorX
will have size implicit_time_derivatives_residual_size() with the
elements uninitialized. This is just a convenience method -- you are
free to use any properly-sized mutable Eigen object as the residual
vector.)""";
        } AllocateImplicitTimeDerivativesResidual;
        // Symbol: drake::systems::System::AllocateInputAbstract
        struct /* AllocateInputAbstract */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Given an input port, allocates the abstract storage. The
``input_port`` must match a port declared via DeclareInputPort.)""";
        } AllocateInputAbstract;
        // Symbol: drake::systems::System::AllocateInputVector
        struct /* AllocateInputVector */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Given an input port, allocates the vector storage. The ``input_port``
must match a port declared via DeclareInputPort.)""";
        } AllocateInputVector;
        // Symbol: drake::systems::System::AllocateOutput
        struct /* AllocateOutput */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Returns a container that can hold the values of all of this System's
output ports. It is sized with the number of output ports and uses
each output port's allocation method to provide an object of the right
type for that port.)""";
        } AllocateOutput;
        // Symbol: drake::systems::System::AllocateTimeDerivatives
        struct /* AllocateTimeDerivatives */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Returns a ContinuousState of the same size as the continuous_state
allocated in CreateDefaultContext. The simulator will provide this
state as the output argument to EvalTimeDerivatives.)""";
        } AllocateTimeDerivatives;
        // Symbol: drake::systems::System::ApplyDiscreteVariableUpdate
        struct /* ApplyDiscreteVariableUpdate */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Given the ``discrete_state`` results of a previous call to
CalcDiscreteVariableUpdate() that dispatched the given collection of
events, modifies the ``context`` to reflect the updated
``discrete_state``.

Parameter ``events``:
    The Event collection that resulted in the given
    ``discrete_state``.

Parameter ``discrete_state``:
    The updated discrete state from a CalcDiscreteVariableUpdate()
    call. This is mutable to permit its contents to be swapped with
    the corresponding ``context`` contents (rather than copied).

Parameter ``context``:
    The Context whose discrete state is modified to match
    ``discrete_state``. Note that swapping contents with
    ``discrete_state`` may cause addresses of individual discrete
    state group vectors in ``context`` to be different on return than
    they were on entry.

Precondition:
    ``discrete_state`` is the result of a previous
    CalcDiscreteVariableUpdate() call that dispatched this ``events``
    collection.)""";
        } ApplyDiscreteVariableUpdate;
        // Symbol: drake::systems::System::ApplyUnrestrictedUpdate
        struct /* ApplyUnrestrictedUpdate */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Given the ``state`` results of a previous call to
CalcUnrestrictedUpdate() that dispatched the given collection of
events, modifies the ``context`` to reflect the updated ``state``.

Parameter ``events``:
    The Event collection that resulted in the given ``state``.

Parameter ``state``:
    The updated State from a CalcUnrestrictedUpdate() call. This is
    mutable to permit its contents to be swapped with the
    corresponding ``context`` contents (rather than copied).

Parameter ``context``:
    The Context whose State is modified to match ``state``. Note that
    swapping contents with the ``state`` may cause addresses of
    continuous, discrete, and abstract state containers in ``context``
    to be different on return than they were on entry.

Precondition:
    ``state`` is the result of a previous CalcUnrestrictedUpdate()
    call that dispatched this ``events`` collection.)""";
        } ApplyUnrestrictedUpdate;
        // Symbol: drake::systems::System::CalcConservativePower
        struct /* CalcConservativePower */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Calculates and returns the conservative power represented by the
current contents of the given ``context``. Prefer
EvalConservativePower() to avoid unnecessary recalculation.

See also:
    EvalConservativePower() for more information.)""";
        } CalcConservativePower;
        // Symbol: drake::systems::System::CalcDiscreteVariableUpdate
        struct /* CalcDiscreteVariableUpdate */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(This method is the public entry point for dispatching all discrete
variable update event handlers. Using all the discrete update handlers
in ``events``, the method calculates the update ``xd(n+1)`` to
discrete variables ``xd(n)`` in ``context`` and outputs the results to
``discrete_state``. See documentation for
DispatchDiscreteVariableUpdateHandler() for more details.)""";
        } CalcDiscreteVariableUpdate;
        // Symbol: drake::systems::System::CalcForcedDiscreteVariableUpdate
        struct /* CalcForcedDiscreteVariableUpdate */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""((Advanced) Manually triggers any DiscreteUpdateEvent that has trigger
type kForced. Invokes the discrete event dispatcher on this System
with the given Context providing the initial values for the discrete
variables. The updated values of the discrete variables are written to
the ``discrete_state`` output argument; no change is made to the
Context.

The default dispatcher will invoke the handlers (if any) associated
with each force-triggered event.

Note:
    There will always be at least one force-triggered event, though
    with no associated handler. By default that will do nothing when
    triggered, but that behavior can be changed by overriding the
    dispatcher (not recommended).

Raises:
    RuntimeError if it invokes an event handler that returns status
    indicating failure.

See also:
    CalcDiscreteVariableUpdate(), CalcForcedUnrestrictedUpdate())""";
        } CalcForcedDiscreteVariableUpdate;
        // Symbol: drake::systems::System::CalcForcedUnrestrictedUpdate
        struct /* CalcForcedUnrestrictedUpdate */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""((Advanced) Manually triggers any UnrestrictedUpdateEvent that has
trigger type kForced. Invokes the unrestricted event dispatcher on
this System with the given Context providing the initial values for
the state variables. The updated values of the state variables are
written to the ``state`` output argument; no change is made to the
Context.

The default dispatcher will invoke the handlers (if any) associated
with each force-triggered event.

Note:
    There will always be at least one force-triggered event, though
    with no associated handler. By default that will do nothing when
    triggered, but that behavior can be changed by overriding the
    dispatcher (not recommended).

Raises:
    RuntimeError if it invokes an event handler that returns status
    indicating failure.

See also:
    CalcUnrestrictedUpdate())""";
        } CalcForcedUnrestrictedUpdate;
        // Symbol: drake::systems::System::CalcImplicitTimeDerivativesResidual
        struct /* CalcImplicitTimeDerivativesResidual */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Evaluates the implicit form of the System equations and returns the
residual.

The explicit and implicit forms of the System equations are

(1) ẋ꜀ = fₑ(𝓒) explicit (2) 0 = fᵢ(𝓒; ẋ꜀) implicit

where ``𝓒 = {a, p, t, x, u}`` is the current value of the given
Context from which accuracy a, parameters p, time t, state x (``={x꜀
xd xₐ}``) and input values u are obtained. Substituting (1) into (2)
shows that the following condition must always hold:

(3) fᵢ(𝓒; fₑ(𝓒)) = 0 always true

When ``fᵢ(𝓒; ẋ꜀ₚ)`` is evaluated with a proposed time derivative ẋ꜀ₚ
that differs from ẋ꜀ the result will be non-zero; we call that the
*residual* of the implicit equation. Given a Context and proposed time
derivative ẋ꜀ₚ, this method returns the residual r such that

(4) r = fᵢ(𝓒; ẋ꜀ₚ).

The returned r will typically be the same length as x꜀ although that
is not required. And even if r and x꜀ are the same size, there will
not necessarily be any elementwise correspondence between them. (That
is, you should not assume that r[i] is the "residual" of ẋ꜀ₚ[i].) For
a Diagram, r is the concatenation of residuals from each of the
subsystems, in order of subsystem index within the Diagram.

A default implementation fᵢ⁽ᵈᵉᶠ⁾ for the implicit form is always
provided and makes use of the explicit form as follows:

(5) fᵢ⁽ᵈᵉᶠ⁾(𝓒; ẋ꜀ₚ) ≜ ẋ꜀ₚ − fₑ(𝓒)

which satisfies condition (3) by construction. (Note that the default
implementation requires the residual to have the same size as x꜀.)
Substantial efficiency gains can often be obtained by replacing the
default function with a customized implementation. Override
DoCalcImplicitTimeDerivativesResidual() to replace the default
implementation with a better one.

Parameter ``context``:
    The source for time, state, inputs, etc. to be used in calculating
    the residual.

Parameter ``proposed_derivatives``:
    The proposed value ẋ꜀ₚ for the time derivatives of x꜀.

Parameter ``residual``:
    The result r of evaluating the implicit function. Can be any
    mutable Eigen vector object of size
    implicit_time_derivatives_residual_size().

Precondition:
    ``proposed_derivatives`` is compatible with this System.

Precondition:
    ``residual`` is of size implicit_time_derivatives_residual_size().

See also:
    SystemBase∷implicit_time_derivatives_residual_size()

See also:
    LeafSystem∷DeclareImplicitTimeDerivativesResidualSize()

See also:
    DoCalcImplicitTimeDerivativesResidual()

See also:
    CalcTimeDerivatives())""";
        } CalcImplicitTimeDerivativesResidual;
        // Symbol: drake::systems::System::CalcKineticEnergy
        struct /* CalcKineticEnergy */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Calculates and returns the kinetic energy represented by the current
configuration and velocity provided in ``context``. Prefer
EvalKineticEnergy() to avoid unnecessary recalculation.

See also:
    EvalKineticEnergy() for more information.)""";
        } CalcKineticEnergy;
        // Symbol: drake::systems::System::CalcNextUpdateTime
        struct /* CalcNextUpdateTime */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(This method is called by a Simulator during its calculation of the
size of the next continuous step to attempt. The System returns the
next time at which some discrete action must be taken, and records
what those actions ought to be in ``events``. Upon reaching that time,
the simulator will merge ``events`` with the other
CompositeEventCollection instances triggered through other mechanisms
(e.g. GetPerStepEvents()), and the merged CompositeEventCollection
will be passed to all event handling mechanisms.

Despite the name, the returned events includes both state-updating
events and publish events.

If there is no timed event coming, the return value is Infinity. If a
finite update time is returned, there will be at least one Event
object in the returned event collection.

``events`` cannot be null. ``events`` will be cleared on entry.)""";
        } CalcNextUpdateTime;
        // Symbol: drake::systems::System::CalcNonConservativePower
        struct /* CalcNonConservativePower */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Calculates and returns the non-conservative power represented by the
current contents of the given ``context``. Prefer
EvalNonConservativePower() to avoid unnecessary recalculation.

See also:
    EvalNonConservativePower() for more information.)""";
        } CalcNonConservativePower;
        // Symbol: drake::systems::System::CalcOutput
        struct /* CalcOutput */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Utility method that computes for *every* output port i the value y(i)
that should result from the current contents of the given Context.
Note that individual output port values can be calculated using
``get_output_port(i).Calc()``; this method invokes that for each
output port in index order. The result may depend on time and the
current values of input ports, parameters, and state variables. The
result is written to ``outputs`` which must already have been
allocated to have the right number of entries of the right types.)""";
        } CalcOutput;
        // Symbol: drake::systems::System::CalcPotentialEnergy
        struct /* CalcPotentialEnergy */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Calculates and returns the potential energy represented by the current
configuration provided in ``context``. Prefer EvalPotentialEnergy() to
avoid unnecessary recalculation.

See also:
    EvalPotentialEnergy() for more information.)""";
        } CalcPotentialEnergy;
        // Symbol: drake::systems::System::CalcTimeDerivatives
        struct /* CalcTimeDerivatives */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Calculates the time derivatives ẋ꜀ of the continuous state x꜀ into a
given output argument. Prefer EvalTimeDerivatives() instead to avoid
unnecessary recomputation.

This method solves the System equations in explicit form:

ẋ꜀ = fₑ(𝓒)

where ``𝓒 = {a, p, t, x, u}`` is the current value of the given
Context from which accuracy a, parameters p, time t, state x (``={x꜀
xd xₐ}``) and input values u are obtained.

Parameter ``context``:
    The source for time, state, inputs, etc. defining the point at
    which the derivatives should be calculated.

Parameter ``derivatives``:
    The time derivatives ẋ꜀. Must be the same size as the continuous
    state vector in ``context``.

See also:
    EvalTimeDerivatives() for more information.

See also:
    CalcImplicitTimeDerivativesResidual() for the implicit form of
    these equations.)""";
        } CalcTimeDerivatives;
        // Symbol: drake::systems::System::CalcUnrestrictedUpdate
        struct /* CalcUnrestrictedUpdate */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(This method is the public entry point for dispatching all unrestricted
update event handlers. Using all the unrestricted update handlers in
``events``, it updates *any* state variables in the ``context``, and
outputs the results to ``state``. It does not allow the dimensionality
of the state variables to change. See the documentation for
DispatchUnrestrictedUpdateHandler() for more details.

Raises:
    RuntimeError if the dimensionality of the state variables changes
    in the callback.)""";
        } CalcUnrestrictedUpdate;
        // Symbol: drake::systems::System::CalcWitnessValue
        struct /* CalcWitnessValue */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Evaluates a witness function at the given context.)""";
        } CalcWitnessValue;
        // Symbol: drake::systems::System::CheckSystemConstraintsSatisfied
        struct /* CheckSystemConstraintsSatisfied */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Returns true if ``context`` satisfies all of the registered
SystemConstraints with tolerance ``tol``.

See also:
    SystemConstraint∷CheckSatisfied.)""";
        } CheckSystemConstraintsSatisfied;
        // Symbol: drake::systems::System::Clone
        struct /* Clone */ {
          // Source: drake/systems/framework/system.h
          const char* doc_0args =
R"""(Creates a deep copy of this system.

Even though the cloned system is functionally identical, any contexts
created for this system are not compatible with the cloned system, and
vice versa.

See also:
    Context∷SetTimeStateAndParametersFrom() for how to copy context
    data between clones.

Warning:
    This implementation is somewhat incomplete at the moment. Many
    systems will not be able to be cloned, and will throw an exception
    instead. To be cloned, at minimum a system must support scalar
    conversion. See system_scalar_conversion.

The result is never nullptr.)""";
          // Source: drake/systems/framework/system.h
          const char* doc_1args =
R"""(Creates a deep copy of this system.

In contrast with the instance member function ``sys.Clone()``, this
static member function ``Clone(sys)`` is useful for C++ users to
preserve the **declared** type of the system being cloned in the
returned pointer. (For both clone overloads, the **runtime** type is
always the same.)

Even though the cloned system is functionally identical, any contexts
created for this system are not compatible with the cloned system, and
vice versa.

Warning:
    This implementation is somewhat incomplete at the moment. Many
    systems will not be able to be cloned, and will throw an exception
    instead. To be cloned, at minimum a system must support scalar
    conversion. See system_scalar_conversion.

The result is never nullptr.

Usage:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    MySystem<double> plant;
    unique_ptr<MySystem<double>> copy = System<double>∷Clone(plant);

.. raw:: html

    </details>

Template parameter ``S``:
    The specific System type to accept and return.)""";
        } Clone;
        // Symbol: drake::systems::System::CopyContinuousStateVector
        struct /* CopyContinuousStateVector */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Returns a copy of the continuous state vector x꜀ into an Eigen vector.)""";
        } CopyContinuousStateVector;
        // Symbol: drake::systems::System::CreateDefaultContext
        struct /* CreateDefaultContext */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(This convenience method allocates a context using AllocateContext()
and sets its default values using SetDefaultContext().)""";
        } CreateDefaultContext;
        // Symbol: drake::systems::System::DeclareInputPort
        struct /* DeclareInputPort */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Adds a port with the specified ``type`` and ``size`` to the input
topology.

Input port names must be unique for this system (passing in a
duplicate ``name`` will throw RuntimeError). If ``name`` is given as
kUseDefaultName, then a default value of e.g. "u2", where 2 is the
input number will be provided. An empty ``name`` is not permitted.

If the port is intended to model a random noise or disturbance input,
``random_type`` can (optionally) be used to label it as such; doing so
enables algorithms for design and analysis (e.g. state estimation) to
reason explicitly about randomness at the system level. All random
input ports are assumed to be statistically independent.

Precondition:
    ``name`` must not be empty.

Raises:
    RuntimeError for a duplicate port name.

Returns:
    the declared port.)""";
        } DeclareInputPort;
        // Symbol: drake::systems::System::DispatchDiscreteVariableUpdateHandler
        struct /* DispatchDiscreteVariableUpdateHandler */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""((Internal use only) This function dispatches all discrete update
events to the appropriate handlers. ``discrete_state`` cannot be null.
Only LeafSystem and Diagram (and some unit test code) provide
implementations and those must be ``final``.)""";
        } DispatchDiscreteVariableUpdateHandler;
        // Symbol: drake::systems::System::DispatchPublishHandler
        struct /* DispatchPublishHandler */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""((Internal use only) This function dispatches all publish events to the
appropriate handlers. Only LeafSystem and Diagram (and some unit test
code) provide implementations and those must be ``final``.)""";
        } DispatchPublishHandler;
        // Symbol: drake::systems::System::DispatchUnrestrictedUpdateHandler
        struct /* DispatchUnrestrictedUpdateHandler */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""((Internal use only) This function dispatches all unrestricted update
events to the appropriate handlers. ``state`` cannot be null. Only
LeafSystem and Diagram (and some unit test code) provide
implementations and those must be ``final``.)""";
        } DispatchUnrestrictedUpdateHandler;
        // Symbol: drake::systems::System::DoApplyDiscreteVariableUpdate
        struct /* DoApplyDiscreteVariableUpdate */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""((Internal use only) Updates the given ``context`` with the results
returned from a previous call to
DispatchDiscreteVariableUpdateHandler() that handled the given
``events``.)""";
        } DoApplyDiscreteVariableUpdate;
        // Symbol: drake::systems::System::DoApplyUnrestrictedUpdate
        struct /* DoApplyUnrestrictedUpdate */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""((Internal use only) Updates the given ``context`` with the results
returned from a previous call to DispatchUnrestrictedUpdateHandler()
that handled the given ``events``.)""";
        } DoApplyUnrestrictedUpdate;
        // Symbol: drake::systems::System::DoCalcConservativePower
        struct /* DoCalcConservativePower */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Override this method to return the rate Pc at which mechanical energy
is being converted *from* potential energy *to* kinetic energy by this
system in the given Context. By default, returns zero. Physical
systems should override. You may assume that ``context`` has already
been validated before it is passed to you here.

See EvalConservativePower() for details on what you must compute here.
In particular, this quantity must be *positive* when potential energy
is *decreasing*, and your conservative power method must *not* depend
explicitly on time or any input port values.)""";
        } DoCalcConservativePower;
        // Symbol: drake::systems::System::DoCalcImplicitTimeDerivativesResidual
        struct /* DoCalcImplicitTimeDerivativesResidual */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Override this if you have an efficient way to evaluate the implicit
time derivatives residual for this System. Otherwise the default
implementation is ``residual = proposed_derivatives −
EvalTimeDerivatives(context)``. Note that you cannot use the default
implementation if you have changed the declared residual size.

Note:
    The public method has already verified that
    ``proposed_derivatives`` is compatible with this System and that
    ``residual`` is non-null and of the declared size (as reported by
    SystemBase∷implicit_time_derivatives_residual_size()). You do not
    have to check those two conditions in your implementation, but if
    you have additional restrictions you should validate that they are
    also met.)""";
        } DoCalcImplicitTimeDerivativesResidual;
        // Symbol: drake::systems::System::DoCalcKineticEnergy
        struct /* DoCalcKineticEnergy */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Override this method for physical systems to calculate the kinetic
energy KE currently present in the motion provided in the given
Context. The default implementation returns 0 which is correct for
non-physical systems. You may assume that ``context`` has already been
validated before it is passed to you here.

See EvalKineticEnergy() for details on what you must compute here. In
particular, your kinetic energy method must *not* depend explicitly on
time or any input port values.)""";
        } DoCalcKineticEnergy;
        // Symbol: drake::systems::System::DoCalcNextUpdateTime
        struct /* DoCalcNextUpdateTime */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Computes the next time at which this System must perform a discrete
action.

Override this method if your System has any discrete actions which
must interrupt the continuous simulation. This method is called only
from the public non-virtual CalcNextUpdateTime() which will already
have error-checked the parameters so you don't have to. You may assume
that ``context`` has already been validated and ``events`` pointer is
not null.

If you override this method, you *must* set the returned ``time``. Set
it to Infinity if there are no upcoming timed events. If you return a
finite update time, you *must* put at least one Event object in the
``events`` collection. These requirements are enforced by the public
CalcNextUpdateTime() method.

Note:
    Despite the name, you must include publish events along with
    state-updating events.

The default implementation returns with the next sample time being
Infinity and no events added to ``events``.)""";
        } DoCalcNextUpdateTime;
        // Symbol: drake::systems::System::DoCalcNonConservativePower
        struct /* DoCalcNonConservativePower */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Override this method to return the rate Pnc at which work W is done on
the system by non-conservative forces. By default, returns zero.
Physical systems should override. You may assume that ``context`` has
already been validated before it is passed to you here.

See EvalNonConservativePower() for details on what you must compute
here. In particular, this quantity must be *negative* if the
non-conservative forces are *dissipative*, positive otherwise. Your
non-conservative power method can depend on anything you find in the
given Context, including time and input ports.)""";
        } DoCalcNonConservativePower;
        // Symbol: drake::systems::System::DoCalcPotentialEnergy
        struct /* DoCalcPotentialEnergy */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Override this method for physical systems to calculate the potential
energy PE currently stored in the configuration provided in the given
Context. The default implementation returns 0 which is correct for
non-physical systems. You may assume that ``context`` has already been
validated before it is passed to you here.

See EvalPotentialEnergy() for details on what you must compute here.
In particular, your potential energy method must *not* depend
explicitly on time, velocities, or any input port values.)""";
        } DoCalcPotentialEnergy;
        // Symbol: drake::systems::System::DoCalcTimeDerivatives
        struct /* DoCalcTimeDerivatives */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Override this if you have any continuous state variables x꜀ in your
concrete System to calculate their time derivatives. The
``derivatives`` vector will correspond elementwise with the state
vector ``Context.state.continuous_state.get_state()``. Thus, if the
state in the Context has second-order structure ``x꜀=[q v z]``, that
same structure applies to the derivatives.

This method is called only from the public non-virtual
CalcTimeDerivatives() which will already have error-checked the
parameters so you don't have to. In particular, implementations may
assume that the given Context is valid for this System; that the
``derivatives`` pointer is non-null, and that the referenced object
has the same constituent structure as was produced by
AllocateTimeDerivatives().

The default implementation does nothing if the ``derivatives`` vector
is size zero and aborts otherwise.)""";
        } DoCalcTimeDerivatives;
        // Symbol: drake::systems::System::DoCalcWitnessValue
        struct /* DoCalcWitnessValue */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Derived classes will implement this method to evaluate a witness
function at the given context.)""";
        } DoCalcWitnessValue;
        // Symbol: drake::systems::System::DoGetInitializationEvents
        struct /* DoGetInitializationEvents */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Implement this method to return any events to be handled at the
simulator's initialization step. ``events`` is cleared in the public
non-virtual GetInitializationEvents(). You may assume that ``context``
has already been validated and that ``events`` is not null. ``events``
can be changed freely by the overriding implementation.

The default implementation returns without changing ``events``.

See also:
    GetInitializationEvents())""";
        } DoGetInitializationEvents;
        // Symbol: drake::systems::System::DoGetMutableTargetSystemCompositeEventCollection
        struct /* DoGetMutableTargetSystemCompositeEventCollection */ {
          // Source: drake/systems/framework/system.h
          const char* doc = R"""()""";
        } DoGetMutableTargetSystemCompositeEventCollection;
        // Symbol: drake::systems::System::DoGetMutableTargetSystemState
        struct /* DoGetMutableTargetSystemState */ {
          // Source: drake/systems/framework/system.h
          const char* doc = R"""()""";
        } DoGetMutableTargetSystemState;
        // Symbol: drake::systems::System::DoGetPerStepEvents
        struct /* DoGetPerStepEvents */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Implement this method to return any events to be handled before the
simulator integrates the system's continuous state at each time step.
``events`` is cleared in the public non-virtual GetPerStepEvents()
before that method calls this function. You may assume that
``context`` has already been validated and that ``events`` is not
null. ``events`` can be changed freely by the overriding
implementation.

The default implementation returns without changing ``events``.

See also:
    GetPerStepEvents())""";
        } DoGetPerStepEvents;
        // Symbol: drake::systems::System::DoGetPeriodicEvents
        struct /* DoGetPeriodicEvents */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Implement this method to return any periodic events. ``events`` is
cleared in the public non-virtual GetPeriodicEvents(). You may assume
that ``context`` has already been validated and that ``events`` is not
null. ``events`` can be changed freely by the overriding
implementation.

The default implementation returns without changing ``events``.

See also:
    GetPeriodicEvents())""";
        } DoGetPeriodicEvents;
        // Symbol: drake::systems::System::DoGetTargetSystemCompositeEventCollection
        struct /* DoGetTargetSystemCompositeEventCollection */ {
          // Source: drake/systems/framework/system.h
          const char* doc = R"""()""";
        } DoGetTargetSystemCompositeEventCollection;
        // Symbol: drake::systems::System::DoGetTargetSystemContext
        struct /* DoGetTargetSystemContext */ {
          // Source: drake/systems/framework/system.h
          const char* doc = R"""()""";
        } DoGetTargetSystemContext;
        // Symbol: drake::systems::System::DoGetTargetSystemContinuousState
        struct /* DoGetTargetSystemContinuousState */ {
          // Source: drake/systems/framework/system.h
          const char* doc = R"""()""";
        } DoGetTargetSystemContinuousState;
        // Symbol: drake::systems::System::DoGetTargetSystemState
        struct /* DoGetTargetSystemState */ {
          // Source: drake/systems/framework/system.h
          const char* doc = R"""()""";
        } DoGetTargetSystemState;
        // Symbol: drake::systems::System::DoGetWitnessFunctions
        struct /* DoGetWitnessFunctions */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Derived classes can override this method to provide witness functions
active for the given state. The default implementation does nothing.
On entry to this function, the context will have already been
validated and the vector of witness functions will have been validated
to be both empty and non-null.)""";
        } DoGetWitnessFunctions;
        // Symbol: drake::systems::System::DoMapPeriodicEventsByTiming
        struct /* DoMapPeriodicEventsByTiming */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Implement this method to return all periodic triggered events
organized by timing.

See also:
    MapPeriodicEventsByTiming() for a detailed description of the
    returned variable.)""";
        } DoMapPeriodicEventsByTiming;
        // Symbol: drake::systems::System::DoMapQDotToVelocity
        struct /* DoMapQDotToVelocity */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Provides the substantive implementation of MapQDotToVelocity().

The default implementation uses the identity mapping, and correctly
does nothing if the System does not have second-order state variables.
It throws RuntimeError if the ``generalized_velocity`` and ``qdot``
are not the same size, but that is not enough to guarantee that the
default implementation is adequate. Child classes must override this
function if qdot != v (even if they are the same size). This occurs,
for example, if a joint uses roll-pitch-yaw rotation angles for
orientation but angular velocity for rotational rate rather than
rotation angle derivatives.

If you implement this method you are required to use no more than
``O(nq)`` time where ``nq`` is the size of ``qdot``, so that the
System can meet the performance guarantee made for the public
interface, and you must also implement DoMapVelocityToQDot().
Implementations may assume that ``qdot`` has already been validated to
be the same size as ``q`` in the given Context, and that
``generalized_velocity`` is non-null.)""";
        } DoMapQDotToVelocity;
        // Symbol: drake::systems::System::DoMapVelocityToQDot
        struct /* DoMapVelocityToQDot */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Provides the substantive implementation of MapVelocityToQDot().

The default implementation uses the identity mapping, and correctly
does nothing if the System does not have second-order state variables.
It throws RuntimeError if the ``generalized_velocity`` (`v`) and
``qdot`` are not the same size, but that is not enough to guarantee
that the default implementation is adequate. Child classes must
override this function if ``qdot != v`` (even if they are the same
size). This occurs, for example, if a joint uses roll-pitch-yaw
rotation angles for orientation but angular velocity for rotational
rate rather than rotation angle derivatives.

If you implement this method you are required to use no more than
``O(nq)`` time where ``nq`` is the size of ``qdot``, so that the
System can meet the performance guarantee made for the public
interface, and you must also implement DoMapQDotToVelocity().
Implementations may assume that ``generalized_velocity`` has already
been validated to be the same size as ``v`` in the given Context, and
that ``qdot`` is non-null.)""";
        } DoMapVelocityToQDot;
        // Symbol: drake::systems::System::EvalConservativePower
        struct /* EvalConservativePower */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Returns a reference to the cached value of the conservative power
(Pc), evaluating first if necessary using CalcConservativePower().

The returned Pc represents the rate at which mechanical energy is
being converted *from* potential energy (PE) *to* kinetic energy (KE)
by this system in the given Context. This quantity will be *positive*
when PE is *decreasing*. By definition here, conservative power may
depend only on quantities that explicitly contribute to PE and KE. See
EvalPotentialEnergy() and EvalKineticEnergy() for details.

Power due to non-conservative forces (e.g. dampers) can contribute to
the rate of change of KE. Therefore this method alone cannot be used
to determine whether KE is increasing or decreasing, only whether the
conservative power is adding or removing kinetic energy.
EvalNonConservativePower() can be used in conjunction with this method
to find the total rate of change of KE.

Non-physical systems where Pc is not meaningful will return Pc = 0.

Parameter ``context``:
    The Context whose contents may be used to evaluate conservative
    power.

Returns ``Pc``:
    The conservative power in watts (W or J/s) represented by the
    contents of the given ``context``.

See also:
    CalcConservativePower(), EvalNonConservativePower(),
    EvalPotentialEnergy(), EvalKineticEnergy())""";
        } EvalConservativePower;
        // Symbol: drake::systems::System::EvalKineticEnergy
        struct /* EvalKineticEnergy */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Returns a reference to the cached value of the kinetic energy (KE),
evaluating first if necessary using CalcKineticEnergy().

By definition here, kinetic energy depends only on "configuration" and
"velocity" (e.g. angular and translational velocity) of moving masses
which includes a subset of the state variables, and parameters that
affect configuration, velocities, or mass properties. The calculated
value may also be affected by the accuracy value supplied in the
Context. KE cannot depend explicitly on time (∂KE/∂t = 0) or input
port values (∂KE/∂u = 0).

Non-physical systems where KE is not meaningful will return KE = 0.

Parameter ``context``:
    The Context whose configuration and velocity variables may be used
    to evaluate kinetic energy.

Returns ``KE``:
    The kinetic energy in joules (J) represented by the configuration
    and velocity given in ``context``.

See also:
    CalcKineticEnergy())""";
        } EvalKineticEnergy;
        // Symbol: drake::systems::System::EvalNonConservativePower
        struct /* EvalNonConservativePower */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Returns a reference to the cached value of the non-conservative power
(Pnc), evaluating first if necessary using CalcNonConservativePower().

The returned Pnc represents the rate at which work W is done on the
system by non-conservative forces. Pnc is *negative* if the
non-conservative forces are *dissipative*, positive otherwise. Time
integration of Pnc yields work W, and the total mechanical energy ``E
= PE + KE − W`` should be conserved by any physically-correct model,
to within integration accuracy of W. Power is in watts (J/s). (Watts
are abbreviated W but not to be confused with work!) Any values in the
supplied Context (including time and input ports) may contribute to
the computation of non-conservative power.

Non-physical systems where Pnc is not meaningful will return Pnc = 0.

Parameter ``context``:
    The Context whose contents may be used to evaluate
    non-conservative power.

Returns ``Pnc``:
    The non-conservative power in watts (W or J/s) represented by the
    contents of the given ``context``.

See also:
    CalcNonConservativePower(), EvalConservativePower())""";
        } EvalNonConservativePower;
        // Symbol: drake::systems::System::EvalPotentialEnergy
        struct /* EvalPotentialEnergy */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Returns a reference to the cached value of the potential energy (PE),
evaluating first if necessary using CalcPotentialEnergy().

By definition here, potential energy depends only on "configuration"
(e.g. orientation and position), which includes a subset of the state
variables, and parameters that affect configuration or conservative
forces (such as lengths and masses). The calculated value may also be
affected by the accuracy value supplied in the Context. PE cannot
depend explicitly on time (∂PE/∂t = 0), velocities (∂PE/∂v = 0), or
input port values (∂PE/∂u = 0).

Non-physical systems where PE is not meaningful will return PE = 0.

Parameter ``context``:
    The Context whose configuration variables may be used to evaluate
    potential energy.

Returns ``PE``:
    The potential energy in joules (J) represented by the
    configuration given in ``context``.

See also:
    CalcPotentialEnergy())""";
        } EvalPotentialEnergy;
        // Symbol: drake::systems::System::EvalTimeDerivatives
        struct /* EvalTimeDerivatives */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Returns a reference to the cached value of the continuous state
variable time derivatives, evaluating first if necessary using
CalcTimeDerivatives().

This method returns the time derivatives ẋ꜀ of the continuous state
x꜀. The referenced return object will correspond elementwise with the
continuous state in the given Context. Thus, if the state in the
Context has second-order structure ``x꜀ = [q v z]``, that same
structure applies to the derivatives so we will have ``ẋ꜀ = [q̇ ̇v̇
ż]``.

Parameter ``context``:
    The Context whose time, input port, parameter, state, and accuracy
    values may be used to evaluate the derivatives.

Returns ``xcdot``:
    Time derivatives ẋ꜀ of x꜀ returned as a reference to an object of
    the same type and size as `context`'s continuous state.

See also:
    BatchEvalTimeDerivatives() for a batch version of this method.

See also:
    CalcTimeDerivatives(), CalcImplicitTimeDerivativesResidual(),
    get_time_derivatives_cache_entry())""";
        } EvalTimeDerivatives;
        // Symbol: drake::systems::System::EvalUniquePeriodicDiscreteUpdate
        struct /* EvalUniquePeriodicDiscreteUpdate */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(If this System contains a unique periodic timing for discrete update
events, this function executes the handlers for those periodic events
to determine what their effect would be. Returns a reference to the
discrete variable cache entry containing what values the discrete
variables would have if these periodic events were triggered.

Note that this function *does not* change the value of the discrete
variables in the supplied Context. However, you can apply the result
to the Context like this:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    const DiscreteValues<T>& updated =
    system.EvalUniquePeriodicDiscreteUpdate(context);
    context.SetDiscreteState(updated);

.. raw:: html

    </details>

You can write the updated values to a different Context than the one
you used to calculate the update; the requirement is only that the
discrete state in the destination has the same structure (number of
groups and size of each group).

You can use GetUniquePeriodicDiscreteUpdateAttribute() to check
whether you can call EvalUniquePeriodicDiscreteUpdate() safely, and to
find the unique periodic timing information (offset and period).

Warning:
    Even if we find a unique discrete update timing as described
    above, there may also be unrestricted updates performed with that
    timing or other timings. (Unrestricted updates can modify any
    state variables *including* discrete variables.) Also, there may
    be trigger types other than periodic that can modify discrete
    variables. This function does not attempt to look for any of
    those; they are simply ignored. If you are concerned with those,
    you can use GetPerStepEvents(), GetInitializationEvents(), and
    GetPeriodicEvents() to get a more comprehensive picture of the
    event landscape.

Parameter ``context``:
    The Context containing the current System state and the mutable
    cache space into which the result is written. The current state is
    *not* modified, though the cache entry may be updated.

Returns:
    A reference to the DiscreteValues cache space in ``context``
    containing the result of applying the discrete update event
    handlers to the current discrete variable values.

Note:
    The referenced cache entry is recalculated if anything in the
    given Context has changed since last calculation. Subsequent calls
    just return the already-calculated value.

Raises:
    RuntimeError if there is not exactly one periodic timing in this
    System (which may be a Diagram) that triggers discrete update
    events.

Raises:
    RuntimeError if it invokes an event handler that returns status
    indicating failure.

@par Implementation If recalculation is needed, copies the current
discrete state values into preallocated ``context`` cache space.
Applies the discrete update event handlers (in an unspecified order)
to the cache copy, possibly updating it. Returns a reference to the
possibly-updated cache space.

See also:
    BatchEvalUniquePeriodicDiscreteUpdate() for a batch version of
    this method.

See also:
    GetUniquePeriodicDiscreteUpdateAttribute(), GetPeriodicEvents())""";
        } EvalUniquePeriodicDiscreteUpdate;
        // Symbol: drake::systems::System::EvalVectorInput
        struct /* EvalVectorInput */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Returns the value of the vector-valued input port with the given
``port_index`` as a BasicVector or a specific subclass ``Vec`` derived
from BasicVector. Causes the value to become up to date first if
necessary. See EvalAbstractInput() for more information.

The result is returned as a pointer to the input port's value of type
``Vec<T>`` or nullptr if the port is not connected.

Precondition:
    ``port_index`` selects an existing input port of this System.

Precondition:
    the port must have been declared to be vector-valued.

Precondition:
    the port's value must be of type Vec<T>.

Template parameter ``Vec``:
    The template type of the input vector, which must be a subclass of
    BasicVector.)""";
        } EvalVectorInput;
        // Symbol: drake::systems::System::ExecuteForcedEvents
        struct /* ExecuteForcedEvents */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(This method triggers all of the forced events registered with this
System (which might be a Diagram). Ordering and status return handling
mimic the Simulator: unrestricted events are processed first, then
discrete update events, then publish events. "Reached termination"
status returns are ignored.

An option is provided to suppress publish events. This can be useful,
for example, to update state in a Diagram without triggering a
visualization.

Parameter ``context``:
    The Context supplied to the handlers and modified in place on
    return.

Raises:
    RuntimeError if it invokes an event handler that returns status
    indicating failure.)""";
        } ExecuteForcedEvents;
        // Symbol: drake::systems::System::ExecuteInitializationEvents
        struct /* ExecuteInitializationEvents */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(This method triggers all of the initialization events returned by
GetInitializationEvents(). The method allocates temporary storage to
perform the updates, and is intended only as a convenience method for
callers who do not want to use the full Simulator workflow.

Note that this is not fully equivalent to Simulator∷Initialize()
because *only* initialization events are handled here, while
Simulator∷Initialize() also processes other events associated with
time zero. Also, "reached termination" returns are ignored here.

Parameter ``context``:
    The Context supplied to the handlers and modified in place on
    return.

Raises:
    RuntimeError if it invokes an event handler that returns status
    indicating failure.)""";
        } ExecuteInitializationEvents;
        // Symbol: drake::systems::System::FindUniquePeriodicDiscreteUpdatesOrThrow
        struct /* FindUniquePeriodicDiscreteUpdatesOrThrow */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""((Internal use only) Static interface to
DoFindUniquePeriodicDiscreteUpdatesOrThrow() to allow a Diagram to
invoke that private method on its subsystems.)""";
        } FindUniquePeriodicDiscreteUpdatesOrThrow;
        // Symbol: drake::systems::System::FixInputPortsFrom
        struct /* FixInputPortsFrom */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Fixes all of the input ports in ``target_context`` to their current
values in ``other_context``, as evaluated by ``other_system``.

Raises:
    RuntimeError unless ``other_context`` and ``target_context`` both
    have the same shape as this System, and the ``other_system``.
    Ignores disconnected inputs.

Raises:
    RuntimeError if ``this`` system's scalar type T != double and
    ``other_system`` has any abstract input ports whose contained type
    depends on scalar type.)""";
        } FixInputPortsFrom;
        // Symbol: drake::systems::System::ForcedPublish
        struct /* ForcedPublish */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""((Advanced) Manually triggers any PublishEvent that has trigger type
kForced. Invokes the publish event dispatcher on this System with the
given Context.

The default dispatcher will invoke the handlers (if any) associated
with each force-triggered event.

Note:
    There will always be at least one force-triggered event, though
    with no associated handler (so will do nothing when triggered).

The Simulator can be configured to call this in Simulator∷Initialize()
and at the start of each continuous integration step. See the
Simulator API for more details.

Raises:
    RuntimeError if it invokes an event handler that returns status
    indicating failure.

See also:
    Publish(), CalcForcedDiscreteVariableUpdate(),
    CalcForcedUnrestrictedUpdate())""";
        } ForcedPublish;
        // Symbol: drake::systems::System::GetInitializationEvents
        struct /* GetInitializationEvents */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(This method is called by Simulator∷Initialize() to gather all update
and publish events that need to be handled at initialization before
the simulator starts integration.

``events`` cannot be null. ``events`` will be cleared on entry.

See also:
    GetPeriodicEvents(), GetPerStepEvents())""";
        } GetInitializationEvents;
        // Symbol: drake::systems::System::GetInputPort
        struct /* GetInputPort */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Returns the typed input port with the unique name ``port_name``. The
current implementation performs a linear search over strings; prefer
get_input_port() when performance is a concern.

Raises:
    RuntimeError if port_name is not found.)""";
        } GetInputPort;
        // Symbol: drake::systems::System::GetMutableOutputVector
        struct /* GetMutableOutputVector */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Returns a mutable Eigen expression for a vector valued output port
with index ``port_index`` in this system. All input ports that
directly depend on this output port will be notified that upstream
data has changed, and may invalidate cache entries as a result.)""";
        } GetMutableOutputVector;
        // Symbol: drake::systems::System::GetMutableSubsystemContext
        struct /* GetMutableSubsystemContext */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Returns a mutable reference to the subcontext that corresponds to the
contained System ``subsystem``.

Raises:
    RuntimeError if ``subsystem`` not contained in ``this`` System.

Precondition:
    The given ``context`` is valid for use with ``this`` System.)""";
        } GetMutableSubsystemContext;
        // Symbol: drake::systems::System::GetMyContextFromRoot
        struct /* GetMyContextFromRoot */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Returns the const Context for ``this`` subsystem, given a root
context. If ``this`` System is already the top level (root) System,
just returns ``root_context``. (A root Context is one that does not
have a parent Context.)

Raises:
    RuntimeError if the given ``root_context`` is not actually a root
    context.

See also:
    GetSubsystemContext())""";
        } GetMyContextFromRoot;
        // Symbol: drake::systems::System::GetMyMutableContextFromRoot
        struct /* GetMyMutableContextFromRoot */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Returns the mutable subsystem context for ``this`` system, given a
root context.

See also:
    GetMyContextFromRoot())""";
        } GetMyMutableContextFromRoot;
        // Symbol: drake::systems::System::GetOutputPort
        struct /* GetOutputPort */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Returns the typed output port with the unique name ``port_name``. The
current implementation performs a linear search over strings; prefer
get_output_port() when performance is a concern.

Raises:
    RuntimeError if port_name is not found.)""";
        } GetOutputPort;
        // Symbol: drake::systems::System::GetPerStepEvents
        struct /* GetPerStepEvents */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(This method is called by Simulator∷Initialize() to gather all update
and publish events that are to be handled in AdvanceTo() at the point
before Simulator integrates continuous state. It is assumed that these
events remain constant throughout the simulation. The "step" here
refers to the major time step taken by the Simulator. During every
simulation step, the simulator will merge ``events`` with the event
collections populated by other types of event triggering mechanism
(e.g., CalcNextUpdateTime()), and the merged CompositeEventCollection
objects will be passed to the appropriate handlers before Simulator
integrates the continuous state.

``events`` cannot be null. ``events`` will be cleared on entry.

See also:
    GetPeriodicEvents(), GetInitializationEvents())""";
        } GetPerStepEvents;
        // Symbol: drake::systems::System::GetPeriodicEvents
        struct /* GetPeriodicEvents */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Returns all periodic events in this System. This includes publish,
discrete update, and unrestricted update events.

``events`` cannot be null. ``events`` will be cleared on entry.

See also:
    GetPerStepEvents(), GetInitializationEvents())""";
        } GetPeriodicEvents;
        // Symbol: drake::systems::System::GetSubsystemContext
        struct /* GetSubsystemContext */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Returns a const reference to the subcontext that corresponds to the
contained System ``subsystem``.

Raises:
    RuntimeError if ``subsystem`` not contained in ``this`` System.

Precondition:
    The given ``context`` is valid for use with ``this`` System.)""";
        } GetSubsystemContext;
        // Symbol: drake::systems::System::GetUniquePeriodicDiscreteUpdateAttribute
        struct /* GetUniquePeriodicDiscreteUpdateAttribute */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Determines whether there exists a unique periodic timing (offset and
period) that triggers one or more discrete update events (and, if so,
returns that unique periodic timing). Thus, this method can be used
(1) as a test to determine whether a system's dynamics are at least
partially governed by difference equations, and (2) to obtain the
difference equation update times. Use
EvalUniquePeriodicDiscreteUpdate() if you want to determine the actual
effects of triggering these events.

Warning:
    Even if we find a unique discrete update timing as described
    above, there may also be unrestricted updates performed with that
    timing or other timings. (Unrestricted updates can modify any
    state variables *including* discrete variables.) Also, there may
    be trigger types other than periodic that can modify discrete
    variables. This function does not attempt to look for any of
    those; they are simply ignored. If you are concerned with those,
    you can use GetPerStepEvents(), GetInitializationEvents(), and
    GetPeriodicEvents() to get a more comprehensive picture of the
    event landscape.

Returns:
    optional<PeriodicEventData> Contains the unique periodic trigger
    timing if it exists, otherwise ``nullopt``.

See also:
    EvalUniquePeriodicDiscreteUpdate(), IsDifferenceEquationSystem())""";
        } GetUniquePeriodicDiscreteUpdateAttribute;
        // Symbol: drake::systems::System::GetWitnessFunctions
        struct /* GetWitnessFunctions */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Gets the witness functions active for the given state.
DoGetWitnessFunctions() does the actual work. The vector of active
witness functions are expected to change only upon an unrestricted
update.

Parameter ``context``:
    a valid context for the System (aborts if not true).

Parameter ``w``:
    a valid pointer to an empty vector that will store pointers to the
    witness functions active for the current state. The method aborts
    if witnesses is null or non-empty.)""";
        } GetWitnessFunctions;
        // Symbol: drake::systems::System::HandlePostConstructionScalarConversion
        struct /* HandlePostConstructionScalarConversion */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""((Internal use only) Scalar conversion (e.g., ToAutoDiffXd) will first
call the SystemScalarConverter to construct the converted system, and
then call this function for any post-construction cleanup.)""";
        } HandlePostConstructionScalarConversion;
        // Symbol: drake::systems::System::HasAnyDirectFeedthrough
        struct /* HasAnyDirectFeedthrough */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Returns ``True`` if any of the inputs to the system might be directly
fed through to any of its outputs and ``False`` otherwise.)""";
        } HasAnyDirectFeedthrough;
        // Symbol: drake::systems::System::HasDirectFeedthrough
        struct /* HasDirectFeedthrough */ {
          // Source: drake/systems/framework/system.h
          const char* doc_1args =
R"""(Returns true if there might be direct-feedthrough from any input port
to the given ``output_port``, and false otherwise.)""";
          // Source: drake/systems/framework/system.h
          const char* doc_2args =
R"""(Returns true if there might be direct-feedthrough from the given
``input_port`` to the given ``output_port``, and false otherwise.)""";
        } HasDirectFeedthrough;
        // Symbol: drake::systems::System::HasInputPort
        struct /* HasInputPort */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Returns true iff the system has an InputPort of the given
``port_name``.)""";
        } HasInputPort;
        // Symbol: drake::systems::System::HasOutputPort
        struct /* HasOutputPort */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Returns true iff the system has an OutputPort of the given
``port_name``.)""";
        } HasOutputPort;
        // Symbol: drake::systems::System::IsDifferenceEquationSystem
        struct /* IsDifferenceEquationSystem */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Returns true iff the state dynamics of this system are governed
exclusively by a difference equation on a single discrete state group
and with a unique periodic update (having zero offset). E.g., it is
amenable to analysis of the form:

x[n+1] = f(n, x[n], u[n], w[n]; p)

where t is time, x is (discrete) state, u is a vector input, w is
random (disturbance) input, and p are parameters. Note that we do NOT
consider the number of input ports here, because in practice many
systems of interest (e.g. MultibodyPlant) have input ports that are
safely treated as constant during the analysis. Consider using
get_input_port_selection() to choose one.

Warning:
    In determining whether this system is governed as above, we do not
    consider unrestricted updates nor any update events that have
    trigger types other than periodic. See
    GetUniquePeriodicDiscreteUpdateAttribute() for more information.

Parameter ``time_period``:
    if non-null, then iff the function returns ``True``, then
    time_period is set to the period data returned from
    GetUniquePeriodicDiscreteUpdateAttribute(). If the function
    returns ``False`` (the system is not a difference equation
    system), then ``time_period`` does not receive a value.

See also:
    GetUniquePeriodicDiscreteUpdateAttribute()

See also:
    EvalUniquePeriodicDiscreteUpdate())""";
        } IsDifferenceEquationSystem;
        // Symbol: drake::systems::System::IsDifferentialEquationSystem
        struct /* IsDifferentialEquationSystem */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Returns true iff the state dynamics of this system are governed
exclusively by a differential equation. E.g., it is amenable to
analysis of the form:

ẋ = f(t, x(t), u(t), w(t); p),

where t is time, x is (continuous) state, u is a vector input, w is
random (disturbance) input, and p are parameters. This requires that
it has no discrete nor abstract states, and no abstract input ports.

Warning:
    In determining whether this system is governed as above, we do not
    consider unrestricted updates which could potentially update the
    state.)""";
        } IsDifferentialEquationSystem;
        // Symbol: drake::systems::System::MapPeriodicEventsByTiming
        struct /* MapPeriodicEventsByTiming */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Maps all periodic triggered events for a System, organized by timing.
Each unique periodic timing attribute (offset and period) is mapped to
the set of Event objects that are triggered with that timing. Those
may include a mix of Publish, DiscreteUpdate, and UnrestrictedUpdate
events.

Parameter ``context``:
    Optional Context to pass on to Event selection functions; not
    commonly needed.)""";
        } MapPeriodicEventsByTiming;
        // Symbol: drake::systems::System::MapQDotToVelocity
        struct /* MapQDotToVelocity */ {
          // Source: drake/systems/framework/system.h
          const char* doc_was_unable_to_choose_unambiguous_names =
R"""(Transforms the time derivative ``qdot`` of the generalized
configuration ``q`` to generalized velocities ``v``. `v` and ``qdot``
are related linearly by ``qdot = N(q) * v``, where ``N`` is a block
diagonal matrix. For example, in a multibody system there will be one
block of ``N`` per tree joint. Although ``N`` is not necessarily
square, its left pseudo-inverse ``N+`` can be used to invert that
relationship without residual error, provided that ``qdot`` is in the
range space of ``N`` (that is, if it *could* have been produced as
``qdot=N*v`` for some ``v``). Using the configuration ``q`` from the
given Context this method calculates ``v = N+ * qdot`` (where
``N+=N+(q)``) for a given ``qdot``. This computation requires only
``O(nq)`` time where ``nq`` is the size of ``qdot``. Note that this
method does not take ``qdot`` from the Context.

See the alternate signature if you already have ``qdot`` in an Eigen
VectorX object; this signature will copy the VectorBase into an Eigen
object before performing the computation.

See also:
    MapVelocityToQDot())""";
        } MapQDotToVelocity;
        // Symbol: drake::systems::System::MapVelocityToQDot
        struct /* MapVelocityToQDot */ {
          // Source: drake/systems/framework/system.h
          const char* doc_was_unable_to_choose_unambiguous_names =
R"""(Transforms a given generalized velocity ``v`` to the time derivative
``qdot`` of the generalized configuration ``q`` taken from the
supplied Context. ``v`` and ``qdot`` are related linearly by ``qdot =
N(q) * v``, where ``N`` is a block diagonal matrix. For example, in a
multibody system there will be one block of ``N`` per tree joint. This
computation requires only ``O(nq)`` time where ``nq`` is the size of
``qdot``. Note that ``v`` is *not* taken from the Context; it is given
as an argument here.

See the alternate signature if you already have the generalized
velocity in an Eigen VectorX object; this signature will copy the
VectorBase into an Eigen object before performing the computation.

See also:
    MapQDotToVelocity())""";
        } MapVelocityToQDot;
        // Symbol: drake::systems::System::Publish
        struct /* Publish */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(This method is the public entry point for dispatching all publish
event handlers. It checks the validity of ``context``, and directly
calls DispatchPublishHandler. ``events`` is a homogeneous collection
of publish events.

Note:
    When publishing is triggered at particular times, those times
    likely will not coincide with integrator step times. A Simulator
    may interpolate to generate a suitable Context, or it may adjust
    the integrator step size so that a step begins exactly at the next
    publication time. In the latter case the change in step size may
    affect the numerical result somewhat since a smaller integrator
    step produces a more accurate solution.)""";
        } Publish;
        // Symbol: drake::systems::System::Scalar
        struct /* Scalar */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(The scalar type with which this System was instantiated.)""";
        } Scalar;
        // Symbol: drake::systems::System::SetDefaultContext
        struct /* SetDefaultContext */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Sets Context fields to their default values. User code should not
override.)""";
        } SetDefaultContext;
        // Symbol: drake::systems::System::SetDefaultParameters
        struct /* SetDefaultParameters */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Assigns default values to all parameters. Overrides must not change
the number of parameters.

Warning:
    ``parameters`` *may be* a mutable view into ``context``. Don't
    assume that evaluating ``context`` will be independent of writing
    to ``parameters``.)""";
        } SetDefaultParameters;
        // Symbol: drake::systems::System::SetDefaultState
        struct /* SetDefaultState */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Assigns default values to all elements of the state. Overrides must
not change the number of state variables. The context's default
parameters will have already been set.

Warning:
    ``state`` *may be* a mutable view into ``context``. Don't assume
    that evaluating ``context`` will be independent of writing to
    ``state``.)""";
        } SetDefaultState;
        // Symbol: drake::systems::System::SetRandomContext
        struct /* SetRandomContext */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Sets Context fields to random values. User code should not override.)""";
        } SetRandomContext;
        // Symbol: drake::systems::System::SetRandomParameters
        struct /* SetRandomParameters */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Assigns random values to all parameters. This default implementation
calls SetDefaultParameters; override this method to provide random
parameters using the stdc++ random library, e.g.:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    std∷uniform_real_distribution<T> uniform();
    parameters->get_mutable_numeric_parameter(0)
    ->SetAtIndex(0, uniform(*generator));

.. raw:: html

    </details>

Overrides must not change the number of state variables.

See also:
    stochastic_systems)""";
        } SetRandomParameters;
        // Symbol: drake::systems::System::SetRandomState
        struct /* SetRandomState */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Assigns random values to all elements of the state. This default
implementation calls SetDefaultState; override this method to provide
random initial conditions using the stdc++ random library, e.g.:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    std∷normal_distribution<T> gaussian();
    state->get_mutable_continuous_state()->get_mutable_vector()
    ->SetAtIndex(0, gaussian(*generator));

.. raw:: html

    </details>

Overrides must not change the number of state variables.

See also:
    stochastic_systems)""";
        } SetRandomState;
        // Symbol: drake::systems::System::System<T>
        struct /* ctor */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Constructs an empty System base class object and allocates base class
resources, possibly supporting scalar-type conversion support
(AutoDiff, etc.) using ``converter``.

See system_scalar_conversion for detailed background and examples
related to scalar-type conversion support.)""";
        } ctor;
        // Symbol: drake::systems::System::ToAutoDiffXd
        struct /* ToAutoDiffXd */ {
          // Source: drake/systems/framework/system.h
          const char* doc_0args =
R"""(Creates a deep copy of this System, transmogrified to use the autodiff
scalar type, with a dynamic-sized vector of partial derivatives. The
result is never nullptr.

Raises:
    RuntimeError if this System does not support autodiff

See system_scalar_conversion for detailed background and examples
related to scalar-type conversion support.)""";
          // Source: drake/systems/framework/system.h
          const char* doc_1args =
R"""(Creates a deep copy of ``from``, transmogrified to use the autodiff
scalar type, with a dynamic-sized vector of partial derivatives. The
result is never nullptr.

Raises:
    RuntimeError if ``from`` does not support autodiff

Usage:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    MySystem<double> plant;
    std∷unique_ptr<MySystem<AutoDiffXd>> ad_plant =
    systems∷System<double>∷ToAutoDiffXd(plant);

.. raw:: html

    </details>

Template parameter ``S``:
    The specific System type to accept and return.

See system_scalar_conversion for detailed background and examples
related to scalar-type conversion support.)""";
        } ToAutoDiffXd;
        // Symbol: drake::systems::System::ToAutoDiffXdMaybe
        struct /* ToAutoDiffXdMaybe */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Creates a deep copy of this system exactly like ToAutoDiffXd(), but
returns nullptr if this System does not support autodiff, instead of
throwing an exception.)""";
        } ToAutoDiffXdMaybe;
        // Symbol: drake::systems::System::ToScalarType
        struct /* ToScalarType */ {
          // Source: drake/systems/framework/system.h
          const char* doc_0args =
R"""(Creates a deep copy of this System, transmogrified to use the scalar
type selected by a template parameter. The result is never nullptr.

Raises:
    RuntimeError if this System does not support the destination type.

Template parameter ``U``:
    The destination scalar type. For a list of supported types, see
    the default_scalars "default scalars".

See system_scalar_conversion for detailed background and examples
related to scalar-type conversion support.)""";
          // Source: drake/systems/framework/system.h
          const char* doc_1args =
R"""(Creates a deep copy of ``from``, transmogrified to use the scalar type
selected by a template parameter. The result is never nullptr.

Raises:
    RuntimeError if ``from`` does not support the destination type.

Usage:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    MySystem<double> plant;
    auto sym_plant =
    systems∷System<double>∷ToScalarType<symbolic∷Expression>(plant);

.. raw:: html

    </details>

Template parameter ``U``:
    The destination scalar type. For a list of supported types, see
    the default_scalars "default scalars".

Template parameter ``S``:
    The specific System pointer type to return.

See system_scalar_conversion for detailed background and examples
related to scalar-type conversion support.)""";
        } ToScalarType;
        // Symbol: drake::systems::System::ToScalarTypeMaybe
        struct /* ToScalarTypeMaybe */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Creates a deep copy of this system exactly like ToScalarType(), but
returns nullptr if this System does not support the destination type,
instead of throwing an exception.

Template parameter ``U``:
    The destination scalar type. For a list of supported types, see
    the default_scalars "default scalars".)""";
        } ToScalarTypeMaybe;
        // Symbol: drake::systems::System::ToSymbolic
        struct /* ToSymbolic */ {
          // Source: drake/systems/framework/system.h
          const char* doc_0args =
R"""(Creates a deep copy of this System, transmogrified to use the symbolic
scalar type. The result is never nullptr.

Raises:
    RuntimeError if this System does not support symbolic

See system_scalar_conversion for detailed background and examples
related to scalar-type conversion support.)""";
          // Source: drake/systems/framework/system.h
          const char* doc_1args =
R"""(Creates a deep copy of ``from``, transmogrified to use the symbolic
scalar type. The result is never nullptr.

Raises:
    RuntimeError if ``from`` does not support symbolic

Usage:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    MySystem<double> plant;
    std∷unique_ptr<MySystem<symbolic∷Expression>> sym_plant =
    systems∷System<double>∷ToSymbolic(plant);

.. raw:: html

    </details>

Template parameter ``S``:
    The specific System pointer type to return.

See system_scalar_conversion for detailed background and examples
related to scalar-type conversion support.)""";
        } ToSymbolic;
        // Symbol: drake::systems::System::ToSymbolicMaybe
        struct /* ToSymbolicMaybe */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Creates a deep copy of this system exactly like ToSymbolic(), but
returns nullptr if this System does not support symbolic, instead of
throwing an exception.)""";
        } ToSymbolicMaybe;
        // Symbol: drake::systems::System::forced_discrete_update_events_exist
        struct /* forced_discrete_update_events_exist */ {
          // Source: drake/systems/framework/system.h
          const char* doc = R"""()""";
        } forced_discrete_update_events_exist;
        // Symbol: drake::systems::System::forced_publish_events_exist
        struct /* forced_publish_events_exist */ {
          // Source: drake/systems/framework/system.h
          const char* doc = R"""()""";
        } forced_publish_events_exist;
        // Symbol: drake::systems::System::forced_unrestricted_update_events_exist
        struct /* forced_unrestricted_update_events_exist */ {
          // Source: drake/systems/framework/system.h
          const char* doc = R"""()""";
        } forced_unrestricted_update_events_exist;
        // Symbol: drake::systems::System::get_constraint
        struct /* get_constraint */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Returns the constraint at index ``constraint_index``.

Raises:
    RuntimeError for an invalid constraint_index.)""";
        } get_constraint;
        // Symbol: drake::systems::System::get_forced_discrete_update_events
        struct /* get_forced_discrete_update_events */ {
          // Source: drake/systems/framework/system.h
          const char* doc = R"""()""";
        } get_forced_discrete_update_events;
        // Symbol: drake::systems::System::get_forced_publish_events
        struct /* get_forced_publish_events */ {
          // Source: drake/systems/framework/system.h
          const char* doc = R"""()""";
        } get_forced_publish_events;
        // Symbol: drake::systems::System::get_forced_unrestricted_update_events
        struct /* get_forced_unrestricted_update_events */ {
          // Source: drake/systems/framework/system.h
          const char* doc = R"""()""";
        } get_forced_unrestricted_update_events;
        // Symbol: drake::systems::System::get_input_port
        struct /* get_input_port */ {
          // Source: drake/systems/framework/system.h
          const char* doc_2args =
R"""(Returns the typed input port at index ``port_index``.

Parameter ``warn_deprecated``:
    Whether or not to print a warning in case the port was marked as
    deprecated.)""";
          // Source: drake/systems/framework/system.h
          const char* doc_0args =
R"""(Convenience method for the case of exactly one input port. This
function ignores deprecated ports, unless there is only one port in
which case it will return the deprecated port.)""";
        } get_input_port;
        // Symbol: drake::systems::System::get_input_port_selection
        struct /* get_input_port_selection */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Returns the typed input port specified by the InputPortSelection or by
the InputPortIndex. Returns nullptr if no port is selected. This is
provided as a convenience method since many algorithms provide the
same common default or optional port semantics.)""";
        } get_input_port_selection;
        // Symbol: drake::systems::System::get_mutable_forced_discrete_update_events
        struct /* get_mutable_forced_discrete_update_events */ {
          // Source: drake/systems/framework/system.h
          const char* doc = R"""()""";
        } get_mutable_forced_discrete_update_events;
        // Symbol: drake::systems::System::get_mutable_forced_publish_events
        struct /* get_mutable_forced_publish_events */ {
          // Source: drake/systems/framework/system.h
          const char* doc = R"""()""";
        } get_mutable_forced_publish_events;
        // Symbol: drake::systems::System::get_mutable_forced_unrestricted_update_events
        struct /* get_mutable_forced_unrestricted_update_events */ {
          // Source: drake/systems/framework/system.h
          const char* doc = R"""()""";
        } get_mutable_forced_unrestricted_update_events;
        // Symbol: drake::systems::System::get_mutable_system_scalar_converter
        struct /* get_mutable_system_scalar_converter */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Returns the SystemScalarConverter for ``this`` system.)""";
        } get_mutable_system_scalar_converter;
        // Symbol: drake::systems::System::get_output_port
        struct /* get_output_port */ {
          // Source: drake/systems/framework/system.h
          const char* doc_2args =
R"""(Returns the typed output port at index ``port_index``.

Parameter ``warn_deprecated``:
    Whether or not to print a warning in case the port was marked as
    deprecated.)""";
          // Source: drake/systems/framework/system.h
          const char* doc_0args =
R"""(Convenience method for the case of exactly one output port. This
function ignores deprecated ports, unless there is only one port in
which case it will return the deprecated port.)""";
        } get_output_port;
        // Symbol: drake::systems::System::get_output_port_selection
        struct /* get_output_port_selection */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Returns the typed output port specified by the OutputPortSelection or
by the OutputPortIndex. Returns nullptr if no port is selected. This
is provided as a convenience method since many algorithms provide the
same common default or optional port semantics.)""";
        } get_output_port_selection;
        // Symbol: drake::systems::System::get_system_scalar_converter
        struct /* get_system_scalar_converter */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""((Advanced) Returns the SystemScalarConverter for this object. This is
an expert-level API intended for framework authors. Most users should
prefer the convenience helpers such as System∷ToAutoDiffXd.)""";
        } get_system_scalar_converter;
        // Symbol: drake::systems::System::get_time_derivatives_cache_entry
        struct /* get_time_derivatives_cache_entry */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""((Advanced) Returns the CacheEntry used to cache time derivatives for
EvalTimeDerivatives().)""";
        } get_time_derivatives_cache_entry;
        // Symbol: drake::systems::System::num_constraints
        struct /* num_constraints */ {
          // Source: drake/systems/framework/system.h
          const char* doc =
R"""(Returns the number of constraints specified for the system.)""";
        } num_constraints;
        // Symbol: drake::systems::System::set_forced_discrete_update_events
        struct /* set_forced_discrete_update_events */ {
          // Source: drake/systems/framework/system.h
          const char* doc = R"""()""";
        } set_forced_discrete_update_events;
        // Symbol: drake::systems::System::set_forced_publish_events
        struct /* set_forced_publish_events */ {
          // Source: drake/systems/framework/system.h
          const char* doc = R"""()""";
        } set_forced_publish_events;
        // Symbol: drake::systems::System::set_forced_unrestricted_update_events
        struct /* set_forced_unrestricted_update_events */ {
          // Source: drake/systems/framework/system.h
          const char* doc = R"""()""";
        } set_forced_unrestricted_update_events;
      } System;
      // Symbol: drake::systems::SystemBase
      struct /* SystemBase */ {
        // Source: drake/systems/framework/system_base.h
        const char* doc =
R"""(Provides non-templatized functionality shared by the templatized
System classes.

Terminology: in general a Drake System is a tree structure composed of
"subsystems", which are themselves System objects. The corresponding
Context is a parallel tree structure composed of "subcontexts", which
are themselves Context objects. There is a one-to-one correspondence
between subsystems and subcontexts. Within a given System (Context),
its child subsystems (subcontexts) are indexed using a SubsystemIndex;
there is no separate SubcontextIndex since the numbering must be
identical.)""";
        // Symbol: drake::systems::SystemBase::AddAbstractParameter
        struct /* AddAbstractParameter */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""((Internal use only) Assigns a ticket to a new abstract parameter with
the given ``index``.

Precondition:
    The supplied index must be the next available one; that is,
    indexes must be assigned sequentially.)""";
        } AddAbstractParameter;
        // Symbol: drake::systems::SystemBase::AddAbstractState
        struct /* AddAbstractState */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""((Internal use only) Assigns a ticket to a new abstract state variable
with the given ``index``.

Precondition:
    The supplied index must be the next available one; that is,
    indexes must be assigned sequentially.)""";
        } AddAbstractState;
        // Symbol: drake::systems::SystemBase::AddDiscreteStateGroup
        struct /* AddDiscreteStateGroup */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""((Internal use only) Assigns a ticket to a new discrete variable group
with the given ``index``.

Precondition:
    The supplied index must be the next available one; that is,
    indexes must be assigned sequentially.)""";
        } AddDiscreteStateGroup;
        // Symbol: drake::systems::SystemBase::AddInputPort
        struct /* AddInputPort */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""((Internal use only) Adds an already-constructed input port to this
System. Insists that the port already contains a reference to this
System, and that the port's index is already set to the next available
input port index for this System, that the port name is unique (just
within this System), and that the port name is non-empty.)""";
        } AddInputPort;
        // Symbol: drake::systems::SystemBase::AddNumericParameter
        struct /* AddNumericParameter */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""((Internal use only) Assigns a ticket to a new numeric parameter with
the given ``index``.

Precondition:
    The supplied index must be the next available one; that is,
    indexes must be assigned sequentially.)""";
        } AddNumericParameter;
        // Symbol: drake::systems::SystemBase::AddOutputPort
        struct /* AddOutputPort */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""((Internal use only) Adds an already-constructed output port to this
System. Insists that the port already contains a reference to this
System, and that the port's index is already set to the next available
output port index for this System, and that the name of the port is
unique.

Raises:
    RuntimeError if the name of the output port is not unique.)""";
        } AddOutputPort;
        // Symbol: drake::systems::SystemBase::AllocateContext
        struct /* AllocateContext */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns a Context suitable for use with this System. Context resources
are allocated based on resource requests that were made during System
construction.)""";
        } AllocateContext;
        // Symbol: drake::systems::SystemBase::ContextSizes
        struct /* ContextSizes */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Return type for get_context_sizes(). Initialized to zero and equipped
with a += operator for Diagram use in aggregation.)""";
          // Symbol: drake::systems::SystemBase::ContextSizes::num_abstract_parameters
          struct /* num_abstract_parameters */ {
            // Source: drake/systems/framework/system_base.h
            const char* doc = R"""()""";
          } num_abstract_parameters;
          // Symbol: drake::systems::SystemBase::ContextSizes::num_abstract_states
          struct /* num_abstract_states */ {
            // Source: drake/systems/framework/system_base.h
            const char* doc = R"""()""";
          } num_abstract_states;
          // Symbol: drake::systems::SystemBase::ContextSizes::num_discrete_state_groups
          struct /* num_discrete_state_groups */ {
            // Source: drake/systems/framework/system_base.h
            const char* doc = R"""()""";
          } num_discrete_state_groups;
          // Symbol: drake::systems::SystemBase::ContextSizes::num_generalized_positions
          struct /* num_generalized_positions */ {
            // Source: drake/systems/framework/system_base.h
            const char* doc = R"""()""";
          } num_generalized_positions;
          // Symbol: drake::systems::SystemBase::ContextSizes::num_generalized_velocities
          struct /* num_generalized_velocities */ {
            // Source: drake/systems/framework/system_base.h
            const char* doc = R"""()""";
          } num_generalized_velocities;
          // Symbol: drake::systems::SystemBase::ContextSizes::num_misc_continuous_states
          struct /* num_misc_continuous_states */ {
            // Source: drake/systems/framework/system_base.h
            const char* doc = R"""()""";
          } num_misc_continuous_states;
          // Symbol: drake::systems::SystemBase::ContextSizes::num_numeric_parameter_groups
          struct /* num_numeric_parameter_groups */ {
            // Source: drake/systems/framework/system_base.h
            const char* doc = R"""()""";
          } num_numeric_parameter_groups;
          // Symbol: drake::systems::SystemBase::ContextSizes::operator+=
          struct /* operator_iadd */ {
            // Source: drake/systems/framework/system_base.h
            const char* doc = R"""()""";
          } operator_iadd;
        } ContextSizes;
        // Symbol: drake::systems::SystemBase::DeclareCacheEntry
        struct /* DeclareCacheEntry */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc_3args_description_value_producer_prerequisites_of_calc =
R"""(Declares a new CacheEntry in this System using the most generic form
of the calculation function. Prefer one of the more convenient
signatures below if you can. The new cache entry is assigned a unique
CacheIndex and DependencyTicket, which can be obtained from the
returned CacheEntry.

Parameter ``description``:
    A human-readable description of this cache entry, most useful for
    debugging and documentation. Not interpreted in any way by Drake;
    it is retained by the cache entry and used to generate the
    description for the corresponding CacheEntryValue in the Context.

Parameter ``value_producer``:
    Provides the computation that maps from a given Context to the
    current value that this cache entry should have, as well as a way
    to allocate storage prior to the computation.

Parameter ``prerequisites_of_calc``:
    Provides the DependencyTicket list containing a ticket for *every*
    Context value on which ``calc_function`` may depend when it
    computes its result. Defaults to ``{all_sources_ticket()}`` if
    unspecified. If the cache value is truly independent of the
    Context (rare!) say so explicitly by providing the list
    ``{nothing_ticket()}``; an explicitly empty list ``{}`` is
    forbidden.

Returns:
    a reference to the newly-created CacheEntry.

Raises:
    RuntimeError if given an explicitly empty prerequisite list.)""";
          // Source: drake/systems/framework/system_base.h
          const char* doc_4args_stdstring_constValueType_voidMySystemconstMyContextValueTypeconst_stdset =
R"""(Declares a cache entry by specifying a model value of concrete type
``ValueType`` and a calculator function that is a class member
function (method) with signature:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    void MySystem∷CalcCacheValue(const MyContext&, ValueType*) const;

.. raw:: html

    </details>

where ``MySystem`` is a class derived from ``SystemBase``, `MyContext`
is a class derived from ``ContextBase``, and ``ValueType`` is any
concrete type such that ``Value<ValueType>`` is permitted. (The method
names are arbitrary.) Template arguments will be deduced and do not
need to be specified. See the DeclareCacheEntry_primary "primary
DeclareCacheEntry() signature" above for more information about the
parameters and behavior.

See also:
    drake∷Value)""";
          // Source: drake/systems/framework/system_base.h
          const char* doc_3args_stdstring_voidMySystemconstMyContextValueTypeconst_stdset =
R"""(Declares a cache entry by specifying only a calculator function that
is a class member function (method) with signature:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    void MySystem∷CalcCacheValue(const MyContext&, ValueType*) const;

.. raw:: html

    </details>

where ``MySystem`` is a class derived from ``SystemBase`` and
``MyContext`` is a class derived from ``ContextBase``. `ValueType` is
a concrete type such that (a) ``Value<ValueType>`` is permitted, and
(b) ``ValueType`` is default constructible. That allows us to create a
model value using ``Value<ValueType>{}`` (value initialized so
numerical types will be zeroed in the model). (The method name is
arbitrary.) Template arguments will be deduced and do not need to be
specified. See the first DeclareCacheEntry() signature above for more
information about the parameters and behavior.

Note:
    The default constructor will be called once immediately to create
    a model value, and subsequent allocations will just copy the model
    value without invoking the constructor again. If you want the
    constructor invoked again at each allocation (not common), use one
    of the other signatures to explicitly provide a method for the
    allocator to call; that method can then invoke the ``ValueType``
    default constructor each time it is called.

See also:
    drake∷Value)""";
        } DeclareCacheEntry;
        // Symbol: drake::systems::SystemBase::DeclareCacheEntryWithKnownTicket
        struct /* DeclareCacheEntryWithKnownTicket */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""((Internal use only) This is for cache entries associated with
pre-defined tickets, for example the cache entry for time derivatives.
See the public API for the most-general DeclareCacheEntry() signature
for the meanings of the other parameters here.)""";
        } DeclareCacheEntryWithKnownTicket;
        // Symbol: drake::systems::SystemBase::DoAllocateContext
        struct /* DoAllocateContext */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Derived class implementations should allocate a suitable concrete
Context type, then invoke the above InitializeContextBase() method. A
Diagram must then invoke AllocateContext() to obtain each of the
subcontexts for its DiagramContext, and must set up inter-subcontext
dependencies among its children and between itself and its children.
Then context resources such as parameters and state should be
allocated.)""";
        } DoAllocateContext;
        // Symbol: drake::systems::SystemBase::DoGetGraphvizFragment
        struct /* DoGetGraphvizFragment */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(The NVI implementation of GetGraphvizFragment() for subclasses to
override if desired. The default behavior should be sufficient in most
cases.)""";
        } DoGetGraphvizFragment;
        // Symbol: drake::systems::SystemBase::EvalAbstractInput
        struct /* EvalAbstractInput */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns the value of the input port with the given ``port_index`` as
an AbstractValue, which is permitted for ports of any type. Causes the
value to become up to date first if necessary, delegating to our
parent Diagram. Returns a pointer to the port's value, or nullptr if
the port is not connected. If you know the actual type, use one of the
more-specific signatures.

Precondition:
    ``port_index`` selects an existing input port of this System.

See also:
    InputPort∷Eval() (preferred))""";
        } EvalAbstractInput;
        // Symbol: drake::systems::SystemBase::EvalAbstractInputImpl
        struct /* EvalAbstractInputImpl */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""((Internal use only) Shared code for updating an input port and
returning a pointer to its abstract value, or nullptr if the port is
not connected. ``func`` should be the user-visible API function name
obtained with **func**.)""";
        } EvalAbstractInputImpl;
        // Symbol: drake::systems::SystemBase::EvalInputValue
        struct /* EvalInputValue */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns the value of an abstract-valued input port with the given
``port_index`` as a value of known type ``V``. Causes the value to
become up to date first if necessary. See EvalAbstractInput() for more
information.

The result is returned as a pointer to the input port's value of type
``V``, or nullptr if the port is not connected.

Precondition:
    ``port_index`` selects an existing input port of this System.

Precondition:
    the port's value must be retrievable from the stored abstract
    value using ``AbstractValue∷get_value<V>``.

See also:
    InputPort∷Eval() (preferred)

Template parameter ``V``:
    The type of data expected.)""";
        } EvalInputValue;
        // Symbol: drake::systems::SystemBase::GetDirectFeedthroughs
        struct /* GetDirectFeedthroughs */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Reports all direct feedthroughs from input ports to output ports. For
a system with m input ports: ``I = i₀, i₁, ..., iₘ₋₁``, and n output
ports, ``O = o₀, o₁, ..., oₙ₋₁``, the return map will contain pairs
(u, v) such that

- 0 ≤ u < m,
- 0 ≤ v < n,
- and there *might* be a direct feedthrough from input iᵤ to each output oᵥ.

See DeclareLeafOutputPort_feedthrough "DeclareLeafOutputPort"
documentation for how leaf systems can report their feedthrough.)""";
        } GetDirectFeedthroughs;
        // Symbol: drake::systems::SystemBase::GetGraphvizFragment
        struct /* GetGraphvizFragment */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""((Advanced) Like GetGraphvizString() but does not wrap the string in a
``digraph { … }``. This is useful when merging the fragment into
another graph, and is how Diagram obtains the Graphviz content for its
subsystems. The parameters are identical to GetGraphvizString(). The
return value contains additional metadata beyond the Graphviz content,
to better support merging.)""";
        } GetGraphvizFragment;
        // Symbol: drake::systems::SystemBase::GetGraphvizString
        struct /* GetGraphvizString */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns a Graphviz string describing this System. To render the
string, use the `Graphviz <http://www.graphviz.org/>`_ tool, ``dot``.

Notes about the display conventions:

- The nodes of the graph are systems, and the solid edges are connections
between system input and output ports.

- The class name of a System is shown in **Bold** atop the node.

- Under the class name, if a System has been given a name via set_name(),
it will be displayed as ``name=...``.

- Systems can elect to display additional properties besides their name;
see GraphvizFragmentParams∷header_lines for implementation details.

- A Diagram's input ports are shown with a
<span style="border:2px solid blue;border-radius:4px">blue border</span>
and output ports are shown with a
<span style="border:2px solid green;border-radius:4px">green border</span>.

- Zero-sized ports are <span style="color:grey">greyed out</span>.

- Deprecated ports are <strike>struck through</strike> and flagged with a
headstone emoji (🪦) after their name.

Parameter ``max_depth``:
    Sets a limit to the depth of nested diagrams to visualize. Use
    zero to render a diagram as a single system block.

Parameter ``options``:
    Arbitrary strings to request alterations to the output. Options
    that are unknown will be silently skipped. These options are often
    bespoke flags that are only understood by particular systems, but
    Drake has one built-in flag that is generally applicable:
    ``"split"``. When set to ``"I/O"``, the system will be added as
    two nodes with all inputs on one node and all outputs on the
    other; this is useful for systems that might otherwise cause
    problematic visual cycles.

Options are applied only to this immediate system; they are not
inherited by the subsystems of a Diagram. To specify an option for a
Diagram's subsystem, prefix the option name with the subsystem's path,
e.g., use ``"plant/split"="I/O"`` to set the ``"split"`` option on the
subsystem named ``"plant"``.)""";
        } GetGraphvizString;
        // Symbol: drake::systems::SystemBase::GetInputPortBaseOrThrow
        struct /* GetInputPortBaseOrThrow */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""((Internal use only) Returns the InputPortBase at index ``port_index``,
throwing RuntimeError we don't like the port index. The name of the
public API method that received the bad index is provided in ``func``
and is included in the error message. The ``warn_deprecated`` governs
whether or not a deprecation warning should occur when the
``port_index`` is deprecated; calls made on behalf of the user should
pass ``True``; calls made on behalf or the framework internals should
pass ``False``.)""";
        } GetInputPortBaseOrThrow;
        // Symbol: drake::systems::SystemBase::GetMemoryObjectName
        struct /* GetMemoryObjectName */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns a name for this System based on a stringification of its type
name and memory address. This is intended for use in diagnostic output
and should not be used for behavioral logic, because the
stringification of the type name may produce differing results across
platforms and because the address can vary from run to run.)""";
        } GetMemoryObjectName;
        // Symbol: drake::systems::SystemBase::GetOutputPortBaseOrThrow
        struct /* GetOutputPortBaseOrThrow */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""((Internal use only) Returns the OutputPortBase at index
``port_index``, throwing RuntimeError if we don't like the port index.
The name of the public API method that received the bad index is
provided in ``func`` and is included in the error message. The
``warn_deprecated`` governs whether or not a deprecation warning
should occur when the ``port_index`` is deprecated; calls made on
behalf of the user should pass ``True``; calls made on behalf or the
framework internals should pass ``False``.)""";
        } GetOutputPortBaseOrThrow;
        // Symbol: drake::systems::SystemBase::GetSystemName
        struct /* GetSystemName */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns a human-readable name for this system, for use in messages and
logging. This will be the same as returned by get_name(), unless that
would be an empty string. In that case we return a non-unique
placeholder name, currently just "_" (a lone underscore).)""";
        } GetSystemName;
        // Symbol: drake::systems::SystemBase::GetSystemPathname
        struct /* GetSystemPathname */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Generates and returns a human-readable full path name of this
subsystem, for use in messages and logging. The name starts from the
root System, with "∷" delimiters between parent and child subsystems,
with the individual subsystems represented by their names as returned
by GetSystemName().)""";
        } GetSystemPathname;
        // Symbol: drake::systems::SystemBase::GetSystemType
        struct /* GetSystemType */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns the most-derived type of this concrete System object as a
human-readable string suitable for use in error messages. The format
is as generated by NiceTypeName and will include namespace
qualification if present.

See also:
    NiceTypeName for more specifics.)""";
        } GetSystemType;
        // Symbol: drake::systems::SystemBase::GetUnsupportedScalarConversionMessage
        struct /* GetUnsupportedScalarConversionMessage */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""((Internal use only) Returns the message to use for a RuntimeError in
the case of unsupported scalar type conversions.)""";
        } GetUnsupportedScalarConversionMessage;
        // Symbol: drake::systems::SystemBase::GraphvizFragment
        struct /* GraphvizFragment */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""((Advanced) The return type of GetGraphvizFragment().)""";
          // Symbol: drake::systems::SystemBase::GraphvizFragment::fragments
          struct /* fragments */ {
            // Source: drake/systems/framework/system_base.h
            const char* doc =
R"""(The Graphviz content for this System. The fragments must be a valid
Graphviz figure when concatenated and then wrapped in a ``digraph { …
}`` or ``subgraph { … }`` stanza. During concatenation, no extra
newlines or any other kind of whitespace should be inserted. (This is
a list of strings, rather than a single string, to avoid redundant
string concatenation until the last moment when we return the final
GetGraphvizString().))""";
          } fragments;
          // Symbol: drake::systems::SystemBase::GraphvizFragment::input_ports
          struct /* input_ports */ {
            // Source: drake/systems/framework/system_base.h
            const char* doc =
R"""(The Graphviz IDs for this System's input ports, to be used for adding
Graphviz edges. The i'th element is the ID for ``get_input_port(i)``.
For a typical LeafSystem these will look like "s123:u0", "s123:u1", …
but for diagrams and other special cases they might vary.)""";
          } input_ports;
          // Symbol: drake::systems::SystemBase::GraphvizFragment::output_ports
          struct /* output_ports */ {
            // Source: drake/systems/framework/system_base.h
            const char* doc =
R"""(The Graphviz IDs for this System's output ports, to be used for adding
Graphviz edges. The i'th element is the ID for ``get_output_port(i)``.
For a typical LeafSystem these will look like "s123:y0", "s123:y1", …
but for diagrams and other special cases they might vary.)""";
          } output_ports;
        } GraphvizFragment;
        // Symbol: drake::systems::SystemBase::GraphvizFragmentParams
        struct /* GraphvizFragmentParams */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""((Advanced) The arguments to the protected method
DoGetGraphvizFragment(). This struct typically is only used by
subclasses of LeafSystem that need to customize their Graphviz
representation. These parameters constitute a polite request; a user's
System's implementation of DoGetGraphvizFragment() is not strictly
required to honor any of these parameters, but generally should
attempt to honor as many as possible.)""";
          // Symbol: drake::systems::SystemBase::GraphvizFragmentParams::header_lines
          struct /* header_lines */ {
            // Source: drake/systems/framework/system_base.h
            const char* doc =
R"""(The header line(s) to use for this Graphviz node's table. The strings
in ``header_lines`` should not contain newlines; those are added
automatically, along with ``

`` breaks between lines.)""";
          } header_lines;
          // Symbol: drake::systems::SystemBase::GraphvizFragmentParams::max_depth
          struct /* max_depth */ {
            // Source: drake/systems/framework/system_base.h
            const char* doc = R"""(As per GetGraphvizString().)""";
          } max_depth;
          // Symbol: drake::systems::SystemBase::GraphvizFragmentParams::node_id
          struct /* node_id */ {
            // Source: drake/systems/framework/system_base.h
            const char* doc = R"""(The Graphviz ID to use for this node.)""";
          } node_id;
          // Symbol: drake::systems::SystemBase::GraphvizFragmentParams::options
          struct /* options */ {
            // Source: drake/systems/framework/system_base.h
            const char* doc = R"""(As per GetGraphvizString().)""";
          } options;
        } GraphvizFragmentParams;
        // Symbol: drake::systems::SystemBase::InitializeContextBase
        struct /* InitializeContextBase */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(This method must be invoked from within derived class
DoAllocateContext() implementations right after the concrete Context
object has been allocated. It allocates cache entries, sets up all
intra-Context dependencies, and marks the ContextBase as initialized
so that we can verify proper derived-class behavior.

Precondition:
    The supplied context must not be null and must not already have
    been initialized.)""";
        } InitializeContextBase;
        // Symbol: drake::systems::SystemBase::IsObviouslyNotInputDependent
        struct /* IsObviouslyNotInputDependent */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""((Internal use only) Checks if a ticket depends on (any) input port.
When this returns "true" the ticket MUST NOT depend on input. When
this returns "false", it just means that we're not sure. This is
intended to be an inexpensive check, withough searching the entire
graph.)""";
        } IsObviouslyNotInputDependent;
        // Symbol: drake::systems::SystemBase::MakeFixInputPortTypeChecker
        struct /* MakeFixInputPortTypeChecker */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""((Internal use only) Given a ``port_index``, returns a function to be
called when validating Context∷FixInputPort requests. The function
should attempt to throw an exception if the input AbstractValue is
invalid, so that errors can be reported at Fix-time instead of
EvalInput-time.)""";
        } MakeFixInputPortTypeChecker;
        // Symbol: drake::systems::SystemBase::NextInputPortName
        struct /* NextInputPortName */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""((Internal use only) Returns a name for the next input port, using the
given name if it isn't kUseDefaultName, otherwise making up a name
like "u3" from the next available input port index.

Precondition:
    ``given_name`` must not be empty.)""";
        } NextInputPortName;
        // Symbol: drake::systems::SystemBase::NextOutputPortName
        struct /* NextOutputPortName */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""((Internal use only) Returns a name for the next output port, using the
given name if it isn't kUseDefaultName, otherwise making up a name
like "y3" from the next available output port index.

Precondition:
    ``given_name`` must not be empty.)""";
        } NextOutputPortName;
        // Symbol: drake::systems::SystemBase::SystemBase
        struct /* ctor */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc = R"""((Internal use only).)""";
        } ctor;
        // Symbol: drake::systems::SystemBase::ThrowCantEvaluateInputPort
        struct /* ThrowCantEvaluateInputPort */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Throws RuntimeError because someone called API method ``func``, that
requires this input port to be evaluatable, but the port was neither
fixed nor connected.)""";
        } ThrowCantEvaluateInputPort;
        // Symbol: drake::systems::SystemBase::ThrowInputPortHasWrongType
        struct /* ThrowInputPortHasWrongType */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Throws RuntimeError because someone called API method ``func``
claiming the input port had some value type that was wrong.)""";
        } ThrowInputPortHasWrongType;
        // Symbol: drake::systems::SystemBase::ThrowInputPortIndexOutOfRange
        struct /* ThrowInputPortIndexOutOfRange */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Throws RuntimeError to report bad input ``port_index`` that was passed
to API method ``func``.)""";
        } ThrowInputPortIndexOutOfRange;
        // Symbol: drake::systems::SystemBase::ThrowNegativePortIndex
        struct /* ThrowNegativePortIndex */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc = R"""()""";
        } ThrowNegativePortIndex;
        // Symbol: drake::systems::SystemBase::ThrowNotAVectorInputPort
        struct /* ThrowNotAVectorInputPort */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Throws RuntimeError because someone misused API method ``func``, that
is only allowed for declared-vector input ports, on an abstract port
whose index is given here.)""";
        } ThrowNotAVectorInputPort;
        // Symbol: drake::systems::SystemBase::ThrowOutputPortIndexOutOfRange
        struct /* ThrowOutputPortIndexOutOfRange */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Throws RuntimeError to report bad output ``port_index`` that was
passed to API method ``func``.)""";
        } ThrowOutputPortIndexOutOfRange;
        // Symbol: drake::systems::SystemBase::ThrowValidateContextMismatch
        struct /* ThrowValidateContextMismatch */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""((Internal use only) Throws RuntimeError with a message that the sanity
check(s) given by ValidateContext have failed.)""";
        } ThrowValidateContextMismatch;
        // Symbol: drake::systems::SystemBase::ValidateContext
        struct /* ValidateContext */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Checks whether the given context was created for this system.

Note:
    This method is sufficiently fast for performance sensitive code.

Raises:
    RuntimeError if the System Ids don't match.)""";
        } ValidateContext;
        // Symbol: drake::systems::SystemBase::ValidateCreatedForThisSystem
        struct /* ValidateCreatedForThisSystem */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Checks whether the given object was created for this system.

Note:
    This method is sufficiently fast for performance sensitive code.

Raises:
    RuntimeError if the System Ids don't match or if ``object``
    doesn't have an associated System Id.

Raises:
    RuntimeError if the argument type is a pointer and it is null.)""";
        } ValidateCreatedForThisSystem;
        // Symbol: drake::systems::SystemBase::abstract_parameter_ticket
        struct /* abstract_parameter_ticket */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns a ticket indicating dependence on a particular abstract
parameter paᵢ.

See also:
    pa_ticket() to obtain a ticket for *all* abstract parameters.)""";
        } abstract_parameter_ticket;
        // Symbol: drake::systems::SystemBase::abstract_state_ticket
        struct /* abstract_state_ticket */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns a ticket indicating dependence on a particular abstract state
variable xaᵢ.

See also:
    xa_ticket() to obtain a ticket for *all* abstract variables.)""";
        } abstract_state_ticket;
        // Symbol: drake::systems::SystemBase::accuracy_ticket
        struct /* accuracy_ticket */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns a ticket indicating dependence on the accuracy setting in the
Context. This is the same ticket for all systems and refers to the
same accuracy value.)""";
        } accuracy_ticket;
        // Symbol: drake::systems::SystemBase::all_input_ports_ticket
        struct /* all_input_ports_ticket */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns a ticket indicating dependence on *all* input ports u of this
system.

See also:
    input_port_ticket() to obtain a ticket for just one input port.)""";
        } all_input_ports_ticket;
        // Symbol: drake::systems::SystemBase::all_parameters_ticket
        struct /* all_parameters_ticket */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns a ticket indicating dependence on *all* parameters p in this
system, including numeric parameters pn, and abstract parameters pa.)""";
        } all_parameters_ticket;
        // Symbol: drake::systems::SystemBase::all_sources_except_input_ports_ticket
        struct /* all_sources_except_input_ports_ticket */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns a ticket indicating dependence on every possible independent
source value *except* input ports. This can be helpful in avoiding the
incorrect appearance of algebraic loops in a Diagram (those always
involve apparent input port dependencies). For an output port, use
this ticket plus tickets for just the input ports on which the output
computation *actually* depends. The sources included in this ticket
are: time, accuracy, state, and parameters. Note that dependencies on
cache entries are *not* included here. Usually that won't matter since
cache entries typically depend on at least one of time, accuracy,
state, or parameters so will be invalidated for the same reason the
current computation is. However, for a computation that depends on a
cache entry that depends only on input ports, be sure that you have
included those input ports in the dependency list, or include a direct
dependency on the cache entry.

See also:
    input_port_ticket() to obtain a ticket for an input port.

See also:
    cache_entry_ticket() to obtain a ticket for a cache entry.

See also:
    all_sources_ticket() to also include all input ports as
    dependencies.)""";
        } all_sources_except_input_ports_ticket;
        // Symbol: drake::systems::SystemBase::all_sources_ticket
        struct /* all_sources_ticket */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns a ticket indicating dependence on every possible independent
source value, including time, accuracy, state, input ports, and
parameters (but not cache entries). This is the default dependency for
computations that have not specified anything more refined. It is
equivalent to the set ``{all_sources_except_input_ports_ticket(),
all_input_ports_ticket()}``.

See also:
    cache_entry_ticket() to obtain a ticket for a cache entry.)""";
        } all_sources_ticket;
        // Symbol: drake::systems::SystemBase::all_state_ticket
        struct /* all_state_ticket */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns a ticket indicating dependence on *all* state variables x in
this system, including continuous variables xc, discrete (numeric)
variables xd, and abstract state variables xa. This does not imply
dependence on time, accuracy, parameters, or inputs; those must be
specified separately. If you mean to express dependence on all
possible value sources, use all_sources_ticket() instead.)""";
        } all_state_ticket;
        // Symbol: drake::systems::SystemBase::assign_next_dependency_ticket
        struct /* assign_next_dependency_ticket */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""((Internal use only) Assigns the next unused dependency ticket number,
unique only within a particular system. Each call to this method
increments the ticket number.)""";
        } assign_next_dependency_ticket;
        // Symbol: drake::systems::SystemBase::cache_entry_ticket
        struct /* cache_entry_ticket */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns a ticket indicating dependence on the cache entry indicated by
``index``. Note that cache entries are *not* included in the
``all_sources`` ticket so must be listed separately.

Precondition:
    ``index`` selects an existing cache entry in this System.)""";
        } cache_entry_ticket;
        // Symbol: drake::systems::SystemBase::configuration_ticket
        struct /* configuration_ticket */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc = R"""()""";
        } configuration_ticket;
        // Symbol: drake::systems::SystemBase::discrete_state_ticket
        struct /* discrete_state_ticket */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns a ticket indicating dependence on a particular discrete state
variable xdᵢ (may be a vector). (We sometimes refer to this as a
"discrete variable group".)

See also:
    xd_ticket() to obtain a ticket for *all* discrete variables.)""";
        } discrete_state_ticket;
        // Symbol: drake::systems::SystemBase::get_cache_entry
        struct /* get_cache_entry */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns a reference to a CacheEntry given its ``index``.)""";
        } get_cache_entry;
        // Symbol: drake::systems::SystemBase::get_context_sizes
        struct /* get_context_sizes */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc_0args =
R"""(Obtains access to the declared Context partition sizes as accumulated
during LeafSystem or Diagram construction .)""";
          // Source: drake/systems/framework/system_base.h
          const char* doc_1args =
R"""(Allows Diagram to access protected get_context_sizes() recursively on
its subsystems.)""";
        } get_context_sizes;
        // Symbol: drake::systems::SystemBase::get_input_port_base
        struct /* get_input_port_base */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns a reference to an InputPort given its ``port_index``.

Precondition:
    ``port_index`` selects an existing input port of this System.)""";
        } get_input_port_base;
        // Symbol: drake::systems::SystemBase::get_mutable_cache_entry
        struct /* get_mutable_cache_entry */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""((Advanced) Returns a mutable reference to a CacheEntry given its
``index``. Note that you do not need mutable access to a CacheEntry to
modify its value in a Context, so most users should not use this
method.)""";
        } get_mutable_cache_entry;
        // Symbol: drake::systems::SystemBase::get_mutable_context_sizes
        struct /* get_mutable_context_sizes */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Obtains mutable access to the Context sizes struct. Should be used
only during LeafSystem or Diagram construction.)""";
        } get_mutable_context_sizes;
        // Symbol: drake::systems::SystemBase::get_name
        struct /* get_name */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns the name last supplied to set_name(), if any. Diagrams built
with DiagramBuilder will always have a default name for every
contained subsystem for which no user-provided name is available.
Systems created by copying with a scalar type change have the same
name as the source system. An empty string is returned if no name has
been set.)""";
        } get_name;
        // Symbol: drake::systems::SystemBase::get_output_port_base
        struct /* get_output_port_base */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns a reference to an OutputPort given its ``port_index``.

Precondition:
    ``port_index`` selects an existing output port of this System.)""";
        } get_output_port_base;
        // Symbol: drake::systems::SystemBase::get_parent_service
        struct /* get_parent_service */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns a pointer to the service interface of the immediately
enclosing Diagram if one has been set, otherwise nullptr.)""";
        } get_parent_service;
        // Symbol: drake::systems::SystemBase::get_system_id
        struct /* get_system_id */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""((Internal) Gets the id used to tag context data as being created by
this system. See system_compatibility.)""";
        } get_system_id;
        // Symbol: drake::systems::SystemBase::implicit_time_derivatives_residual_size
        struct /* implicit_time_derivatives_residual_size */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns the size of the implicit time derivatives residual vector. By
default this is the same as num_continuous_states() but a LeafSystem
can change it during construction via
LeafSystem∷DeclareImplicitTimeDerivativesResidualSize().)""";
        } implicit_time_derivatives_residual_size;
        // Symbol: drake::systems::SystemBase::input_port_ticket
        struct /* input_port_ticket */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns a ticket indicating dependence on input port uᵢ indicated by
``index``.

Precondition:
    ``index`` selects an existing input port of this System.)""";
        } input_port_ticket;
        // Symbol: drake::systems::SystemBase::ke_ticket
        struct /* ke_ticket */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns a ticket for the cache entry that holds the kinetic energy
calculation.

See also:
    System∷EvalKineticEnergy())""";
        } ke_ticket;
        // Symbol: drake::systems::SystemBase::kinematics_ticket
        struct /* kinematics_ticket */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc = R"""()""";
        } kinematics_ticket;
        // Symbol: drake::systems::SystemBase::nothing_ticket
        struct /* nothing_ticket */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns a ticket indicating that a computation does not depend on
*any* source value; that is, it is a constant. If this appears in a
prerequisite list, it must be the only entry.)""";
        } nothing_ticket;
        // Symbol: drake::systems::SystemBase::num_abstract_parameters
        struct /* num_abstract_parameters */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns the number of declared abstract parameters.)""";
        } num_abstract_parameters;
        // Symbol: drake::systems::SystemBase::num_abstract_states
        struct /* num_abstract_states */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns the number of declared abstract state variables.)""";
        } num_abstract_states;
        // Symbol: drake::systems::SystemBase::num_cache_entries
        struct /* num_cache_entries */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns the number nc of cache entries currently allocated in this
System. These are indexed from 0 to nc-1.)""";
        } num_cache_entries;
        // Symbol: drake::systems::SystemBase::num_continuous_states
        struct /* num_continuous_states */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns the number of declared continuous state variables.)""";
        } num_continuous_states;
        // Symbol: drake::systems::SystemBase::num_discrete_state_groups
        struct /* num_discrete_state_groups */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns the number of declared discrete state groups (each group is a
vector-valued discrete state variable).)""";
        } num_discrete_state_groups;
        // Symbol: drake::systems::SystemBase::num_input_ports
        struct /* num_input_ports */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns the number of input ports currently allocated in this System.
These are indexed from 0 to num_input_ports()-1.)""";
        } num_input_ports;
        // Symbol: drake::systems::SystemBase::num_numeric_parameter_groups
        struct /* num_numeric_parameter_groups */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns the number of declared numeric parameters (each of these is a
vector-valued parameter).)""";
        } num_numeric_parameter_groups;
        // Symbol: drake::systems::SystemBase::num_output_ports
        struct /* num_output_ports */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns the number of output ports currently allocated in this System.
These are indexed from 0 to num_output_ports()-1.)""";
        } num_output_ports;
        // Symbol: drake::systems::SystemBase::num_total_inputs
        struct /* num_total_inputs */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns the total dimension of all of the vector-valued input ports
(as if they were muxed).)""";
        } num_total_inputs;
        // Symbol: drake::systems::SystemBase::num_total_outputs
        struct /* num_total_outputs */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns the total dimension of all of the vector-valued output ports
(as if they were muxed).)""";
        } num_total_outputs;
        // Symbol: drake::systems::SystemBase::numeric_parameter_ticket
        struct /* numeric_parameter_ticket */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns a ticket indicating dependence on a particular numeric
parameter pnᵢ (may be a vector).

See also:
    pn_ticket() to obtain a ticket for *all* numeric parameters.)""";
        } numeric_parameter_ticket;
        // Symbol: drake::systems::SystemBase::output_port_ticket
        struct /* output_port_ticket */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""((Internal use only) Returns a ticket indicating dependence on the
output port indicated by ``index``. No user-definable quantities in a
system can meaningfully depend on that system's own output ports.

Precondition:
    ``index`` selects an existing output port of this System.)""";
        } output_port_ticket;
        // Symbol: drake::systems::SystemBase::pa_ticket
        struct /* pa_ticket */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns a ticket indicating dependence on all of the abstract
parameters pa in the current Context.

See also:
    abstract_parameter_ticket() to obtain a ticket for just one
    abstract parameter.)""";
        } pa_ticket;
        // Symbol: drake::systems::SystemBase::pc_ticket
        struct /* pc_ticket */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns a ticket for the cache entry that holds the conservative power
calculation.

See also:
    System∷EvalConservativePower())""";
        } pc_ticket;
        // Symbol: drake::systems::SystemBase::pe_ticket
        struct /* pe_ticket */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns a ticket for the cache entry that holds the potential energy
calculation.

See also:
    System∷EvalPotentialEnergy())""";
        } pe_ticket;
        // Symbol: drake::systems::SystemBase::pn_ticket
        struct /* pn_ticket */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns a ticket indicating dependence on all of the numerical
parameters in the current Context.

See also:
    numeric_parameter_ticket() to obtain a ticket for just one numeric
    parameter.)""";
        } pn_ticket;
        // Symbol: drake::systems::SystemBase::pnc_ticket
        struct /* pnc_ticket */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns a ticket for the cache entry that holds the non-conservative
power calculation.

See also:
    System∷EvalNonConservativePower())""";
        } pnc_ticket;
        // Symbol: drake::systems::SystemBase::q_ticket
        struct /* q_ticket */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns a ticket indicating that a computation depends on
configuration state variables q. There is no ticket representing just
one of the state variables qᵢ.)""";
        } q_ticket;
        // Symbol: drake::systems::SystemBase::set_implicit_time_derivatives_residual_size
        struct /* set_implicit_time_derivatives_residual_size */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Allows a LeafSystem to override the default size for the implicit time
derivatives residual and a Diagram to sum up the total size. If no
value is set, the default size is n=num_continuous_states().

Parameter ``n``:
    The size of the residual vector output argument of
    System∷CalcImplicitTimeDerivativesResidual(). If n <= 0 restore to
    the default, num_continuous_states().

See also:
    implicit_time_derivatives_residual_size()

See also:
    LeafSystem∷DeclareImplicitTimeDerivativesResidualSize()

See also:
    System∷CalcImplicitTimeDerivativesResidual())""";
        } set_implicit_time_derivatives_residual_size;
        // Symbol: drake::systems::SystemBase::set_name
        struct /* set_name */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Sets the name of the system. Do not use the path delimiter character
':' in the name. When creating a Diagram, names of sibling subsystems
should be unique. DiagramBuilder uses this method to assign a unique
default name if none is provided.)""";
        } set_name;
        // Symbol: drake::systems::SystemBase::set_parent_service
        struct /* set_parent_service */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""((Internal use only) Declares that ``parent_service`` is the service
interface of the Diagram that owns this subsystem. Throws if the
parent service has already been set and ``parent_service`` is
non-null.)""";
        } set_parent_service;
        // Symbol: drake::systems::SystemBase::time_ticket
        struct /* time_ticket */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns a ticket indicating dependence on time. This is the same
ticket for all systems and refers to the same time value.)""";
        } time_ticket;
        // Symbol: drake::systems::SystemBase::v_ticket
        struct /* v_ticket */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns a ticket indicating dependence on velocity state variables v.
This does *not* also indicate a dependence on configuration variables
q -- you must list that explicitly or use kinematics_ticket() instead.
There is no ticket representing just one of the state variables vᵢ.)""";
        } v_ticket;
        // Symbol: drake::systems::SystemBase::xa_ticket
        struct /* xa_ticket */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns a ticket indicating dependence on all of the abstract state
variables in the current Context.

See also:
    abstract_state_ticket() to obtain a ticket for just one abstract
    state variable.)""";
        } xa_ticket;
        // Symbol: drake::systems::SystemBase::xc_ticket
        struct /* xc_ticket */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns a ticket indicating dependence on *all* of the continuous
state variables q, v, or z.)""";
        } xc_ticket;
        // Symbol: drake::systems::SystemBase::xcdot_ticket
        struct /* xcdot_ticket */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns a ticket for the cache entry that holds time derivatives of
the continuous variables.

See also:
    EvalTimeDerivatives())""";
        } xcdot_ticket;
        // Symbol: drake::systems::SystemBase::xd_ticket
        struct /* xd_ticket */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns a ticket indicating dependence on all of the numerical
discrete state variables, in any discrete variable group.

See also:
    discrete_state_ticket() to obtain a ticket for just one discrete
    state variable.)""";
        } xd_ticket;
        // Symbol: drake::systems::SystemBase::xd_unique_periodic_update_ticket
        struct /* xd_unique_periodic_update_ticket */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""((Internal use only) Returns a ticket for the cache entry that holds
the unique periodic discrete update computation.

See also:
    System∷EvalUniquePeriodicDiscreteUpdate())""";
        } xd_unique_periodic_update_ticket;
        // Symbol: drake::systems::SystemBase::z_ticket
        struct /* z_ticket */ {
          // Source: drake/systems/framework/system_base.h
          const char* doc =
R"""(Returns a ticket indicating dependence on any or all of the
miscellaneous continuous state variables z. There is no ticket
representing just one of the state variables zᵢ.)""";
        } z_ticket;
      } SystemBase;
      // Symbol: drake::systems::SystemConstraint
      struct /* SystemConstraint */ {
        // Source: drake/systems/framework/system_constraint.h
        const char* doc =
R"""(A SystemConstraint is a generic base-class for constraints on Systems.

A SystemConstraint is a means to inform our algorithms *about* the
implemented system behavior -- declaring the constraint does not
*cause* the system behavior to change. It is meant to improve analysis
by telling our algorithms that "all valid solutions of this dynamical
system will satisfy the following (in)equalities". Examples could
include conserved quantities or joint limits on a mechanism.

This class is intentionally similar to, but (so far) independent from
solvers∷Constraint. This is primarily because there is no notion of
decision variables in the system classes (yet); rather each individual
algorithm (e.g. trajectory optimization, or system identification)
constructs decision variables for the particular mathematical program
that is being formulated, and must bind the system constraint to those
variables (e.g. by populating the Context with the decision variables
and calling Calc).

We can convert a SystemConstraint to a solvers∷Constraint by using
SystemConstraintWrapper or SystemConstraintAdapter.

See also:
    LeafSystem<T>∷DeclareEqualityConstraint and
    LeafSystem<T>∷DeclareInequalityConstraint for use cases.)""";
        // Symbol: drake::systems::SystemConstraint::Calc
        struct /* Calc */ {
          // Source: drake/systems/framework/system_constraint.h
          const char* doc =
R"""(Evaluates the function pointer passed in through the constructor,
writing the output to ``value``. ``value`` will be
(non-conservatively) resized to match the constraint function output.)""";
        } Calc;
        // Symbol: drake::systems::SystemConstraint::CheckSatisfied
        struct /* CheckSatisfied */ {
          // Source: drake/systems/framework/system_constraint.h
          const char* doc =
R"""(Evaluates the function pointer, and check if all of the outputs are
within the desired bounds.)""";
        } CheckSatisfied;
        // Symbol: drake::systems::SystemConstraint::SystemConstraint<T>
        struct /* ctor */ {
          // Source: drake/systems/framework/system_constraint.h
          const char* doc_2args =
R"""((Advanced) Constructs a default (zero-sized) SystemConstraint.

Most users should call a LeafSystem method like
DeclareEqualityConstraint to create (and add) constraints, not call
this constructor directly.

Parameter ``description``:
    a human-readable description useful for debugging.)""";
          // Source: drake/systems/framework/system_constraint.h
          const char* doc_4args =
R"""((Advanced) Constructs a SystemConstraint. Depending on the ``bounds``
it could be an equality constraint f(x) = 0, or an inequality
constraint lower_bound <= f(x) <= upper_bound.

Most users should call a LeafSystem method like
DeclareEqualityConstraint to create (and add) constraints, not call
this constructor directly.

Parameter ``description``:
    a human-readable description useful for debugging.)""";
        } ctor;
        // Symbol: drake::systems::SystemConstraint::bounds
        struct /* bounds */ {
          // Source: drake/systems/framework/system_constraint.h
          const char* doc = R"""()""";
        } bounds;
        // Symbol: drake::systems::SystemConstraint::description
        struct /* description */ {
          // Source: drake/systems/framework/system_constraint.h
          const char* doc = R"""()""";
        } description;
        // Symbol: drake::systems::SystemConstraint::get_system
        struct /* get_system */ {
          // Source: drake/systems/framework/system_constraint.h
          const char* doc =
R"""(Returns a reference to the System that owns this constraint. Note that
for a constraint on a diagram this will be the diagram itself, never a
leaf system whose constraint was re-expressed.)""";
        } get_system;
        // Symbol: drake::systems::SystemConstraint::get_system_id
        struct /* get_system_id */ {
          // Source: drake/systems/framework/system_constraint.h
          const char* doc =
R"""((Internal use only) Gets the id of the subsystem associated with this
object, if one has been set.)""";
        } get_system_id;
        // Symbol: drake::systems::SystemConstraint::is_equality_constraint
        struct /* is_equality_constraint */ {
          // Source: drake/systems/framework/system_constraint.h
          const char* doc = R"""()""";
        } is_equality_constraint;
        // Symbol: drake::systems::SystemConstraint::lower_bound
        struct /* lower_bound */ {
          // Source: drake/systems/framework/system_constraint.h
          const char* doc = R"""()""";
        } lower_bound;
        // Symbol: drake::systems::SystemConstraint::set_system_id
        struct /* set_system_id */ {
          // Source: drake/systems/framework/system_constraint.h
          const char* doc =
R"""((Internal use only) Records the id of the subsystem associated with
this object.)""";
        } set_system_id;
        // Symbol: drake::systems::SystemConstraint::size
        struct /* size */ {
          // Source: drake/systems/framework/system_constraint.h
          const char* doc = R"""()""";
        } size;
        // Symbol: drake::systems::SystemConstraint::type
        struct /* type */ {
          // Source: drake/systems/framework/system_constraint.h
          const char* doc = R"""()""";
        } type;
        // Symbol: drake::systems::SystemConstraint::upper_bound
        struct /* upper_bound */ {
          // Source: drake/systems/framework/system_constraint.h
          const char* doc = R"""()""";
        } upper_bound;
      } SystemConstraint;
      // Symbol: drake::systems::SystemConstraintBounds
      struct /* SystemConstraintBounds */ {
        // Source: drake/systems/framework/system_constraint.h
        const char* doc =
R"""(The bounds of a SystemConstraint. This also encompasses the form of
the constraint: equality constraints occur when both the lower and
upper bounds are all zeros.)""";
        // Symbol: drake::systems::SystemConstraintBounds::Equality
        struct /* Equality */ {
          // Source: drake/systems/framework/system_constraint.h
          const char* doc =
R"""(Creates constraint of type SystemConstraintType∷kEquality, with the
given size for ``f(x)``.)""";
        } Equality;
        // Symbol: drake::systems::SystemConstraintBounds::SystemConstraintBounds
        struct /* ctor */ {
          // Source: drake/systems/framework/system_constraint.h
          const char* doc_0args =
R"""(Creates constraint bounds with zero size.)""";
          // Source: drake/systems/framework/system_constraint.h
          const char* doc_2args_lower_upper =
R"""(Creates a constraint with the given upper and lower bounds for
``f(x)``. The type() of this constraint will be kInequality, except in
the unusual case where both lower and upper are all zeros (in which
case it is kEquality). It is not currently allowed to set lower ==
upper (creating an equality constraint in the form f(x) = b), except
when b == 0. Using a non-zero b might be allowed in the future.)""";
          // Source: drake/systems/framework/system_constraint.h
          const char* doc_2args_lower_stdnulloptt =
R"""(Creates an inequality constraint with the given lower bounds for
``f(x)``. The upper bounds are all positive infinity.)""";
          // Source: drake/systems/framework/system_constraint.h
          const char* doc_2args_stdnulloptt_upper =
R"""(Creates an inequality constraint with the given upper bounds for
``f(x)``. The lower bounds are all negative infinity.)""";
        } ctor;
        // Symbol: drake::systems::SystemConstraintBounds::lower
        struct /* lower */ {
          // Source: drake/systems/framework/system_constraint.h
          const char* doc = R"""()""";
        } lower;
        // Symbol: drake::systems::SystemConstraintBounds::size
        struct /* size */ {
          // Source: drake/systems/framework/system_constraint.h
          const char* doc = R"""()""";
        } size;
        // Symbol: drake::systems::SystemConstraintBounds::type
        struct /* type */ {
          // Source: drake/systems/framework/system_constraint.h
          const char* doc = R"""()""";
        } type;
        // Symbol: drake::systems::SystemConstraintBounds::upper
        struct /* upper */ {
          // Source: drake/systems/framework/system_constraint.h
          const char* doc = R"""()""";
        } upper;
      } SystemConstraintBounds;
      // Symbol: drake::systems::SystemConstraintIndex
      struct /* SystemConstraintIndex */ {
        // Source: drake/systems/framework/framework_common.h
        const char* doc =
R"""(Serves as the local index for constraints declared on a given System.)""";
      } SystemConstraintIndex;
      // Symbol: drake::systems::SystemConstraintType
      struct /* SystemConstraintType */ {
        // Source: drake/systems/framework/system_constraint.h
        const char* doc = R"""(The form of a SystemConstraint.)""";
        // Symbol: drake::systems::SystemConstraintType::kEquality
        struct /* kEquality */ {
          // Source: drake/systems/framework/system_constraint.h
          const char* doc = R"""(The constraint is of the form f(x)=0.)""";
        } kEquality;
        // Symbol: drake::systems::SystemConstraintType::kInequality
        struct /* kInequality */ {
          // Source: drake/systems/framework/system_constraint.h
          const char* doc = R"""()""";
        } kInequality;
      } SystemConstraintType;
      // Symbol: drake::systems::SystemOutput
      struct /* SystemOutput */ {
        // Source: drake/systems/framework/system_output.h
        const char* doc =
R"""(Conveniently stores a snapshot of the values of every output port of a
System. There is framework support for allocating the right types and
filling them in but otherwise this is not used internally. Note that
there is never any live connection between a SystemOutput object and
the System whose output values it has captured.

A ``SystemOutput<T>`` object can only be obtained using
``System<T>∷AllocateOutput()`` or by copying an existing SystemOutput
object.)""";
        // Symbol: drake::systems::SystemOutput::GetMutableData
        struct /* GetMutableData */ {
          // Source: drake/systems/framework/system_output.h
          const char* doc =
R"""((Advanced) Returns mutable access to an AbstractValue object that is
suitable for holding the value of output port ``index`` of the
allocating System. This works for any output port regardless of it
actual type. Most users should just call ``System<T>∷CalcOutputs()``
to get all the output port values at once.)""";
        } GetMutableData;
        // Symbol: drake::systems::SystemOutput::GetMutableVectorData
        struct /* GetMutableVectorData */ {
          // Source: drake/systems/framework/system_output.h
          const char* doc =
R"""((Advanced) Returns mutable access to a ``BasicVector<T>`` object that
is suitable for holding the value of output port ``index`` of the
allocating System. The object's concrete type is preserved from the
output port. Most users should just call ``System<T>∷CalcOutputs()``
to get all the output port values at once.

Raises:
    RuntimeError if the port is not vector-valued.)""";
        } GetMutableVectorData;
        // Symbol: drake::systems::SystemOutput::SystemOutput<T>
        struct /* ctor */ {
          // Source: drake/systems/framework/system_output.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::systems::SystemOutput::get_data
        struct /* get_data */ {
          // Source: drake/systems/framework/system_output.h
          const char* doc =
R"""(Returns the last-saved value of output port ``index`` as an
AbstractValue. This works for any output port regardless of it actual
type.)""";
        } get_data;
        // Symbol: drake::systems::SystemOutput::get_system_id
        struct /* get_system_id */ {
          // Source: drake/systems/framework/system_output.h
          const char* doc =
R"""((Internal) Gets the id of the System that created this output. See
system_compatibility.)""";
        } get_system_id;
        // Symbol: drake::systems::SystemOutput::get_vector_data
        struct /* get_vector_data */ {
          // Source: drake/systems/framework/system_output.h
          const char* doc =
R"""(Returns the last-saved value of output port ``index`` as a
``BasicVector<T>``, although the actual concrete type is preserved
from the actual output port.

Raises:
    RuntimeError if the port is not vector-valued.)""";
        } get_vector_data;
        // Symbol: drake::systems::SystemOutput::num_ports
        struct /* num_ports */ {
          // Source: drake/systems/framework/system_output.h
          const char* doc =
R"""(Returns the number of output ports specified for this SystemOutput
during allocation.)""";
        } num_ports;
      } SystemOutput;
      // Symbol: drake::systems::SystemScalarConverter
      struct /* SystemScalarConverter */ {
        // Source: drake/systems/framework/system_scalar_converter.h
        const char* doc =
R"""(Helper class to convert a System<U> into a System<T>, intended for
internal use by the System framework, not directly by users.

For user-facing documentation see system_scalar_conversion.

Because it is not templated on a System subclass, this class can be
used by LeafSystem without any direct knowledge of the subtypes being
converted. In other words, it enables a runtime flavor of the CRTP.

Throughout this class, the following template naming convention
applies:

Template parameter ``S``:
    is the System subclass that this object will convert from and to.

Template parameter ``U``:
    the source scalar type (to convert from), which must be one of the
    default_scalars "default scalars".

Template parameter ``T``:
    the resulting scalar type (to convert into), which must be one of
    the default_scalars "default scalars".

Note:
    Conversions where ``T`` and ``U`` types are the same are not
    supported. Template functions such as IsConvertible<T, U>() are
    still callable, but will always return false, null, etc.)""";
        // Symbol: drake::systems::SystemScalarConverter::Convert
        struct /* Convert */ {
          // Source: drake/systems/framework/system_scalar_converter.h
          const char* doc =
R"""(Converts a System<U> into a System<T>. This is the API that LeafSystem
uses to provide a default implementation of DoToAutoDiffXd, etc.
Returns null when IsConvertible() is false.)""";
        } Convert;
        // Symbol: drake::systems::SystemScalarConverter::IsConvertible
        struct /* IsConvertible */ {
          // Source: drake/systems/framework/system_scalar_converter.h
          const char* doc =
R"""(Returns true iff this object can convert a System<U> into a System<T>,
i.e., whether Convert() will return non-null.)""";
        } IsConvertible;
        // Symbol: drake::systems::SystemScalarConverter::MakeWithoutSubtypeChecking
        struct /* MakeWithoutSubtypeChecking */ {
          // Source: drake/systems/framework/system_scalar_converter.h
          const char* doc =
R"""((Advanced) Creates a converter similar to the single-argument
constructor, with the built-in checks for guaranteed subtype
preservation of the System turned off. In general, subtype
preservation is an important invariant of scalar conversion, so be
extremely cautious about disabling it.)""";
        } MakeWithoutSubtypeChecking;
        // Symbol: drake::systems::SystemScalarConverter::Remove
        struct /* Remove */ {
          // Source: drake/systems/framework/system_scalar_converter.h
          const char* doc =
R"""(Removes from this converter the ability to convert from System<U> to
System<T>.)""";
        } Remove;
        // Symbol: drake::systems::SystemScalarConverter::RemoveUnlessAlsoSupportedBy
        struct /* RemoveUnlessAlsoSupportedBy */ {
          // Source: drake/systems/framework/system_scalar_converter.h
          const char* doc =
R"""(Removes from this converter all pairs where ``other.IsConvertible<T,
U>`` is false. The subtype ``S`` need not be the same between this and
``other``.)""";
        } RemoveUnlessAlsoSupportedBy;
        // Symbol: drake::systems::SystemScalarConverter::SystemScalarConverter
        struct /* ctor */ {
          // Source: drake/systems/framework/system_scalar_converter.h
          const char* doc_0args =
R"""((Advanced) Creates a converter that supports no conversions. The
single- argument constructor below is the overload intended for users.)""";
          // Source: drake/systems/framework/system_scalar_converter.h
          const char* doc_1args =
R"""(Creates a converter that uses S's scalar-converting copy constructor
to perform system scalar conversion. That constructor takes the form
of:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    template <typename T>
    class FooSystem final : public LeafSystem<T> {
      template <typename U>
      explicit FooSystem(const FooSystem<U>& other);
    };

.. raw:: html

    </details>

By default, the converter supports conversions to and from all of the
default_scalars "default scalars", but systems may specialize the
scalar_conversion∷Traits to disable support for some or all of these
conversions. Conversions where ``T`` and ``U`` types are the same are
not supported.

This an implicit conversion constructor (not marked ``explicit``), in
order to make calling code substantially more readable, with
relatively little risk of an unwanted accidental conversion happening.

See system_scalar_conversion for additional overview documentation.)""";
        } ctor;
        // Symbol: drake::systems::SystemScalarConverter::empty
        struct /* empty */ {
          // Source: drake/systems/framework/system_scalar_converter.h
          const char* doc =
R"""(Returns true iff no conversions are supported. (In other words,
whether this is a default-constructed object.))""";
        } empty;
      } SystemScalarConverter;
      // Symbol: drake::systems::SystemSymbolicInspector
      struct /* SystemSymbolicInspector */ {
        // Source: drake/systems/framework/system_symbolic_inspector.h
        const char* doc =
R"""(The SystemSymbolicInspector uses symbolic∷Expressions to analyze
various properties of the System, such as time invariance and
input-to-output sparsity, along with many others.

A SystemSymbolicInspector is only interesting if the Context contains
purely vector-valued elements. If any abstract-valued elements are
present, the SystemSymbolicInspector will not be able to parse the
governing equations reliably.

It would be possible to report system properties for a specific
configuration of the abstract inputs, state, or parameters. We
intentionally do not provide such an analysis, because it would invite
developers to shoot themselves in the foot by accidentally overstating
sparsity, for instance if a given input affects a given output in some
modes, but not the mode tested.

Even with that limitation on scope, SystemSymbolicInspector has risks,
if the System contains C++ native conditionals like "if" or "switch".
symbolic∷Expression does not provide an implicit conversion to
``bool``, so it is unlikely that anyone will accidentally write a
System that both uses native conditionals and compiles with a
symbolic∷Expression scalar type. However, it is possible, for instance
using an explicit cast, or ``std∷equal_to``.)""";
        // Symbol: drake::systems::SystemSymbolicInspector::HasAffineDynamics
        struct /* HasAffineDynamics */ {
          // Source: drake/systems/framework/system_symbolic_inspector.h
          const char* doc =
R"""(Returns true iff all of the derivatives and discrete updates have at
most an affine dependence on state and input. Note that the return
value does NOT depend on the output methods (they may be affine or
not).)""";
        } HasAffineDynamics;
        // Symbol: drake::systems::SystemSymbolicInspector::IsAbstract
        struct /* IsAbstract */ {
          // Source: drake/systems/framework/system_symbolic_inspector.h
          const char* doc =
R"""(Returns true if any field in the ``context`` is abstract-valued.)""";
        } IsAbstract;
        // Symbol: drake::systems::SystemSymbolicInspector::IsConnectedInputToOutput
        struct /* IsConnectedInputToOutput */ {
          // Source: drake/systems/framework/system_symbolic_inspector.h
          const char* doc =
R"""(Returns true if the input port at the given ``input_port_index`` is or
might possibly be a term in the output at the given
``output_port_index``.)""";
        } IsConnectedInputToOutput;
        // Symbol: drake::systems::SystemSymbolicInspector::IsTimeInvariant
        struct /* IsTimeInvariant */ {
          // Source: drake/systems/framework/system_symbolic_inspector.h
          const char* doc =
R"""(Returns true if there is no dependence on time in the dynamics
(continuous nor discrete) nor the outputs.)""";
        } IsTimeInvariant;
        // Symbol: drake::systems::SystemSymbolicInspector::SystemSymbolicInspector
        struct /* ctor */ {
          // Source: drake/systems/framework/system_symbolic_inspector.h
          const char* doc =
R"""(Constructs a SystemSymbolicInspector for the given ``system`` by
initializing every vector-valued element in the Context with symbolic
variables.)""";
        } ctor;
        // Symbol: drake::systems::SystemSymbolicInspector::constraints
        struct /* constraints */ {
          // Source: drake/systems/framework/system_symbolic_inspector.h
          const char* doc =
R"""(Returns a reference to the symbolic representation of the constraints.)""";
        } constraints;
        // Symbol: drake::systems::SystemSymbolicInspector::continuous_state
        struct /* continuous_state */ {
          // Source: drake/systems/framework/system_symbolic_inspector.h
          const char* doc =
R"""(Returns a reference to the symbolic representation of the continuous
state.)""";
        } continuous_state;
        // Symbol: drake::systems::SystemSymbolicInspector::derivatives
        struct /* derivatives */ {
          // Source: drake/systems/framework/system_symbolic_inspector.h
          const char* doc =
R"""(Returns a copy of the symbolic representation of the continuous-time
dynamics.)""";
        } derivatives;
        // Symbol: drake::systems::SystemSymbolicInspector::discrete_state
        struct /* discrete_state */ {
          // Source: drake/systems/framework/system_symbolic_inspector.h
          const char* doc =
R"""(Returns a reference to the symbolic representation of the discrete
state.

Parameter ``i``:
    The discrete state group number.)""";
        } discrete_state;
        // Symbol: drake::systems::SystemSymbolicInspector::discrete_update
        struct /* discrete_update */ {
          // Source: drake/systems/framework/system_symbolic_inspector.h
          const char* doc =
R"""(Returns a reference to the symbolic representation of the
discrete-time dynamics.

Parameter ``i``:
    The discrete state group number.)""";
        } discrete_update;
        // Symbol: drake::systems::SystemSymbolicInspector::input
        struct /* input */ {
          // Source: drake/systems/framework/system_symbolic_inspector.h
          const char* doc =
R"""(Returns a reference to the symbolic representation of the input.

Parameter ``i``:
    The input port number.)""";
        } input;
        // Symbol: drake::systems::SystemSymbolicInspector::numeric_parameters
        struct /* numeric_parameters */ {
          // Source: drake/systems/framework/system_symbolic_inspector.h
          const char* doc =
R"""(Returns a reference to the symbolic representation of the numeric
parameters.

Parameter ``i``:
    The numeric parameter group number.)""";
        } numeric_parameters;
        // Symbol: drake::systems::SystemSymbolicInspector::output
        struct /* output */ {
          // Source: drake/systems/framework/system_symbolic_inspector.h
          const char* doc =
R"""(Returns a reference to the symbolic representation of the output.

Parameter ``i``:
    The output port number.)""";
        } output;
        // Symbol: drake::systems::SystemSymbolicInspector::time
        struct /* time */ {
          // Source: drake/systems/framework/system_symbolic_inspector.h
          const char* doc =
R"""(@name Reference symbolic components A set of accessor methods that
provide direct access to the symbolic forms of the System. This class
carefully sets up and names all of the symbolic elements of the
Context, and other methods should be able to reap the benefits.
Returns a reference to the symbolic representation of time.)""";
        } time;
      } SystemSymbolicInspector;
      // Symbol: drake::systems::SystemTypeTag
      struct /* SystemTypeTag */ {
        // Source: drake/systems/framework/system_type_tag.h
        const char* doc =
R"""(A tag object that denotes a System subclass ``S`` in function
signatures.

For example, ``SystemTypeTag<MySystem>{}`` will create a dummy object
that can be used to call functions that look like:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    template <template <typename> class S>
    const char* get_foo(SystemTypeTag<S>) { return S<double>∷get_foo(); }
    
    int main() {
       std∷cout << get_foo(SystemTypeTag<MySystem>{});
    }

.. raw:: html

    </details>

In this case, we could directly call get_foo<MySystem>() by specifying
the template argument, but that is not always possible. In particular,
tag objects are acutely useful when calling templated constructors,
because there is no other mechanism for the caller to specify the
template type.)""";
        // Symbol: drake::systems::SystemTypeTag::SystemTypeTag<S>
        struct /* ctor */ {
          // Source: drake/systems/framework/system_type_tag.h
          const char* doc = R"""()""";
        } ctor;
      } SystemTypeTag;
      // Symbol: drake::systems::SystemVisitor
      struct /* SystemVisitor */ {
        // Source: drake/systems/framework/system_visitor.h
        const char* doc =
R"""(Provides a "Visitor Pattern" for System and Diagram. Rather than
adding more virtual methods to the System base class, or performing a
dynamic_cast to test if a System is a Diagram, you may use the visitor
pattern enabled by this class, e.g.:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    template <typename T>
    class MySystemVisitor : public SystemVisitor {
    ...
    }
    
    MySystemVisitor<T> visitor;
    system.Accept(visitor);

.. raw:: html

    </details>

will call the correct ``Visit`` overload.

Note:
    This method does *not* recurse through the subsystems of a
    Diagram, but that is easy to do: just call Diagram∷GetSystems() in
    your visitor and then call Accept on the subsystems.)""";
        // Symbol: drake::systems::SystemVisitor::SystemVisitor<T>
        struct /* ctor */ {
          // Source: drake/systems/framework/system_visitor.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::systems::SystemVisitor::VisitDiagram
        struct /* VisitDiagram */ {
          // Source: drake/systems/framework/system_visitor.h
          const char* doc =
R"""(This method will be called by System<T>∷accept() if the System *is* a
Diagram<T>.)""";
        } VisitDiagram;
        // Symbol: drake::systems::SystemVisitor::VisitSystem
        struct /* VisitSystem */ {
          // Source: drake/systems/framework/system_visitor.h
          const char* doc =
R"""(This method will be called by System<T>∷accept() if the System *is
not* a Diagram<T>.)""";
        } VisitSystem;
      } SystemVisitor;
      // Symbol: drake::systems::TriggerType
      struct /* TriggerType */ {
        // Source: drake/systems/framework/event.h
        const char* doc =
R"""(Predefined types of triggers for events. Used at run time to determine
why the associated event has occurred.)""";
        // Symbol: drake::systems::TriggerType::kForced
        struct /* kForced */ {
          // Source: drake/systems/framework/event.h
          const char* doc =
R"""(This trigger indicates that an associated event is triggered by
directly calling the corresponding public system API for event
handling (e.g. Publish(context)).)""";
        } kForced;
        // Symbol: drake::systems::TriggerType::kInitialization
        struct /* kInitialization */ {
          // Source: drake/systems/framework/event.h
          const char* doc =
R"""(This trigger indicates that an associated event is triggered at system
initialization.)""";
        } kInitialization;
        // Symbol: drake::systems::TriggerType::kPerStep
        struct /* kPerStep */ {
          // Source: drake/systems/framework/event.h
          const char* doc =
R"""(This trigger indicates that an associated event is triggered whenever
a ``solver`` takes a ``step``. A ``solver`` is an abstract construct
that controls or tracks the time and state evolution of a System. A
simulator is a ``solver``- it advances time a finite duration by
integrating a system, modifying its state accordingly- as is a process
that receives some numeric state from IPC that is then used to, e.g.,
update abstract state. Steps may occur at irregular time intervals: a
step typically coincides with a point in time where it is advantageous
to poll for events, like immediately after an integrator has advanced
time and state.

Per-step events are most commonly created in
System∷GetPerStepEvents(). A very common use of such per-step events
is to update a discrete or abstract state variable that changes
whenever the continuous state advances; examples are computing the
"min" or "max" of some state variable, recording a signal in a delay
buffer, or publishing. Per-step events are also useful to implement
feedback controllers interfaced with physical devices; the controller
can be implemented in the event handler, and the "step" would
correspond to receiving sensory data from the hardware.)""";
        } kPerStep;
        // Symbol: drake::systems::TriggerType::kPeriodic
        struct /* kPeriodic */ {
          // Source: drake/systems/framework/event.h
          const char* doc =
R"""(This type indicates that an associated event is triggered by the
system proceeding to a time t ∈ {tᵢ = t₀ + p * i} for some period p,
time offset t₀, and i is a non-negative integer.

See also:
    PeriodicEventData. Periodic events are commonly created in
    System∷CalcNextUpdateTime().)""";
        } kPeriodic;
        // Symbol: drake::systems::TriggerType::kTimed
        struct /* kTimed */ {
          // Source: drake/systems/framework/event.h
          const char* doc =
R"""(This trigger indicates that an associated event is triggered by the
system proceeding to a single, arbitrary time. Timed events are
commonly created in System∷CalcNextUpdateTime().)""";
        } kTimed;
        // Symbol: drake::systems::TriggerType::kUnknown
        struct /* kUnknown */ {
          // Source: drake/systems/framework/event.h
          const char* doc = R"""()""";
        } kUnknown;
        // Symbol: drake::systems::TriggerType::kWitness
        struct /* kWitness */ {
          // Source: drake/systems/framework/event.h
          const char* doc =
R"""(This trigger indicates that an associated event is triggered by the
zero crossing of a witness function.

See also:
    WitnessTriggeredEventData.)""";
        } kWitness;
      } TriggerType;
      // Symbol: drake::systems::TriggerTypeSet
      struct /* TriggerTypeSet */ {
        // Source: drake/systems/framework/event.h
        const char* doc =
R"""(This set-type alias provides a convenient API vocabulary for systems
to specify multiple trigger types.)""";
      } TriggerTypeSet;
      // Symbol: drake::systems::UnrestrictedUpdateEvent
      struct /* UnrestrictedUpdateEvent */ {
        // Source: drake/systems/framework/event.h
        const char* doc =
R"""(This class represents an unrestricted update event. It has an optional
callback function to do custom handling of this event, and that can
write updates to a mutable, non-null State object.

See also:
    LeafSystem for more convenient interfaces to unrestricted update
    events via the Declare*UnrestrictedUpdateEvent() methods.)""";
        // Symbol: drake::systems::UnrestrictedUpdateEvent::UnrestrictedUpdateEvent<T>
        struct /* ctor */ {
          // Source: drake/systems/framework/event.h
          const char* doc_0args =
R"""(Constructs an empty UnrestrictedUpdateEvent.)""";
          // Source: drake/systems/framework/event.h
          const char* doc_1args =
R"""(Constructs an UnrestrictedUpdateEvent with the given callback
function.)""";
        } ctor;
        // Symbol: drake::systems::UnrestrictedUpdateEvent::handle
        struct /* handle */ {
          // Source: drake/systems/framework/event.h
          const char* doc =
R"""(Calls the optional callback function, if one exists, with ``system``,
``context``, ``this`` and ``state``.)""";
        } handle;
        // Symbol: drake::systems::UnrestrictedUpdateEvent::is_discrete_update
        struct /* is_discrete_update */ {
          // Source: drake/systems/framework/event.h
          const char* doc = R"""()""";
        } is_discrete_update;
      } UnrestrictedUpdateEvent;
      // Symbol: drake::systems::UseDefaultName
      struct /* UseDefaultName */ {
        // Source: drake/systems/framework/framework_common.h
        const char* doc =
R"""((Advanced.) Tag type that indicates a system or port should use a
default name, instead of a user-provided name. Most users will use the
kUseDefaultName constant, without ever having to mention this type.)""";
      } UseDefaultName;
      // Symbol: drake::systems::ValueProducer
      struct /* ValueProducer */ {
        // Source: drake/systems/framework/value_producer.h
        const char* doc =
R"""(ValueProducer computes an AbstractValue output based on a ContextBase
input. This is commonly used for declaring output ports and cache
entries.

It provides two functions for that purpose: - Allocate() returns new
storage that is suitably typed to hold the output. - Calc() takes a
context as input and writes to an output pointer.

For example, given this example calculator lambda:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    std∷function calc = [](const Context<T>& context, std∷string* output) {
    output = std∷to_string(context.get_time());
    };

.. raw:: html

    </details>

We can capture it into a producer and then call it:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ValueProducer producer(calc);
    std∷unique_ptr<AbstractValue> storage = producer.Allocate();
    const LeafContext<T> context;
    producer.Calc(context, storage.get());
    EXPECT_THAT(storage->get_value<std∷string>(), ∷testing∷StartsWith("0.0"));

.. raw:: html

    </details>

Sugar is provided to create ValueProducer objects from function
pointers that operate on un-erased types, so that the user can ignore
the details of type erasure and Context<T> downcasting. Refer to the
ValueProducer_constructors "Constructor overloads" for details.)""";
        // Symbol: drake::systems::ValueProducer::Allocate
        struct /* Allocate */ {
          // Source: drake/systems/framework/value_producer.h
          const char* doc =
R"""(Invokes the allocate function provided to the constructor.

Raises:
    RuntimeError if is_valid() is false.)""";
        } Allocate;
        // Symbol: drake::systems::ValueProducer::AllocateCallback
        struct /* AllocateCallback */ {
          // Source: drake/systems/framework/value_producer.h
          const char* doc =
R"""(Signature of a function suitable for allocating an object that can
hold a value compatible with our Calc function. The result is always
returned as an AbstractValue but must contain the correct concrete
type.)""";
        } AllocateCallback;
        // Symbol: drake::systems::ValueProducer::Calc
        struct /* Calc */ {
          // Source: drake/systems/framework/value_producer.h
          const char* doc =
R"""(Invokes the calc function provided to the constructor.

Raises:
    RuntimeError if is_valid() is false.)""";
        } Calc;
        // Symbol: drake::systems::ValueProducer::CalcCallback
        struct /* CalcCallback */ {
          // Source: drake/systems/framework/value_producer.h
          const char* doc =
R"""(Signature of a function suitable for calculating a context-dependent
value, given a place to put the value. The function may presume that
the storage pointed to by the second argument will be of the proper
type (as returned by an AllocateCallback), but should not presume that
the storage has been initialized with any particular value; the
function should always fully overwrite the output storage with a new
value.)""";
        } CalcCallback;
        // Symbol: drake::systems::ValueProducer::NoopCalc
        struct /* NoopCalc */ {
          // Source: drake/systems/framework/value_producer.h
          const char* doc =
R"""(This static function is provided for users who need an empty
CalcCallback. Passing ``&ValueProducer∷NoopCalc`` as ValueProducer's
last constructor argument will create a function that does not compute
anything, but can still allocate.)""";
        } NoopCalc;
        // Symbol: drake::systems::ValueProducer::ValueProducer
        struct /* ctor */ {
          // Source: drake/systems/framework/value_producer.h
          const char* doc =
R"""(Creates an invalid object; calls to Allocate or Calc will throw.)""";
          // Source: drake/systems/framework/value_producer.h
          const char* doc_overload_5d =
R"""(Overload (5d). Refer to the C++ ValueProducer_constructors
"Constructor overloads" for further details.

Parameter ``allocate``:
    Callback function that allocates storage for the value. It takes
    no arguments and must return an AbstractValue.

Parameter ``calc``:
    Callback function that computes the value. It takes two arguments
    (context, value) and does not return anything; instead, it should
    mutate the AbstractValue object pointed to by ``value`` with the
    new result.)""";
        } ctor;
        // Symbol: drake::systems::ValueProducer::is_valid
        struct /* is_valid */ {
          // Source: drake/systems/framework/value_producer.h
          const char* doc =
R"""(Returns true iff the allocate and calc callbacks are both non-null.
(The only way they can be null is if the ValueProducer was default
constructed or moved from.))""";
        } is_valid;
      } ValueProducer;
      // Symbol: drake::systems::VectorBase
      struct /* VectorBase */ {
        // Source: drake/systems/framework/vector_base.h
        const char* doc =
R"""(VectorBase is an abstract base class that real-valued signals between
Systems and real-valued System state vectors must implement. Classes
that inherit from VectorBase will typically provide names for the
elements of the vector, and may also provide other computations for
the convenience of Systems handling the signal. The vector is always a
column vector. It may or may not be contiguous in memory. Contiguous
subclasses should typically inherit from BasicVector, not from
VectorBase directly.)""";
        // Symbol: drake::systems::VectorBase::CopyToPreSizedVector
        struct /* CopyToPreSizedVector */ {
          // Source: drake/systems/framework/vector_base.h
          const char* doc =
R"""(Copies this entire VectorBase into a pre-sized Eigen Vector.

Implementations should ensure this operation is O(N) in the size of
the value.

Raises:
    RuntimeError if ``vec`` is the wrong size.)""";
        } CopyToPreSizedVector;
        // Symbol: drake::systems::VectorBase::CopyToVector
        struct /* CopyToVector */ {
          // Source: drake/systems/framework/vector_base.h
          const char* doc =
R"""(Copies this entire VectorBase into a contiguous Eigen Vector.

Implementations should ensure this operation is O(N) in the size of
the value and allocates only the O(N) memory that it returns.)""";
        } CopyToVector;
        // Symbol: drake::systems::VectorBase::DoGetAtIndexChecked
        struct /* DoGetAtIndexChecked */ {
          // Source: drake/systems/framework/vector_base.h
          const char* doc =
R"""(Implementations should ensure this operation is O(1) and allocates no
memory. The index has already been checked for negative, but not size.
Implementations must throw an exception when index >= size().)""";
        } DoGetAtIndexChecked;
        // Symbol: drake::systems::VectorBase::DoGetAtIndexUnchecked
        struct /* DoGetAtIndexUnchecked */ {
          // Source: drake/systems/framework/vector_base.h
          const char* doc =
R"""(Implementations should ensure this operation is O(1) and allocates no
memory. The index need not be validated when in release mode.)""";
        } DoGetAtIndexUnchecked;
        // Symbol: drake::systems::VectorBase::DoPlusEqScaled
        struct /* DoPlusEqScaled */ {
          // Source: drake/systems/framework/vector_base.h
          const char* doc =
R"""(Adds in multiple scaled vectors to this vector. All vectors are
guaranteed to be the same size.

You should override this method if possible with a more efficient
approach that leverages structure; the default implementation performs
element-by-element computations that are likely inefficient, but even
this implementation minimizes memory accesses for efficiency. If the
vector is contiguous, for example, implementations that leverage SIMD
operations should be far more efficient. Overriding implementations
should ensure that this operation remains O(N) in the size of the
value and allocates no memory.)""";
        } DoPlusEqScaled;
        // Symbol: drake::systems::VectorBase::GetAtIndex
        struct /* GetAtIndex */ {
          // Source: drake/systems/framework/vector_base.h
          const char* doc =
R"""(Returns the element at the given index in the vector.

Raises:
    RuntimeError if the index is >= size() or negative. Consider
    operator[]() instead if bounds-checking is unwanted.)""";
        } GetAtIndex;
        // Symbol: drake::systems::VectorBase::GetElementBounds
        struct /* GetElementBounds */ {
          // Source: drake/systems/framework/vector_base.h
          const char* doc =
R"""(Get the bounds for the elements. If lower and upper are both empty
size vectors, then there are no bounds. Otherwise, the bounds are
(*lower)(i) <= GetAtIndex(i) <= (*upper)(i) The default output is no
bounds.)""";
        } GetElementBounds;
        // Symbol: drake::systems::VectorBase::PlusEqScaled
        struct /* PlusEqScaled */ {
          // Source: drake/systems/framework/vector_base.h
          const char* doc_2args =
R"""(Add in scaled vector ``rhs`` to this vector.

Raises:
    RuntimeError if ``rhs`` is a different size than this.)""";
          // Source: drake/systems/framework/vector_base.h
          const char* doc_1args =
R"""(Add in multiple scaled vectors to this vector.

Raises:
    RuntimeError if any rhs are a different size than this.)""";
        } PlusEqScaled;
        // Symbol: drake::systems::VectorBase::ScaleAndAddToVector
        struct /* ScaleAndAddToVector */ {
          // Source: drake/systems/framework/vector_base.h
          const char* doc =
R"""(Adds a scaled version of this vector to Eigen vector ``vec``.

Raises:
    RuntimeError if ``vec`` is the wrong size.

Implementations should ensure this operation remains O(N) in the size
of the value and allocates no memory.)""";
        } ScaleAndAddToVector;
        // Symbol: drake::systems::VectorBase::SetAtIndex
        struct /* SetAtIndex */ {
          // Source: drake/systems/framework/vector_base.h
          const char* doc =
R"""(Replaces the state at the given index with the value.

Raises:
    RuntimeError if the index is >= size(). Consider operator[]()
    instead if bounds-checking is unwanted.)""";
        } SetAtIndex;
        // Symbol: drake::systems::VectorBase::SetFrom
        struct /* SetFrom */ {
          // Source: drake/systems/framework/vector_base.h
          const char* doc =
R"""(Replaces the entire vector with the contents of ``value``.

Raises:
    RuntimeError if ``value`` is not a column vector with size() rows.

Implementations should ensure this operation is O(N) in the size of
the value and allocates no memory.)""";
        } SetFrom;
        // Symbol: drake::systems::VectorBase::SetFromVector
        struct /* SetFromVector */ {
          // Source: drake/systems/framework/vector_base.h
          const char* doc =
R"""(Replaces the entire vector with the contents of ``value``.

Raises:
    RuntimeError if ``value`` is not a column vector with size() rows.

Implementations should ensure this operation is O(N) in the size of
the value and allocates no memory.)""";
        } SetFromVector;
        // Symbol: drake::systems::VectorBase::SetZero
        struct /* SetZero */ {
          // Source: drake/systems/framework/vector_base.h
          const char* doc =
R"""(Sets all elements of this vector to zero.)""";
        } SetZero;
        // Symbol: drake::systems::VectorBase::ThrowMismatchedSize
        struct /* ThrowMismatchedSize */ {
          // Source: drake/systems/framework/vector_base.h
          const char* doc = R"""()""";
        } ThrowMismatchedSize;
        // Symbol: drake::systems::VectorBase::ThrowOutOfRange
        struct /* ThrowOutOfRange */ {
          // Source: drake/systems/framework/vector_base.h
          const char* doc = R"""()""";
        } ThrowOutOfRange;
        // Symbol: drake::systems::VectorBase::VectorBase<T>
        struct /* ctor */ {
          // Source: drake/systems/framework/vector_base.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::systems::VectorBase::operator+=
        struct /* operator_iadd */ {
          // Source: drake/systems/framework/vector_base.h
          const char* doc =
R"""(Add in vector ``rhs`` to this vector.

Raises:
    RuntimeError if ``rhs`` is a different size than this.)""";
        } operator_iadd;
        // Symbol: drake::systems::VectorBase::operator-=
        struct /* operator_isub */ {
          // Source: drake/systems/framework/vector_base.h
          const char* doc =
R"""(Subtract in vector ``rhs`` to this vector.

Raises:
    RuntimeError if ``rhs`` is a different size than this.)""";
        } operator_isub;
        // Symbol: drake::systems::VectorBase::operator[]
        struct /* operator_array */ {
          // Source: drake/systems/framework/vector_base.h
          const char* doc =
R"""(Returns the element at the given index in the vector.

Precondition:
    0 <= ``index`` < size())""";
        } operator_array;
        // Symbol: drake::systems::VectorBase::size
        struct /* size */ {
          // Source: drake/systems/framework/vector_base.h
          const char* doc =
R"""(Returns the number of elements in the vector.

Implementations should ensure this operation is O(1) and allocates no
memory.)""";
        } size;
      } VectorBase;
      // Symbol: drake::systems::VectorSystem
      struct /* VectorSystem */ {
        // Source: drake/systems/framework/vector_system.h
        const char* doc =
R"""(A base class that specializes LeafSystem for use with only zero or one
vector input ports, and only zero or one vector output ports.

.. pydrake_system::

    name: VectorSystem
    input_ports:
    - u0
    output_ports:
    - y0

By default, this base class does not declare any state; subclasses may
optionally declare continuous or discrete state, but not both;
subclasses may not declare abstract state.)""";
        // Symbol: drake::systems::VectorSystem::CalcVectorOutput
        struct /* CalcVectorOutput */ {
          // Source: drake/systems/framework/vector_system.h
          const char* doc =
R"""(Converts the parameters to Eigen∷VectorBlock form, then delegates to
DoCalcVectorOutput().)""";
        } CalcVectorOutput;
        // Symbol: drake::systems::VectorSystem::DeclarePeriodicDiscreteUpdate
        struct /* DeclarePeriodicDiscreteUpdate */ {
          // Source: drake/systems/framework/vector_system.h
          const char* doc =
R"""(Declares a discrete update rate. You must override
DoCalcVectorDiscreteVariableUpdates() to handle the update.)""";
        } DeclarePeriodicDiscreteUpdate;
        // Symbol: drake::systems::VectorSystem::DoCalcTimeDerivatives
        struct /* DoCalcTimeDerivatives */ {
          // Source: drake/systems/framework/vector_system.h
          const char* doc =
R"""(Converts the parameters to Eigen∷VectorBlock form, then delegates to
DoCalcVectorTimeDerivatives().)""";
        } DoCalcTimeDerivatives;
        // Symbol: drake::systems::VectorSystem::DoCalcVectorDiscreteVariableUpdates
        struct /* DoCalcVectorDiscreteVariableUpdates */ {
          // Source: drake/systems/framework/vector_system.h
          const char* doc =
R"""(Provides a convenience method for VectorSystem subclasses. This method
serves as the callback for DeclarePeriodicDiscreteUpdate(),
immediately above.

The ``state`` will be either empty or the discrete state, depending on
whether discrete state was declared at context-creation time.

By default, this function does nothing if the ``next_state`` is empty,
and throws an exception otherwise.)""";
        } DoCalcVectorDiscreteVariableUpdates;
        // Symbol: drake::systems::VectorSystem::DoCalcVectorOutput
        struct /* DoCalcVectorOutput */ {
          // Source: drake/systems/framework/vector_system.h
          const char* doc =
R"""(Provides a convenience method for VectorSystem subclasses. This method
performs the same logical operation as System∷DoCalcOutput but
provides VectorBlocks to represent the input, state, and output.
Subclasses with outputs should override this method, and not the base
class method (which is ``final``).

The ``state`` will be either empty, the continuous state, or the
discrete state, depending on which (or none) was declared at
context-creation time.

The ``input`` will be empty (zero-sized) when this System is declared
to be non-direct-feedthrough.

By default, this function does nothing if the ``output`` is empty, and
throws an exception otherwise.)""";
        } DoCalcVectorOutput;
        // Symbol: drake::systems::VectorSystem::DoCalcVectorTimeDerivatives
        struct /* DoCalcVectorTimeDerivatives */ {
          // Source: drake/systems/framework/vector_system.h
          const char* doc =
R"""(Provides a convenience method for VectorSystem subclasses. This method
performs the same logical operation as System∷DoCalcTimeDerivatives
but provides VectorBlocks to represent the input, continuous state,
and derivatives. Subclasses should override this method, and not the
base class method (which is ``final``). The ``state`` will be either
empty or the continuous state, depending on whether continuous state
was declared at context-creation time.

By default, this function does nothing if the ``derivatives`` are
empty, and throws an exception otherwise.)""";
        } DoCalcVectorTimeDerivatives;
        // Symbol: drake::systems::VectorSystem::EvalVectorInput
        struct /* EvalVectorInput */ {
          // Source: drake/systems/framework/vector_system.h
          const char* doc =
R"""(Causes the vector-valued input port to become up-to-date, and returns
the port's value as an Eigen vector. If the system has zero inputs,
then returns an empty vector.)""";
        } EvalVectorInput;
        // Symbol: drake::systems::VectorSystem::GetVectorState
        struct /* GetVectorState */ {
          // Source: drake/systems/framework/vector_system.h
          const char* doc =
R"""(Returns a reference to an Eigen vector version of the state from
within the Context.)""";
        } GetVectorState;
        // Symbol: drake::systems::VectorSystem::VectorSystem<T>
        struct /* ctor */ {
          // Source: drake/systems/framework/vector_system.h
          const char* doc_3args =
R"""(Creates a system with one input port and one output port of the given
sizes, when the sizes are non-zero. Either size can be zero, in which
case no input (or output) port is created.

The ``direct_feedthrough`` specifies whether the input port direct
feeds through to the output port. (See
SystemBase∷GetDirectFeedthroughs().) When nullopt, assumes true (the
output is direct feedthrough). When false, the DoCalcVectorOutput
``input`` will be empty (zero-sized).

Does *not* declare scalar-type conversion support (AutoDiff, etc.). To
enable AutoDiff support, use the SystemScalarConverter-based
constructor. (For that, see system_scalar_conversion at the example
titled "Example using drake∷systems∷VectorSystem as the base class".))""";
          // Source: drake/systems/framework/vector_system.h
          const char* doc_4args =
R"""(Creates a system with one input port and one output port of the given
sizes, when the sizes are non-zero. Either size can be zero, in which
case no input (or output) port is created. This constructor allows
subclasses to declare scalar-type conversion support (AutoDiff, etc.).

The ``direct_feedthrough`` specifies whether the input port direct
feeds through to the output port. (See
SystemBase∷GetDirectFeedthroughs().) When nullopt, infers feedthrough
from the symbolic form if available, or else assumes true (the output
is direct feedthrough). When false, the DoCalcVectorOutput ``input``
will be empty (zero-sized).

The scalar-type conversion support will use ``converter``. To enable
scalar-type conversion support, pass a ``SystemTypeTag<S>{}`` where
``S`` must be the exact class of ``this`` being constructed.

See system_scalar_conversion for detailed background and examples
related to scalar-type conversion support, especially the example
titled "Example using drake∷systems∷VectorSystem as the base class".)""";
        } ctor;
      } VectorSystem;
      // Symbol: drake::systems::WitnessFunction
      struct /* WitnessFunction */ {
        // Source: drake/systems/framework/witness_function.h
        const char* doc =
R"""(Class that stores a function that is able to help determine the time
and state at which a step of the initial value problem integration of
a System should end, which may be done for any number of purposes,
including publishing or state reinitialization (i.e., event handling).
System authors declare witness functions through
LeafSystem∷MakeWitnessFunction().

For the ensuing discussion, consider two times (``t₀`` and ``t₁ >
t₀``) and states corresponding to those times (``x(t₀)`` and
``x(t₁)``). A witness function, ``w(t, x)``, "triggers" only when it
crosses zero at a time ``t*`` where ``t₀ < t* ≤ t₁``. Note the
half-open interval. For an example of a witness function, consider the
"signed distance" (i.e., Euclidean distance when bodies are disjoint
and minimum translational distance when bodies intersect) between two
rigid bodies; this witness function can be used to determine both the
time of impact for rigid bodies and their states at that time of
impact.

Precision in the definition of the witness function is necessary,
because we want the witness function to trigger only once if, for
example, ``w(t₀, x(t₀)) ≠ 0``, `w(t₁, x(t₁)) = 0`, and ``w(t₂, x(t₂))
≠ 0``, for some t₂ > t₁. In other words, if the witness function is
evaluated over the intervals [t₀, t₁] and [t₁, t₂], meaning that the
zero occurs precisely at an interval endpoint, the witness function
should trigger once. Similarly, the witness function should trigger
exactly once if ``w(t₀, x(t₀)) ≠ 0``, `w(t*, x(t*)) = 0`, and ``w(t₁,
x(t₁)) = 0``, for ``t* ∈ (t₀, t₁)``. We can define the trigger
condition formally over interval ``[t₀, t₁]`` using the function:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    T(w, t₀, x(t₀), t₁) =   1   if w(t₀, x(t₀)) ≠ 0 and
                                   w(t₀, x(t₀))⋅w(t₁, x(t₁)) ≤ 0
                            0   if w(t₀, x(t₀)) = 0 or
                                   w(t₀, x(t₀))⋅w(t₁, x(t₁)) > 0

.. raw:: html

    </details>

We wish for the witness function to trigger if the trigger function
evaluates to one. The trigger function can be further modified, if
desired, to incorporate the constraint that the witness function
should trigger only when crossing from positive values to negative
values, or vice versa.

A good witness function should not cross zero repeatedly over a small
interval of time (relative to the maximum designated integration step
size) or over small changes in state; when a witness function has been
"bracketed" over an interval of time (i.e., it changes sign), that
witness function will ideally cross zero only once in that interval.

A witness function trigger time is isolated only to a small interval
of time (as described in Simulator). The disadvantage of this scheme
is that it always requires the length of the interval to be reduced to
the requisite length *and that each function evaluation (which
requires numerical integration) is extraordinarily expensive*. If, for
example, the (slow) bisection algorithm were used to isolate the time
interval, the number of integrations necessary to cut the interval
from a length of ℓ to a length of ε will be log₂(ℓ / ε). Bisection is
just one of several possible algorithms for isolating the time
interval, though it's a reliable choice and always converges linearly.)""";
        // Symbol: drake::systems::WitnessFunction::CalcCallback
        struct /* CalcCallback */ {
          // Source: drake/systems/framework/witness_function.h
          const char* doc =
R"""(Signature of a function suitable for calculating a value of a
particular witness function.)""";
        } CalcCallback;
        // Symbol: drake::systems::WitnessFunction::CalcWitnessValue
        struct /* CalcWitnessValue */ {
          // Source: drake/systems/framework/witness_function.h
          const char* doc =
R"""(Evaluates the witness function at the given context.)""";
        } CalcWitnessValue;
        // Symbol: drake::systems::WitnessFunction::WitnessFunction<T>
        struct /* ctor */ {
          // Source: drake/systems/framework/witness_function.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::systems::WitnessFunction::description
        struct /* description */ {
          // Source: drake/systems/framework/witness_function.h
          const char* doc =
R"""(Gets the description of this witness function (used primarily for
logging and debugging).)""";
        } description;
        // Symbol: drake::systems::WitnessFunction::direction_type
        struct /* direction_type */ {
          // Source: drake/systems/framework/witness_function.h
          const char* doc =
R"""(Gets the direction(s) under which this witness function triggers.)""";
        } direction_type;
        // Symbol: drake::systems::WitnessFunction::get_event
        struct /* get_event */ {
          // Source: drake/systems/framework/witness_function.h
          const char* doc =
R"""(Gets the event that will be dispatched when the witness function
triggers. A null pointer indicates that no event will be dispatched.)""";
        } get_event;
        // Symbol: drake::systems::WitnessFunction::get_mutable_event
        struct /* get_mutable_event */ {
          // Source: drake/systems/framework/witness_function.h
          const char* doc =
R"""(Gets a mutable pointer to the event that will occur when the witness
function triggers.)""";
        } get_mutable_event;
        // Symbol: drake::systems::WitnessFunction::get_system
        struct /* get_system */ {
          // Source: drake/systems/framework/witness_function.h
          const char* doc =
R"""(Gets a reference to the System used by this witness function.)""";
        } get_system;
        // Symbol: drake::systems::WitnessFunction::should_trigger
        struct /* should_trigger */ {
          // Source: drake/systems/framework/witness_function.h
          const char* doc =
R"""(Checks whether the witness function should trigger using given values
at w0 and wf. Note that this function is not specific to a particular
witness function.)""";
        } should_trigger;
      } WitnessFunction;
      // Symbol: drake::systems::WitnessFunctionDirection
      struct /* WitnessFunctionDirection */ {
        // Source: drake/systems/framework/witness_function.h
        const char* doc = R"""()""";
        // Symbol: drake::systems::WitnessFunctionDirection::kCrossesZero
        struct /* kCrossesZero */ {
          // Source: drake/systems/framework/witness_function.h
          const char* doc =
R"""(Witness function triggers *any time* the function crosses/touches
zero, *except* when the witness function evaluates to zero at the
beginning of the interval. Conceptually equivalent to
kPositiveThenNonNegative OR kNegativeThenNonNegative.)""";
        } kCrossesZero;
        // Symbol: drake::systems::WitnessFunctionDirection::kNegativeThenNonNegative
        struct /* kNegativeThenNonNegative */ {
          // Source: drake/systems/framework/witness_function.h
          const char* doc =
R"""(Witness function triggers when the function crosses or touches zero
after an initial negative evaluation.)""";
        } kNegativeThenNonNegative;
        // Symbol: drake::systems::WitnessFunctionDirection::kNone
        struct /* kNone */ {
          // Source: drake/systems/framework/witness_function.h
          const char* doc =
R"""(This witness function will never be triggered.)""";
        } kNone;
        // Symbol: drake::systems::WitnessFunctionDirection::kPositiveThenNonPositive
        struct /* kPositiveThenNonPositive */ {
          // Source: drake/systems/framework/witness_function.h
          const char* doc =
R"""(Witness function triggers when the function crosses or touches zero
after an initial positive evaluation.)""";
        } kPositiveThenNonPositive;
      } WitnessFunctionDirection;
      // Symbol: drake::systems::WitnessTriggeredEventData
      struct /* WitnessTriggeredEventData */ {
        // Source: drake/systems/framework/event.h
        const char* doc =
R"""(An event data variant for storing data from a witness function
triggering to be passed to event handlers. A witness function isolates
time to a (typically small) window during which the witness function
crosses zero. The time and state at both sides of this window are
passed to the event handler so that the system can precisely determine
the reason that the witness function triggered.)""";
        // Symbol: drake::systems::WitnessTriggeredEventData::WitnessTriggeredEventData<T>
        struct /* ctor */ {
          // Source: drake/systems/framework/event.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::systems::WitnessTriggeredEventData::set_t0
        struct /* set_t0 */ {
          // Source: drake/systems/framework/event.h
          const char* doc =
R"""(Sets the time at the left end of the window. Note that ``t0`` should
be smaller than ``tf`` after both values are set.)""";
        } set_t0;
        // Symbol: drake::systems::WitnessTriggeredEventData::set_tf
        struct /* set_tf */ {
          // Source: drake/systems/framework/event.h
          const char* doc =
R"""(Sets the time at the right end of the window. Note that ``tf`` should
be larger than ``t0`` after both values are set.)""";
        } set_tf;
        // Symbol: drake::systems::WitnessTriggeredEventData::set_triggered_witness
        struct /* set_triggered_witness */ {
          // Source: drake/systems/framework/event.h
          const char* doc =
R"""(Sets the witness function that triggered the event handler.)""";
        } set_triggered_witness;
        // Symbol: drake::systems::WitnessTriggeredEventData::set_xc0
        struct /* set_xc0 */ {
          // Source: drake/systems/framework/event.h
          const char* doc =
R"""(Sets a pointer to the continuous state at the left end of the
isolation window.)""";
        } set_xc0;
        // Symbol: drake::systems::WitnessTriggeredEventData::set_xcf
        struct /* set_xcf */ {
          // Source: drake/systems/framework/event.h
          const char* doc =
R"""(Sets a pointer to the continuous state at the right end of the
isolation window.)""";
        } set_xcf;
        // Symbol: drake::systems::WitnessTriggeredEventData::t0
        struct /* t0 */ {
          // Source: drake/systems/framework/event.h
          const char* doc =
R"""(Gets the time at the left end of the window. Default is NaN (which
indicates that the value is invalid).)""";
        } t0;
        // Symbol: drake::systems::WitnessTriggeredEventData::tf
        struct /* tf */ {
          // Source: drake/systems/framework/event.h
          const char* doc =
R"""(Gets the time at the right end of the window. Default is NaN (which
indicates that the value is invalid).)""";
        } tf;
        // Symbol: drake::systems::WitnessTriggeredEventData::triggered_witness
        struct /* triggered_witness */ {
          // Source: drake/systems/framework/event.h
          const char* doc =
R"""(Gets the witness function that triggered the event handler.)""";
        } triggered_witness;
        // Symbol: drake::systems::WitnessTriggeredEventData::xc0
        struct /* xc0 */ {
          // Source: drake/systems/framework/event.h
          const char* doc =
R"""(Gets a pointer to the continuous state at the left end of the
isolation window.)""";
        } xc0;
        // Symbol: drake::systems::WitnessTriggeredEventData::xcf
        struct /* xcf */ {
          // Source: drake/systems/framework/event.h
          const char* doc =
R"""(Gets a pointer to the continuous state at the right end of the
isolation window.)""";
        } xcf;
      } WitnessTriggeredEventData;
      // Symbol: drake::systems::scalar_conversion
      struct /* scalar_conversion */ {
        // Symbol: drake::systems::scalar_conversion::FromDoubleTraits
        struct /* FromDoubleTraits */ {
          // Source: drake/systems/framework/scalar_conversion_traits.h
          const char* doc =
R"""(A concrete traits class providing sugar to support for converting only
from the ``double`` scalar type. For example, if a
MySystem<symbolic∷Expression> cannot be converted into a
MySystem<double>, it could specialize Traits as follows:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    namespace drake {
    namespace systems {
    namespace scalar_conversion {
    template <> struct Traits<MySystem> : public FromDoubleTraits {};
    }  // namespace scalar_conversion
    }  // namespace systems
    }  // namespace drake

.. raw:: html

    </details>)""";
        } FromDoubleTraits;
        // Symbol: drake::systems::scalar_conversion::NonSymbolicTraits
        struct /* NonSymbolicTraits */ {
          // Source: drake/systems/framework/scalar_conversion_traits.h
          const char* doc =
R"""(A concrete traits class providing sugar to disable support for
symbolic evaluation (i.e., the symbolic∷Expression scalar type).

For example, if MySystem does not support the symbolic expression
scalar type, it could specialize Traits as follows:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    namespace drake {
    namespace systems {
    namespace scalar_conversion {
    template <> struct Traits<MySystem> : public NonSymbolicTraits {};
    }  // namespace scalar_conversion
    }  // namespace systems
    }  // namespace drake

.. raw:: html

    </details>)""";
        } NonSymbolicTraits;
        // Symbol: drake::systems::scalar_conversion::Traits
        struct /* Traits */ {
          // Source: drake/systems/framework/scalar_conversion_traits.h
          const char* doc =
R"""(A templated traits class for whether an ``S<U>`` can be converted into
an ``S<T>``; the default value is true for all values of ``S``, `T`,
and ``U``. Particular scalar-dependent classes (``S``) may specialize
this template to indicate whether the framework should support
conversion for any given combination of ``T`` and ``U``.

When ``Traits<S>∷supported<T, U>`` is ``std∷true_type``, the
"scalar-converting copy constructor" that relates ``S``, `T`, and
``U`` will be used. That constructor takes the form of, e.g., when
``S`` is ``Foo``:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    template <typename T>
    class Foo {
      template <typename U>
      explicit Foo(const Foo<U>& other);
    };

.. raw:: html

    </details>

See system_scalar_conversion for detailed background and examples
related to scalar-type conversion support.

When ``Traits<S>∷supported<T, U>`` is ``std∷false_type``, the
``S<T>∷S(const S<U>&)`` scalar-conversion copy constructor will not
even be mentioned by the framework, so that ``S`` need not even
compile for certain values of ``T`` and ``U``.

Template parameter ``S``:
    is the scalar-templated type to copy)""";
        } Traits;
        // Symbol: drake::systems::scalar_conversion::ValueConverter
        struct /* ValueConverter */ {
          // Source: drake/systems/framework/scalar_conversion_traits.h
          const char* doc =
R"""(Converts a scalar ``U u`` to its corresponding scalar ``T t``. When U
== T, the scalar is unchanged. When demoting Expression to
non-Expression, throws when there are unbound variables. In all other
cases, information beyond the double value (e.g., possible
derivatives) might be discarded.)""";
          // Symbol: drake::systems::scalar_conversion::ValueConverter::operator()
          struct /* operator_call */ {
            // Source: drake/systems/framework/scalar_conversion_traits.h
            const char* doc = R"""()""";
          } operator_call;
        } ValueConverter;
      } scalar_conversion;
      // Symbol: drake::systems::system_scalar_converter_internal
      struct /* system_scalar_converter_internal */ {
        // Symbol: drake::systems::system_scalar_converter_internal::AddPydrakeConverterFunction
        struct /* AddPydrakeConverterFunction */ {
          // Source: drake/systems/framework/system_scalar_converter.h
          const char* doc = R"""()""";
        } AddPydrakeConverterFunction;
        // Symbol: drake::systems::system_scalar_converter_internal::Make
        struct /* Make */ {
          // Source: drake/systems/framework/system_scalar_converter.h
          const char* doc = R"""()""";
        } Make;
        // Symbol: drake::systems::system_scalar_converter_internal::ThrowConversionMismatch
        struct /* ThrowConversionMismatch */ {
          // Source: drake/systems/framework/system_scalar_converter.h
          const char* doc = R"""()""";
        } ThrowConversionMismatch;
      } system_scalar_converter_internal;
    } systems;
  } drake;
} pydrake_doc_systems_framework;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
