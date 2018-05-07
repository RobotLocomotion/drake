#pragma once

// Putting this document in drake::systems namespace makes the links work.
namespace drake {
namespace systems {

/** @defgroup cache_design_notes System Cache Design and Implementation Notes

<!-- Fluff needed to keep Doxygen from misformatting due to quotes and
     this being in the "autobrief" location. -->
@parblock                       <center>
          \"There are only two hard things in computer science:<br>
          <b>cache invalidation</b>, and naming things.\"<br>
                            -- Phil Karlton
                                </center>
@endparblock

@warning DRAFT -- to be reviewed along with caching code. The text here refers
to objects that are not yet in Drake's master branch.

<h2>Background</h2>

Drake System objects are used to specify the computational _structure_ of a
model to be studied. The actual _values_ during computation are stored in a
separate Context object. The Context contains _source_ values (time, parameters,
states, input ports) and _computed_ values (e.g. derivatives, output ports)
that depend on some or all of the source values. We call the particular
dependencies of a computed value its _prerequisites_. The caching system
described here manages computed values so that
 - they are recomputed _only if_ a prerequisite has changed,
 - they are marked out of date _whenever_ a prerequisite changes, and
 - _every_ access to a computed value first ensures that it is up to date.

Accessing computed values is a critical, inner-loop activity during
simulation (many times per step) so the implementation is designed to provide
validity-checked access at minimal computational cost. Marking computations
out of date as prerequisites change is also frequent (at least once per step),
and potentially expensive, so must be efficient; the implementation goes to
great lengths to minimize the cost of that operation.

@anchor cache_design_goals
<h2>Design Constraints and Goals</h2>

Caching is entirely about performance. Hence, other than correctness, the
performance goals listed above are the primary architectural constraints. Other
goals also influenced the design. Here are the main goals in roughly descending
order of importance.
 1. Must be correct (same result with caching on or off).
 2. Must be fast.
 3. Must preserve independence of System and Context objects (e.g., no
    cross-pointers).
 4. Should provide a simple conceptual model and API for users.
 5. Should treat all value sources and dependencies in Drake uniformly.
 6. Should be backwards compatible with the existing API.

In service of correctness and speed we need instrumentation for debugging and
performance measurement. For example, it should be possible to disable caching
to verify that the results don’t change.

<b>Not a goal</b> for the initial implementation: automatic determination or
validation of dependency lists. Instead we rely on a conservative default
(depends on everything), and the ability to disable caching during testing.
Several follow-ons are possible to improve this:
 - Runtime validation that computations only request sub-computations for which
   they hold tickets (probably affordable only in Debug), and
 - Use of symbolic expressions to calculate dependency lists automatically.

@anchor cache_design_architecture
<h2>Architecture</h2>

This is the basic architecture Drake uses to address the above design goals.

Every declared source and computed value is assigned a small-integer
DependencyTicket (“ticket”) by the System (unique within a subsystem). The
Context contains corresponding DependencyTracker ("tracker") objects that manage
that value’s downstream dependents and upstream prerequisites; these can be
accessed efficiently using the ticket. Cached computations are declared and
accessed through System-owned CacheEntry objects; their values are stored in
Context-owned CacheEntryValue objects whose validity with respect to
source values is rigorously tracked via their associated dependency
trackers. Computed values may in turn serve as
source values for further computations. A change to a source value invalidates
any computed values that depend on it, recursively. Any value contained in a
Context may be exported via an output port that exposes that value to downstream
Systems; inter-subsystem dependencies are tracked using the same mechanism as
intra-subsystem dependencies.

From a user perspective:
 - Cache entries of arbitrary type are allocated to hold the results of all
   significant computations, including built-ins like derivatives, discrete
   updates, and output ports as well as user-defined internal computations.
   Prerequisites for these computations are explicitly noted at the time they
   are declared; the default is dependency on all sources.
 - Computation is initiated when a result is requested via an “Eval” method, if
   the result is not already up to date with respect to its prerequisites, which
   are recursively obtained using their Eval methods.
 - Cached results are automatically invalidated when any of their prerequisites
   change.

Figure 1 below illustrates the computational structure of a Drake LeafSystem,
paired with its LeafContext. A Drake Diagram interconnects subsystems like
these by connecting output ports of subsystems to input ports of other
subsystems, and aggregates results such as derivative calculations. When
referring to individual Systems within a diagram, we use the terms
“subsystem” and “subcontext”. */

// Can't put the following in doxygen comments because the "-->" arrows
// terminate them!

/* Looks something like this ...

                       drake::systems::LeafSystem
                ┌─────────────────────────────────────┐
    time ------>│>------.          ┌───────┐    ···---│--->  derivatives,
                │       `--------->│cache  │----------│--->  updates, etc.
   input    --->│>---------------->│entry  │          │
   ports u  --->│                  └───────┘          │──────┐
           ┌--->│                  ┌───────┐    ···---│---------->  output
           |    │ (Parameters)---->│cache  │----------│---------->  ports y
           |    │ (State)--------->│entry  │    ···---│---------->
           |    │                  └───────┘          │      │
           |    └─────────────────────────────────────┘      │
           |           │                                     │
           └<----------│-(Fixed input)                       │
                       │                                     │
                       └─────────────────────────────────────┘
                             drake::systems::LeafContext
*/

/** @addtogroup cache_design_notes  <!-- continuing on -->
@image html systems/framework/images/system_context_cache.png "Figure 1: Computational structure of a Drake System."

In Figure 1 above, values are shown in gray like the Context to emphasize
that they are actually being stored in the Context. The System can declare the
structure (colored borders), but does not contain the actual values.There can
also be values in the Context of which the System is unaware, including the
Fixed Input values as shown, and additional cached computations. Computed values
depend on source values, but are shown rounded on one side to emphasize that
they then become sources to downstream computations. The arrows as drawn should
be considered “is-prerequisite-of” edges; drawn in the reverse direction they
could be labeled “depends-on” edges.

@anchor cache_design_value_sources
<h2>Value sources</h2>

When a cache entry is allocated, a list of prerequisite value sources is
provided by listing the dependency tickets for those sources. Only
_intra_-System dependencies are permitted; _inter_-System dependencies are
expressed by listing an input port as a prerequisite. There are six kinds of
value sources within a System’s Context that can serve as prerequisites for
cached results within that Context:
 1. Time
 2. Input ports
 3. Parameters (numerical and abstract)
 4. State (including continuous, discrete, and abstract variables)
 5. Other cache entries
 6. Accuracy

In addition, we support "fixed" input ports whose values are provided locally;
each such value is a value source, but is restricted to its corresponding
input port. Fixed values are semantically like additional Parameters.

The Accuracy setting serves as a prerequisite for cached computations that are
computed approximately, to make sure they get recomputed if accuracy
requirements change. That is a technical detail not of interest to most users.
Its dependencies are handled similarly to time dependencies.

Each value source has a unique DependencyTicket that is used to locate its
DependencyTracker in the Context. Further granularity is provided for individual
value sources from the above categories. For example, configuration variables q
and velocity variables v have separate tickets. The ticket is used to designate
that source as a prerequisite for a cache entry. Tickets are assigned during
System construction, whenever a component is specified that can serve as a value
source. Each value source’s DependencyTracker maintains a list of its dependents
(called “subscribers”) so that it can perform invalidation at run time, and is
registered with its prerequisites so that it can be properly invalidated.

@anchor cache_design_output_ports
<h2>Output ports</h2>

An Output Port for a System is a “window” onto one of the value
sources within that System's Context; that is the only way in which internal
values of a System are exported to other Systems in a Diagram. That value source
may be
 - a cache entry allocated specifically for the output port, or
 - a pre-existing source like a state subgroup or a cached value that has other
   uses, or
 - an output port of a contained subsystem that has been exported.

An output port is a subscriber to its source, and a prerequisite to the
downstream input ports that it drives.

Every output port has an associated dependency ticket and tracker. If there is
a cache entry associated with the output port, it has its own ticket and tracker
to which the output port’s tracker subscribes. A Diagram output port that is
exported from a contained subsystem still has its own tracker and subscribes
to the source output port’s tracker.

@anchor cache_design_input_ports
<h2>Input ports</h2>

The value source for a subsystem input port is either
 - an output port of a peer subsystem, or
 - an input port of its parent Diagram, or
 - a locally-stored value.

When an input port’s value comes from a locally-stored value, we call it a
fixed input port. Note that the fixed value is stored as a source value in the
Context. There is no corresponding entry in the System since providing values
for input ports is done exclusively via the Context (see Figure 1 above).

Fixed input ports are treated identically to Parameters -- they may have
numerical or abstract value types; they may be changed with downstream
invalidation handled automatically; and their values do not change during a
time-advancing simulation. The values for fixed input ports are represented by
FixedInputPortValue objects, which have their own ticket and tracker to
which the corresponding input port subscribes.

Every input port has an associated dependency ticket and tracker. The tracker
is automatically subscribed to the input port’s source’s tracker.

@anchor cache_design_known_computations
<h2>Known computations</h2>

Certain computations are defined by the System framework so are automatically
assigned cache entries in the Context. Currently those are:
 - Leaf output ports
 - Time derivatives
 - Discrete state updates
 - Power and energy (scalars)

Output ports that have their own Calc() methods are also automatically assigned
cache entries. Currently that applies to every leaf output port, but not to
diagram output ports.

@anchor cache_design_declaring
<h2>Declaring a cache entry</h2>

The API for declaring a cache entry is similar to the existing output port API.
A cache entry is defined by an Allocator() method returning an AbstractValue,
and a Calculator() method that takes a const (sub)Context and an AbstractValue
object of the type returned by the Allocator, and computes the correct value
given the Context, and a list of prerequisites for the Calculator() function,
which are DependencyTickets for value sources in the same subcontext.

In the absence of explicit prerequisites, a cache entry is implicitly presumed
to be dependent on all possible values sources, so will be invalidated whenever
accuracy, time, any input port, parameter, or state variable changes value. No
implicit dependency on other cache entries is assumed.

A typical declaration looks like this (in the constructor for a LeafSystem):
@code{.cpp}
  const CacheEntry& pe_cache_entry =
      DeclareCacheEntry("potential energy", 0.0,
                        &MySystem::CalcPotentialEnergy,
                        {all_parameters_ticket(), configuration_ticket()});
@endcode

That is a templatized “sugar” method where the allocator has been specified to
simply copy the given default value. The usual variants are available, as for
output port declarations. The new CacheEntry object’s CacheIndex and
DependencyTicket can be obtained from the entry if needed. The signature of the
most-general method is:
@code{.cpp}
  const CacheEntry& DeclareCacheEntry(
      std::string                   description,
      CacheEntry::AllocCallback     alloc_function,
      CacheEntry::CalcCallback      calc_function,
      std::vector<DependencyTicket> prerequisites_of_calc_function);
@endcode

If no dependencies are listed the default is `{all_sources_ticket()}`. A cache
entry that is truly independent of all sources must explicitly say so by
specifying `{nothing_ticket()}`.

@anchor cache_design_implementation
<h2>Implementation</h2>

The logical cache entry object is split into two pieces to reflect the const
and mutable aspects. Given a CacheIndex, either piece may be obtained very
efficiently. The `const` part is a CacheEntry owned by the System, and consists
of:
 - The Allocator() and Calculator() methods,
 - a list of the Calculator’s prerequisites, and
 - the assigned dependency ticket for this cache entry.

The mutable part is a CacheEntryValue and associated DependencyTracker, both
owned by the Context. The CacheEntryValue is designed to optimize access
efficiency. It contains:
 - an AbstractValue (obtained by invoking the Allocator()), and
 - a flag indicating whether the value is up to date with respect to its
   prerequisites, and
 - a serial number that is incremented whenever the contained value changes, and
 - the index to its DependencyTracker (i.e. the ticket).

The “up to date” boolean is set (validated) when a new cache entry value is
assigned; it is reset (invalidated) by the associated DependencyTracker as a
result of a prerequisite change. For debugging and performance analysis, there
is also a flag indicating whether caching has been disabled for this entry, in
which case the value is considered in need of recomputation every time it
is accessed.

The Eval() method for a CacheEntry operates as follows:
 1. The CacheEntryValue is obtained using an array index into the Context’s
    cache.
 2. If the value is not up to date (or caching is disabled), call the
    Calculator() method to bring it up to date, and set the “up to date”
    boolean to true.
 3. Return a reference to the cache entry’s AbstractValue.

The DependencyTracker is designed to optimize invalidation efficiency.
It contains:
 - Pointers to the prerequisite DependencyTrackers to which it has subscribed,
 - pointers to downstream subscribers that have registered as dependents of
   this cache entry,
 - a pointer back to the CacheEntryValue for efficient invalidation, and
 - bookkeeping and statistics-gathering members.

A prerequisite change to a particular value source works like this:
 1. A “set” method or method providing mutable access to a source is invoked
    on a Context.
 2. A unique “change event” number N is assigned.
 3. The ticket associated with the source is used to find its DependencyTracker
    (just an array index operation).
 4. The tracker’s notification method is invoked, providing the change event
    number N.
 5. The tracker records N, and then notifies all trackers on its “subscribers”
    list that change event N is occurring.
 6. A notified tracker first checks its recorded change event number. If it is
    already set to N then no further action is taken. Otherwise, it records N,
    invalidates the associated cache entry (if any) and recursively notifies all
    trackers on its “subscribers” list.

For efficiency, changes to multiple value sources can be grouped into the same
change event. Also, DependencyTracker subscriber lists and cache entry
references are just direct pointers (internal to the overall diagram Context)
so no lookups are required. This is very fast but makes cloning a Context more
difficult because the pointers must be fixed up to point to corresponding
entities in the clone.

@anchor cache_design_loose_ends
<h2>Notes & Loose Ends</h2>

<h3>No cross pointers</h3>
It is important to emphasize that a Context never contains pointers to a
particular System. Thus the allocator and calculator functors for a cache entry
reside in the (sub)System, not in the (sub)Context. That implies that you
cannot evaluate a cache entry without both a System and a Context. It also means
you can use the same Context with different Systems, and that a Context could
be serialized then deserialized to use later, provided a compatible System
is available.

<h3>System vs Context</h3>

Drake’s System framework provides a careful separation between the persistent
structure of a System and its ephemeral values (Context). The structure is
represented in const objects derived from SystemBase, while the values are
represented in mutable objects derived from ContextBase. Generally that means
that what are logically single objects are split into two separate classes.
Since users are intended to interact primarily with System objects, we use
the logical names for the System half of the object, and more obscure names
for the Context half that stores the runtime values. Here are some examples:

  Logical object   |  System-side class  | Context-side class
:------------------|:--------------------|:------------------
System             | System              | Context
Output port        | OutputPort          | (cache entry)
Input port         | InputPortDescriptor | FixedInputPortValue
Cache entry        | CacheEntry          | CacheEntryValue
Dependency tracker | (ticket only)       | DependencyTracker

(The names are as they exist now; some clearly need changing!)

The System-side objects are used for declaring the computational structure of
a System; the Context-side objects are used to maintain correct current values.
For example, the necessary allocation and calculation methods for a cache entry
are stored in the System-side CacheEntry object in the System, while the result
of executing those methods is stored in the corresponding CacheEntryValue object
in the Context.

<h3>Output Port cache entries</h3>

Each port and each cache entry is assigned a unique DependencyTracker object.
Output ports generally also have their own cache entries for storing their
values; that results in two DependencyTrackers -- the output port
DependencyTracker lists its cache entry’s DependencyTracker as a prerequisite.
(TODO: In the initial implementation there will always be a cache entry
assigned for leaf output ports, but this design is intended to allow output
ports to forward from other sources without making unnecessary copies -- an
output port DependencyTracker is still needed in those cases even though there
won’t be an associated cache entry.)

<h3>Declaring dependencies for built-in computations</h3>

The built-in computations like time derivatives and discrete updates are
associated with bespoke “Calc” methods which need to have known dependency
lists. Rather than add these with yet more System methods, we should consider
changing the API to define them analogously to the OutputPort and CacheEntry
declarations which accept Allocator() and Calculator() functors and a
dependency list for the Calculator(). Currently they are just defaulting to
“depends on everything”.
*/

}  // namespace systems
}  // namespace drake
