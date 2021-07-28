/** @file
 Doxygen-only documentation for @ref cache_design_notes.  */

#pragma once

// Putting this document in drake::systems namespace makes the links work.
namespace drake {
namespace systems {

/** @defgroup cache_design_notes System Cache Design and Implementation Notes
    @ingroup technical_notes

<!-- Fluff needed to keep Doxygen from misformatting due to quotes and
     this being in the "autobrief" location. -->
@parblock                       <center>
          \"There are only two hard things in computer science:<br>
          <b>cache invalidation</b>, and naming things.\"<br>
                            -- Phil Karlton
                                </center>
@endparblock

<h2>Background</h2>
Drake System objects are used to specify the computational _structure_ of a
model to be studied. The actual _values_ during computation are stored in a
separate Context object. The %Context contains _source_ values (time,
parameters, states, input ports, accuracy) and _computed_ values (e.g.
derivatives, output ports) that depend on some or all of the source values. We
call the particular dependencies of a computed value its _prerequisites_. The
caching system described here manages computed values so that

- they are recomputed _only if_ a prerequisite has changed,
- they are marked out of date _whenever_ a prerequisite changes, and
- _every_ access to a computed value first ensures that it is up to date.

Accessing computed values is a critical, inner-loop activity during
simulation (many times per step) so the implementation is designed to provide
validity-checked access at minimal computational cost. Marking computations
out of date as prerequisites change is also frequent (at least once per step),
and potentially expensive, so must be efficient; the implementation goes to
great lengths to minimize the cost of that operation.

Caching is enabled by default in Drake. Methods are provided for disabling and
otherwise manipulating cache entries. See ContextBase for more information.

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
to verify that the results don't change.

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
DependencyTicket ("ticket") by the System (unique within a subsystem). The
Context contains corresponding DependencyTracker ("tracker") objects that manage
that value's downstream dependents and upstream prerequisites; these can be
accessed efficiently using the ticket. Cached computations are declared and
accessed through System-owned CacheEntry objects; their values are stored in
Context-owned CacheEntryValue objects whose validity with respect to source
values is tracked via their associated dependency trackers. Computed
values may in turn serve as source values for further computations. A change to
a source value invalidates any computed values that depend on it, recursively.
Such changes are initiated via %Context methods that send "out of date"
notifications to all downstream dependency trackers, which set the "out of date"
flag on cache entry values.

Any value contained in a Context may be exported via an output port that exposes
that value to downstream Systems; inter-subsystem dependencies are tracked using
the same mechanism as intra-subsystem dependencies. The current implementation
assigns a cache entry to each output port and requires copying %Context values
to that cache entry in order to expose them. This may be improved later to allow
values to be exposed directly without the intermediate cache entry, but the
semantics will be unchanged.

From a user perspective:

- Cache entries of arbitrary type are allocated to hold the results of all
  significant computations, including built-ins like derivatives, energy,
  and output ports as well as user-defined internal computations.
  Prerequisites for these computations are explicitly noted at the time they
  are declared; the default for user-defined cache entries is that they are
  dependent on all sources.
- Computation is initiated when a result is requested via an "Eval" method, if
  the result is not already up to date with respect to its prerequisites, which
  are recursively obtained using their Eval methods.
- Cached results are automatically marked out of date when any of their
  prerequisites may have changed.

Figure 1 below illustrates the computational structure of a Drake LeafSystem,
paired with its LeafContext. A Drake Diagram interconnects subsystems like
these by connecting output ports of subsystems to input ports of other
subsystems, and aggregates results such as derivative calculations. When
referring to individual Systems within a diagram, we use the terms
"subsystem" and "subcontext".
<!--
                       drake::systems::LeafSystem
                ┌─────────────────────────────────────┐
    time ──────>│>──────┐          ┌───────┐    ···───│───>  derivatives,
                │       └─────────>│cache  │──────────│───>  energy, etc.
   input    ───>│>────────────────>│entry  │          │
   ports u  ───>│                  └───────┘          │──────┐
           ┌───>│                  ┌───────┐    ···───│──────────>  output
           │    │ (Parameters p)──>│cache  │──────────│──────────>  ports y
           │    │ (State x)───────>│entry  │    ···───│──────────>
           │    │                  └───────┘          │      │
           │    └─────────────────────────────────────┘      │
           │           │                                     │
           └<──────────│─(Fixed input)                       │
                       │                                     │
                       └─────────────────────────────────────┘
                             drake::systems::LeafContext
-->

@image html drake/systems/framework/images/system_context_cache.png "Figure 1: Computational structure of a Drake System."

In Figure 1 above, values are shown in gray like the Context to emphasize
that they are actually being stored in the %Context. The System can declare the
structure (colored borders), but does not contain the actual values. There can
also be values in the %Context of which the %System is unaware, including the
Fixed Input values as shown, and additional cached computations. Computed values
depend on source values, but are shown rounded on one side to emphasize that
they then become sources to downstream computations. The arrows as drawn should
be considered "is-prerequisite-of" edges; drawn in the reverse direction they
could be labeled "depends-on" edges.

@anchor cache_design_value_sources
<h2>%Value sources</h2>

When a cache entry is allocated, a list of prerequisite value sources is
provided by listing the dependency tickets for those sources. Only
_intra_-System dependencies are permitted; _inter_-System dependencies are
expressed by listing an input port as a prerequisite. There are six kinds of
value sources within a System's Context that can serve as prerequisites for
cached results within that %Context:

1. Time
2. Input ports
3. %Parameters (numerical and abstract)
4. %State (including continuous, discrete, and abstract variables)
5. Other cache entries
6. Accuracy

In addition, we support "fixed" input ports whose values are provided locally;
each such value is a value source, but is restricted to its corresponding
input port. Fixed values are semantically like additional %Parameters.

The Accuracy setting serves as a prerequisite for cached computations that are
computed approximately, to make sure they get recomputed if accuracy
requirements change. That is a technical detail not of interest to most users.
Its dependencies are handled similarly to time dependencies.

Each value source has a unique DependencyTicket that is used to locate its
DependencyTracker in the Context. Further granularity is provided for individual
value sources from the above categories. For example, configuration variables q
and velocity variables v have separate tickets. The ticket is used to designate
that source as a prerequisite for a cache entry. Tickets are assigned during
%System construction, whenever a component is specified that can serve as a
value source. Each value source's %DependencyTracker maintains a list of its
dependents (called "subscribers") so that it can perform "out of date"
notifications at run time, and is registered with its prerequisites so that it
can be properly notified when it may be invalid.

@anchor cache_design_output_ports
<h2>Output ports</h2>

An Output Port for a System is a "window" onto one of the value sources within
that %System's Context; that is the only way in which internal values of a
%System are exported to other Systems in a Diagram. That value source may be

- a cache entry allocated specifically for the output port, or
- a pre-existing source like a state subgroup or a cached value that has other
  uses (not implemented yet), or
- an output port of a contained subsystem that has been exported.

An output port is a subscriber to its source, and a prerequisite to the
downstream input ports that it drives.

Every output port has an associated dependency ticket and tracker. If there is
a cache entry associated with the output port, it has its own ticket and tracker
to which the output port's tracker subscribes. A Diagram output port that is
exported from a contained subsystem still has its own tracker and subscribes
to the source output port's tracker.

@anchor cache_design_input_ports
<h2>Input ports</h2>

The value source for a subsystem input port is either

- an output port of a peer subsystem, or
- an input port of its parent Diagram, or
- a locally-stored value.

When an input port's value comes from a locally-stored value, we call it a
_fixed_ input port. Note that the fixed value is stored as a source value in the
Context. There is no corresponding entry in the System since providing values
for input ports is done exclusively via the %Context (see Figure 1 above).

Fixed input ports are treated identically to Parameters -- they may have
numerical or abstract value types; they may be changed with downstream
invalidation handled automatically; and their values do not change during a
time-advancing simulation. The values for fixed input ports are represented by
FixedInputPortValue objects, which have their own ticket and tracker to
which the corresponding input port subscribes.

Every input port has an associated dependency ticket and tracker. The tracker
is automatically subscribed to the input port's source's tracker.

@anchor cache_design_known_computations
<h2>Known computations</h2>

Certain computations are defined by the system framework so are automatically
assigned cache entries in the Context. Currently those are:

- Leaf output ports
- Time derivatives
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
given the %Context, and a list of prerequisites for the Calculator() function,
which are DependencyTickets for value sources in the same subcontext.

In the absence of explicit prerequisites, a cache entry is implicitly presumed
to be dependent on all possible values sources, so will be marked "out of date"
whenever accuracy, time, any input port, parameter, or state variable may have
a changed value. However, no implicit dependency on other cache entries
is assumed.

A typical declaration looks like this (in the constructor for a LeafSystem):
@code{.cpp}
  const CacheEntry& pe_cache_entry =
      DeclareCacheEntry("potential energy", 0.0,
                        &MySystem::CalcPotentialEnergy,
                        {all_parameters_ticket(), q_ticket()});
@endcode

That is a templatized "sugar" method where the allocator has been specified to
simply copy the given default value. The usual variants are available, as for
output port declarations.
See @ref DeclareCacheEntry_documentation "Declare cache entries" in SystemBase
for the full collection of variants.

If no dependencies are listed in a cache entry declaration, the default is
`{all_sources_ticket()}`. A cache entry that is truly independent of all sources
must explicitly say so by specifying `{nothing_ticket()}`. The available tickets
are defined in the SystemBase class; see
@ref DependencyTicket_documentation "Dependency tickets" there. Once declared,
the new CacheEntry object's CacheIndex and DependencyTicket can be obtained from
the entry if needed.

@anchor predefined_dependency_tickets
<h2>Predefined dependency tickets</h2>

The following dependency tickets are always available and are used to select
particular built-in dependency trackers. During System construction, an API user
can obtain the ticket number as the return value of methods
like `time_ticket()` that are provided by SystemBase. Add `_ticket()` to
the names in the first column of the table below to obtain the name of the
method to use in a prerequisite list as shown in the previous section. Ticket
methods followed by `(i)` in the table below are assigned as the indicated
resource is allocated; they do not have pre-assigned ticket numbers.

 Ticket name         |Abbr|  Prerequisite indicated  |  Subscribes to
:--------------------|:--:|:-------------------------|:------------------
nothing              |    | has no prerequisite      | —
time                 | t  | simulated time           | —
accuracy             | a  | accuracy setting         | —
q                    |    | continuous configuration | — ¹
v                    |    | continuous velocity      | — ¹
z                    |    | misc. continuous state   | — ¹
xc                   |    | any continuous state     | q v z
xd                   |    | any discrete state       | xdᵢ ∀i ¹ ²
xa                   |    | any abstract state       | xaᵢ ∀i ¹ ²
all_state            | x  | any state variable       | xc xd xa
pn                   |    | any numeric parameter    | pnᵢ ∀i ¹ ²
pa                   |    | any abstract parameter   | paᵢ ∀i ¹ ²
all_parameters       | p  | any parameter p          | pn pa
all_input_ports      | u  | any input port u         | uᵢ ∀i
all_sources          |    | any change to %Context   | t, a, x, p, u
configuration        |    | may affect pose or PE    | q, z, a, p, xd, xa ³
kinematics           |    | may affect pose/motion   | configuration, v ³
xcdot                |    | d/dt xc cached value     | all_sources ¹ ⁵
pe                   |    | potential energy         | all_sources ¹ ⁵
ke                   |    | kinetic energy           | all_sources ¹ ⁵
pc                   |    | conservative power       | all_sources ¹ ⁵
pnc                  |    | non-conservative power   | all_sources ¹ ⁵
numeric_parameter(i) |pnᵢ | one numeric parameter    | — ²
abstract_parameter(i)|paᵢ | one abstract parameter   | — ²
discrete_state(i)    |xdᵢ | one discrete state group | — ²
abstract_state(i)    |xaᵢ | one abstract state       | — ²
input_port(i)        | uᵢ | one input port           | peer, parent, or self ⁴
cache_entry(i)       | cᵢ | one cache entry          | explicit prerequisites

_Notes_

1. %Diagram has additional subscriptions for this tracker. See the
   diagram-specific table in the
   @ref caching_implementation_for_diagrams "Diagram-specific implementation"
   section below.
2. There are no Diagram-level trackers for individual discrete/abstract
   variables and numeric/abstract parameters.
3. Until issue [#9171](https://github.com/RobotLocomotion/drake/issues/9171)
   is resolved we don't know which non-`v` state variables or which parameters
   may affect kinematics, so we have to depend on all of them.
4. Input ports are dependent on the source that provides their values. That
   may be the output port of a peer subsystem, the input port of the parent
   diagram, or a locally-stored fixed input port value.
5. %Diagram currently subscribes to all_sources for this tracker, but in the
   future it will only subscribe to the corresponding leaf trackers and no
   longer subscribe to the Diagram's all_sources tracker.  The leaf trackers
   are sufficient on their own; the all_sources tracker is redundant.

Fixed input port values and output ports also have associated trackers. There
are methods for obtaining their tickets also but they are for internal use.
Only input ports may subscribe to those trackers, and that is handled by the
framework when the source for an input port is established.

@anchor cache_handling_composite_trackers
<h3>Handling of composite trackers</h3>

In the above table, entries that don't subscribe to anything ("—") are primary
source objects. For example, q is an independent state variable so its tracker
doesn't depend on anything else. Trackers like xc are composites, meaning they
are shorthand for a group of independent objects. xc is a tracker that
subscribes to the trackers for q, v, and z. Similarly xd stands for _all_
discrete state variables, and subscribes to each of the individual discrete
state trackers. That way if a q is modified, xc gets notified automatically.
Similarly a change to a single discrete state variable notifies xd. Once those
subscriptions are set up, no additional code is required in Drake to propagate
the notifications. Let's call this the "up" direction, where a low-level entity
notifies its "parent" composite entity.

More subtly, we also have to issue notifications in the "down" direction. That's
because the Context provides methods like SetContinuousState() and
get_mutable_state() which effectively change a group of source objects. That
_could_ be handled with more subscriptions, at the cost of introducing cycles
into the dependency DAG (the "change event" would serve to break those cycles
during notification sweeps). Instead, since we always know the constituents at
the time a high-level modification request is made, we simply have bespoke code
that notifies the lowest-level constituents, then depend on the "up" direction
subscriptions to get the composite trackers notified. See the
@ref context_base_change_notification_methods "notifications methods" in
ContextBase.

There are additional considerations for Diagram notifications; those are
discussed in the next section.

@anchor caching_implementation_for_diagrams
<h2>Diagram-specific implementation</h2>

Diagrams have some implementation details that don't apply to leaf systems.
Diagrams do not have their own state variables and parameters. Instead, their
states and parameters are references to their child subsystems' corresponding
states and parameters. This applies individually to each state and parameter
category. Diagram contexts do have trackers for their composite state and
parameters, and we must subscribe those to the corresponding child subsystem
trackers to ensure notifications propagate upward. Using capital letters to
denote the %Diagram composite trackers we have:

- Q  = { qₛ : s ∈ 𝕊}
- V  = { vₛ : s ∈ 𝕊}
- Z  = { zₛ : s ∈ 𝕊}
- Xd = {xdₛ : s ∈ 𝕊}
- Xa = {xaₛ : s ∈ 𝕊}
- Pn = {pnₛ : s ∈ 𝕊}
- Pa = {paₛ : s ∈ 𝕊}

where 𝕊 is the set of immediate child subsystems of a %Diagram. Note that the
higher-level built-in trackers (all_state, all_parameters, configuration, etc.)
do not require special treatment. They simply subscribe to some or all of the
diagram source trackers listed above. They may also subscribe to time, accuracy,
and diagram-level input ports, none of which have dependencies on child
subsystems.

In addition to composite sources, Diagrams have a limited set of built-in
computations that are composites of their children's corresponding computations.
These are

- xcdot: composite time derivatives
- pe: summed potential energy
- ke: summed kinetic energy
- pc: summed conservative power
- pnc: summed non-conservative power

Each of these has a cache entry and an associated Calc() method that sets the
cache value by visiting the children to Eval() the corresponding quantities,
which are then combined to produce the composite quantity. xcdot is just a
container for the collected child quantities so just needs to ensure these are
up to date. Energy and power are scalars formed by summing the corresponding
scalar quantities from each child subsystem. To ensure that the diagram cache
entries are updated appropriately, they subscribe to the corresponding child
cache entries' dependency trackers. That way if a leaf quantity gets notified
of a leaf context change, any composite diagram quantity that needs that
leaf quantity is automatically notified also.

Modifications to state variables and parameters are always initiated through one
of the mutable methods of the Context class, but not necessarily at the root
DiagramContext of a %Context tree. Wherever a modification is initiated, we
must propagate that change in both directions: down to the child subsystems,
and up to the parent diagram. If someone changes the root diagram Q, that is
a change to each of its constituents qₛ, so we need to notify "down" to the
trackers for each of those, to make sure that subsystem s computations that
depend on qₛ will be properly marked "out of date". In the "up" direction, a
user who has access to one of the child subcontexts may initiate a change to a
qₛ; that should be propagated to the diagram Q since some part of it
has changed.

We implement the two directions asymmetrically. The "up" direction is
implemented simply by subscribing every diagram-level tracker to each of
its constituent object's trackers. That way any change to a constituent
notifies its parent via the usual subscriber notification code. Other than the
code to set up the subscriptions properly, there is no explicit code needed to
handle the "up" notifications.
See `%DiagramContext::SubscribeDiagramCompositeTrackersToChildrens()` for the
code that sets up these subscriptions. The following table shows the
Diagram-specific notifications and subscriptions:

  Diagram tracker    | Notifications sent ("down") |  Subscribes to ("up")
:--------------------|:----------------------------|:---------------------
time                 | subsystem time              | — ¹
accuracy             | subsystem accuracy          | — ¹
q                    | subsystem q                 | subsystem q
v                    | subsystem v                 | subsystem v
z                    | subsystem z                 | subsystem z
xd                   | subsystem xd                | subsystem xd
xa                   | subsystem xa                | subsystem xa
pn                   | subsystem pn                | subsystem pn
pa                   | subsystem pa                | subsystem pa
xcdot                | —                           | subsystem xcdot
pe                   | —                           | subsystem pe
ke                   | —                           | subsystem ke
pc                   | —                           | subsystem pc
pnc                  | —                           | subsystem pnc

_Notes_

1. Time and accuracy can be set only in the root Context so can only
   propagate downward.

The "down" direction _could_ have been handled via subscriptions also, at the
cost of introducing cycles into the dependency DAG (the "change event" would
serve to break those cycles during notification sweeps). Instead, the "down"
direction is implemented via bespoke code for the
@ref context_value_change_methods "Context modification methods"
that recursively visits each subcontext and executes the same modification
and out of date notifications there. That is similar to the treatment of local
composite trackers as discussed in
@ref cache_handling_composite_trackers "Handling of composite trackers"
above.

Note that a lower-level subcontext may also have fixed input port values, and
that a Diagram is completely unaware of any subsystem input ports whose values
have been fixed locally. When a subcontext fixed input port value changes, all
downstream computations are properly notified. For example, a subcontext
time derivative cache entry might depend on the fixed input port. As long as
the composite %Diagram time derivatives cache entry has subscribed to its
subsystems' time derivative cache entries, that change will also mark the
%Diagram time derivative out of date.

@anchor cache_trackers_figure
<h2>Tracker Subscription Summary</h2>

The figure below depicts the tracker subscriptions detailed in the above sections.
The arrow direction reads as "subscribes to (up)", i.e., "depends on":

- Folder-shaped nodes denote a Diagram tracker.
- Oval-shaped nodes denote a System tracker (the same for both LeafSystem or
  Diagram).
- \b Black arrows are per the "Subscribes to" column in the "Predefined
  dependency tickets" table.
  - <span style="color:green">\b Green</span> arrows are per the input_port(i)
    row in the same table.  Unlike black arrows which are always subscribed,
    the green arrows are only subscribed if that particular input port source
    is in effect.
- <span style="color:blue">\b Blue</span> arrows are per the "Subscribes to"
  column in the "Diagram-specific implementation" table.
- <span style="color:red">\b Red</span> arrows are the implicit subscriptions
  implemented as "Context modification methods", per the "Notifications sent"
  column in the "Diagram-specific implementation" table.  When the diagram
  quantity at the arrowhead is modified, the leaf quantities at the tail are
  invalidated.  The dotted line style denotes an implicit subscription, not
  part of the tracker.
- <span style="border-bottom: 1px dashed">\b Dashed</span> lines indicate an
  overly conservative subscription, planned to be made more precise in the
  future.  See the specific text in the prior sections for details.
- Doubled-line edges indicate that the item has a multiplicity of 0..many (∀i).

@dotfile systems/framework/images/cache_doxygen_trackers.dot

@anchor cache_design_implementation
<h2>Implementation</h2>

The logical cache entry object is split into two pieces to reflect the const
and mutable aspects. Given a CacheIndex, either piece may be obtained very
efficiently. The `const` part is a CacheEntry owned by the System, and consists
of:

 - The Allocator() and Calculator() methods,
 - a list of the Calculator's prerequisites, and
 - the assigned dependency ticket for this cache entry.

The mutable part is a CacheEntryValue and associated DependencyTracker, both
owned by the Context. The CacheEntryValue is designed to optimize access
efficiency. It contains:

 - an AbstractValue (obtained by invoking the Allocator()), and
 - a flag indicating whether the value is out of date with respect to its
   prerequisites, and
 - a serial number that is incremented whenever the contained value changes, and
 - the index to its DependencyTracker (i.e. the ticket).

When a prerequisite of a cache entry value changes, the associated
DependencyTracker sets its CacheEntryValue's "out of date" boolean `true`. When
a new value is computed and assigned the "out of date" boolean is cleared to
`false` by the code that performed the assignment. For debugging and performance
analysis, there is also a flag indicating whether caching has been disabled for
this entry, in which case the value is considered in need of recomputation every
time it is accessed. The out of date flag operates even when caching is disabled
but is ignored in that case by Eval() methods. The sense of these flags is
chosen so that only a single check that an int is zero is required to know that
a value may be returned without computation.

To summarize, the Eval() method for a CacheEntry operates as follows:

 1. The CacheEntryValue is obtained using an array index into the Context's
    cache.
 2. If the value is out of date (or caching is disabled), call the
    Calc() method to bring it up to date, and clear the "out of date"
    boolean to false.
 3. Return a reference to the cache entry's AbstractValue.
 4. Downcast to the specified concrete type.

The DependencyTracker is designed to optimize invalidation efficiency.
It contains:

 - Pointers to the prerequisite DependencyTrackers to which it has subscribed,
 - pointers to downstream subscribers that have registered as dependents of
   this cache entry,
 - a pointer back to the CacheEntryValue for efficient invalidation, and
 - bookkeeping and statistics-gathering members.

A prerequisite change to a particular value source works like this:

1. A "set" method or method providing mutable access to a source is invoked
   on a Context.
2. A unique "change event" number N is assigned.
3. The ticket associated with the source is used to find its DependencyTracker
   (just an array index operation).
4. The tracker's notification method is invoked, providing the change event
   number N.
5. The tracker records N, and then notifies all trackers on its "subscribers"
   list that change event N is occurring.
6. A notified tracker first checks its recorded change event number. If it is
   already set to N then no further action is taken. Otherwise, it records N,
   marks the associated cache entry (if any) out of date, and recursively
   notifies all trackers on its "subscribers" list.

For efficiency, changes to multiple value sources can be grouped into the same
change event. Also, DependencyTracker subscriber lists and cache entry
references are just direct pointers (internal to the overall diagram Context)
so no lookups are required. This is very fast but makes cloning a %Context more
difficult because the pointers must be fixed up to point to corresponding
entities in the clone.

(Note that the "down" direction for Diagram notifications (as described above)
currently uses bespoke recursive code to notify all the subsystems. It would
likely be faster to use the subscription mechanism there also.)

@anchor cache_design_loose_ends
<h2>Notes & Loose Ends</h2>

<h3>No cross pointers</h3>
It is important to emphasize that a Context _never_ contains pointers to a
particular System. Thus the allocator and calculator functors for a cache entry
reside in the (sub)%System, not in the (sub)%Context. That implies that you
cannot evaluate a cache entry without both a %System and a %Context. It also
means you can use the same %Context with different Systems, and that a %Context
could be serialized then deserialized to use later, provided that a compatible
%System is available.

<h3>%System vs %Context</h3>

Drake's System framework provides a careful separation between the persistent
structure of a %System and its ephemeral values (Context). The structure is
represented in const objects derived from SystemBase, while the values are
represented in mutable objects derived from ContextBase. Generally that means
that what are logically single objects are split into two separate classes.
Since users are intended to interact primarily with %System objects, we use
the logical names for the %System half of the object, and more obscure names
for the %Context half that stores the runtime values. Here are some examples:

  Logical object   |  System-side class  | Context-side class
:------------------|:--------------------|:------------------
%System            | System              | Context
Output port        | OutputPort          | (cache entry)
Input port         | InputPort           | FixedInputPortValue
%Cache entry       | CacheEntry          | CacheEntryValue
Dependency tracker | (ticket only)       | DependencyTracker

The System-side objects are used for declaring the computational structure of
a System; the Context-side objects are used to maintain correct current values.
For example, the necessary allocation and calculation methods for a cache entry
are stored in the System-side CacheEntry object in the %System, while the result
of executing those methods is stored in the corresponding CacheEntryValue object
in the Context.

<h3>Output Port cache entries</h3>

Each port and each cache entry is assigned a unique DependencyTracker object.
Output ports generally also have their own cache entries for storing their
values; that results in two DependencyTrackers -- the output port
%DependencyTracker lists its cache entry's %DependencyTracker as a prerequisite.
(TODO: In the initial implementation there will always be a cache entry
assigned for leaf output ports, but this design is intended to allow output
ports to forward from other sources without making unnecessary copies -- an
output port %DependencyTracker is still needed in those cases even though there
won't be an associated cache entry.)

The `Calc()` function and list of prerequisite supplied to the
@ref DeclareLeafOutputPort_documentation "declare output ports APIs" are
used directly to construct the output port's cache entry.

<h3>Declaring dependencies for built-in computations</h3>

The built-in computations like time derivatives, energy, and power are
associated with bespoke "Calc" methods which need to have known dependency
lists. Rather than add these with yet more System methods, we should consider
changing the API to define them analogously to the OutputPort and CacheEntry
declarations which accept Allocator() and Calculator() functors and a
dependency list for the Calculator(). Currently they are just defaulting to
"depends on everything".

See Drake issue [#9205](https://github.com/RobotLocomotion/drake/issues/9205)
on GitHub for a checklist of caching loose ends.
*/

}  // namespace systems
}  // namespace drake
