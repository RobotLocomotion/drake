#pragma once

/** @file
Declares DependencyTracker and DependencyGraph which is the container for
trackers. */

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/text_logging.h"
#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/framework_common.h"

namespace drake {
namespace systems {

class DependencyGraph;  // defined below

//==============================================================================
//                             DEPENDENCY TRACKER
//==============================================================================
/** Manages value interdependencies for a particular value or set of values in
a Context.

A %DependencyTracker ("tracker" for short) provides notifications of changes to
the managed value to downstream subscribers, and may invalidate an associated
cache entry. The "managed value" can be a source like time or state, or a cached
computation. A particular tracker is selected using a DependencyTicket
("ticket") which provides very fast access to the tracker. The ticket is used by
both the System and Context as a way to identify dependencies, while trackers
exist only in the Context.

Each %DependencyTracker manages dependencies for a value, or group of related
values, upon which some downstream computations may depend, and maintains lists
of downstream dependents (subscribers) and upstream prerequisites. An optional
CacheEntryValue may be registered with a tracker in which case the tracker will
mark the cache value out of date when one of its prerequisites has changed.

A single %DependencyTracker can represent interdependencies within its
subcontext, and to and from other subcontexts within the same containing Context
tree. Trackers are always owned by a DependencyGraph that is part of a
particular subcontext, and should always be created through methods of
DependencyGraph; don't construct them directly yourself.

%DependencyTracker objects within a Context are nodes in a directed acylic graph
formed by "is-prerequisite-of" edges leading from source values (like time,
state, parameters, and input ports) to dependent cached computations and output
ports. A %DependencyTracker maintains lists of both its downstream subscribers
and its upstream prerequisites. The entries in both lists are pointers to other
%DependencyTrackers. That requires special handling when cloning a Context,
since the internal pointers to the %DependencyTracker objects in the source must
be replaced by their corresponding pointers in the copy.

%DependencyTrackers may simply group upstream values, without representing a
new value or computation. For example, the three continuous state subgroups
q, v, and z are each associated with their own %DependencyTracker. There is
also a tracker that monitors changes to _any_ variable within the entire
collection of continuous variables `xc≜{q,v,z}`; that tracker
subscribes to the three individual trackers. Similarly, individual discrete
variable groups dᵢ collectively determine the discrete state `xd≜{dᵢ}`,
individual abstract state variables aᵢ determine the abstract state `xa≜{aᵢ}`,
and the full state is `x≜{xc,xd,xa}`. Here is a graph showing
time and state trackers and some hypothetical cache entry trackers.
<pre>
                   (q)--------➙(position kinematics)
                      ➘
                 (v)--➙(xc)---     (time)----➙(xc_dot)
                 (z)--➚       ╲              ➚
                               ➘            ╱
                (d₀)--➙(xd)---➙(x)----------
                (d₁)--➚        ➚
                              ╱
                (a₀)--➙(xa)---
                (a₁)--➚
</pre>
The parenthesized nodes are %DependencyTrackers for the indicated values, and
a directed edge `(a)->(b)` can be read as "a is-prerequisite-of b" or
"a determines b". The graph also maintains reverse-direction edges (not shown).
A reversed edge `(a)<-(b)` could be read as "b subscribes-to a" or
"b depends-on a".)

These grouped trackers simplify dependency specification for quantities that
depend on many sources, which is very common. For example, they allow a user to
express a dependence on "all the inputs" without actually having to
know how many inputs there are, which might change over time. Grouped trackers
also serve to reduce the total number of edges in the dependency graph,
providing faster invalidation. For example, if there are 10 computations
dependent on q, v, and z (which frequently change together) we would have 30
edges. Introducing (xc) reduces that to 13 edges.

Downstream computations may subscribe to any of the individual or grouped
nodes. */

// Implementation notes:
// Invalidation operations occur very frequently at run time and must execute
// very fast. To make invalidation an O(N) operation, it is essential to avoid
// unnecessary repeated invalidations of the same subgraph during an
// invalidation sweep. That is handled via a unique "change event"
// serial number that is stored in a tracker when it is first invalidated.
// Encountering a node with a matching change event number terminates that
// branch of an invalidation sweep using that change event. Calling code can
// improve performance further by grouping simultaneous changes (say time and
// state) together into a single change event.
//
// Lots of things can go wrong so we maintain lots of redundant information here
// and check it religiously in Debug builds, less so in Release builds.
//
// For maximum speed, DependencyTracker objects contain pointers to other
// trackers within the same containing Context tree. That is, the referenced
// trackers can be in any subcontext that shares the same root Context as the
// subcontext that owns this tracker; they are not necessarily limited to
// trackers within the same subcontext. There is also
// a pointer to the containing Context's system pathname service for use in
// logging and error messages. These pointers provide great performance but
// make DependencyTrackers hard to clone because the pointers have to be set to
// point to corresponding entries in the new copy.
//
// DependencyTrackers for cache entries have to invalidate the associated cache
// value when notified of prerequisite changes. That simply sets a bool that is
// maintained by the CacheEntryValue object. This is an inner loop activity
// that must be done very efficiently. It is faster to invalidate with
// unconditional, inline code than to have a runtime test or abstract interface
// for cache invalidation. To permit that, we allocate a per-Cache dummy
// CacheEntryValue, make it available for all non-cache DependencyTrackers
// to "invalidate", and require that the definition of the cache invalidation
// method is visible here rather than use an abstract interface to it.

class DependencyTracker {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DependencyTracker)

  /** (Internal use only) */
  using PointerMap = std::unordered_map<const DependencyTracker*,
                                        const DependencyTracker*>;

  /** Returns the human-readable description for this tracker. */
  const std::string& description() const { return description_; }

  /** Returns the description, preceded by the full pathname of the subsystem
  associated with the owning subcontext. */
  std::string GetPathDescription() const;

  /** Returns the DependencyTicket for this %DependencyTracker in its containing
  DependencyGraph. The ticket is unique within the containing subcontext. */
  DependencyTicket ticket() const { return ticket_; }

  // This is intended for use in connecting pre-defined trackers to cache
  // entry values that are created later. See the private constructor
  // documentation for more information.
  /** (Internal use only) Sets the cache entry value to be marked out-of-date
  when this tracker's prerequisites change.
  @pre The supplied cache entry value is non-null.
  @pre No cache entry value has previously been assigned. */

  void set_cache_entry_value(CacheEntryValue* cache_value) {
    DRAKE_DEMAND(cache_value != nullptr);
    DRAKE_DEMAND(!has_associated_cache_entry_);
    cache_value_ = cache_value;
    has_associated_cache_entry_ = true;
  }

  // This is for validating that this tracker is associated with the right
  // cache entry. Don't use this during runtime invalidation.
  /** (Internal use only) Returns a pointer to the CacheEntryValue if this
  tracker is a cache entry tracker, otherwise nullptr. */
  const CacheEntryValue* cache_entry_value() const {
    DRAKE_DEMAND(cache_value_ != nullptr);
    if (!has_associated_cache_entry_)
      return nullptr;  // cache_value_ actually points to a dummy entry.
    return cache_value_;
  }

  /** Notifies `this` %DependencyTracker that its managed value was directly
  modified or made available for mutable access. That is, this is the
  _initiating_ event of a value modification. All of our downstream
  subscribers are notified but the associated cache entry (if any) is _not_
  invalidated (see below for why). A unique, positive `change_event` should
  have been obtained from the owning Context and supplied here.

  Why don't we invalidate the cache entry? Recall that this method is for
  _initiating_ a change event, meaning that the quantity that this tracker
  tracks is _initiating_ an invalidation sweep, as opposed to just reacting to
  prerequisite changes. Normally cache entries become invalid because their
  prerequisites change; they are not usually the first step in an invalidation
  sweep. So it is unusual for NoteValueChange() to be called on a cache entry's
  dependency tracker. But if it is called, that is likely to mean the cache
  entry was just given a new value, and is therefore _valid_; invalidating it
  now would be an error. */
  void NoteValueChange(int64_t change_event) const;

  /** @name              Prerequisites and subscribers
  These methods deal with dependencies associated with this tracker. */
  //@{

  /** Subscribes `this` tracker to an upstream prerequisite's tracker. The
  upstream tracker will keep a const pointer back to `this` tracker in its
  subscriber list, and `this` tracker will keep a pointer to the prerequisite
  tracker in its prerequisites list. */
  void SubscribeToPrerequisite(DependencyTracker* prerequisite);

  /** Unsubscribes `this` tracker from an upstream prerequisite tracker to
  which we previously subscribed. Both the prerequisite list in `this` tracker
  and the subscriber list in `prerequisite` are modified.
  @pre The supplied pointer must not be null.
  @pre This tracker must already be subscribed to the given `prerequisite`. */
  void UnsubscribeFromPrerequisite(DependencyTracker* prerequisite);

  /** Adds a downstream subscriber to `this` %DependencyTracker, which will keep
  a pointer to the subscribing tracker. The subscriber will be notified whenever
  this %DependencyTracker is notified of a value or prerequisite change.
  @pre The subscriber has already recorded its dependency on this tracker in its
       prerequisite list. */
  void AddDownstreamSubscriber(const DependencyTracker& subscriber);

  /** Removes a downstream subscriber from `this` %DependencyTracker.
  @pre The subscriber has already removed the dependency on this tracker from
       its prerequisite list. */
  void RemoveDownstreamSubscriber(const DependencyTracker& subscriber);

  /** Returns `true` if this tracker has already subscribed to `prerequisite`.
  This is slow and should not be used in performance-sensitive code. */
  bool HasPrerequisite(const DependencyTracker& prerequisite) const;

  /** Returns `true` if `subscriber` is one of this tracker's subscribers.
  This is slow and should not be used in performance-sensitive code. */
  bool HasSubscriber(const DependencyTracker& subscriber) const;

  /** Returns the total number of "depends-on" edges emanating from `this`
  tracker, pointing to its upstream prerequisites. */
  int num_prerequisites() const {
    return static_cast<int>(prerequisites_.size());
  }

  /** Returns a reference to the prerequisite trackers. */
  const std::vector<const DependencyTracker*>& prerequisites() const {
    return prerequisites_;
  }

  /** Returns the total number of "is-prerequisite-of" edges emanating from
  `this` tracker, pointing to its downstream subscribers. */
  int num_subscribers() const { return static_cast<int>(subscribers_.size()); }

  /** Returns a reference to the subscribing trackers. */
  const std::vector<const DependencyTracker*>& subscribers() const {
    return subscribers_;
  }
  //@}

  /** @name                     Runtime statistics
  These methods track runtime operations and are useful for debugging and for
  performance analysis. */
  //@{

  /** What is the total number of notifications received by this tracker?
  This is the sum of managed-value change event notifications and prerequisite
  change notifications received. */
  int64_t num_notifications_received() const {
    return num_value_change_events() + num_prerequisite_change_events();
  }

  /** How many times did we receive a repeat notification for the same change
  event that we ignored? */
  int64_t num_ignored_notifications() const {
    return num_ignored_notifications_;
  }

  /** What is the total number of notifications sent to downstream subscribers
  by this trackers? */
  int64_t num_notifications_sent() const {
    return num_downstream_notifications_sent_;
  }

  /** How many times was this tracker notified of a change event for a direct
  change to a value it tracks? */
  int64_t num_value_change_events() const {
    return num_value_change_notifications_received_;
  }

  /** How many times was this tracker notified of a change to one of its value's
  prerequisites? */
  int64_t num_prerequisite_change_events() const {
    return num_prerequisite_notifications_received_;
  }
  //@}

  /** @name                Testing/debugging utilities
  Methods used in test cases or for debugging. */
  //@{

  /** Throws an std::exception if there is something clearly wrong with this
  %DependencyTracker object. If the owning subcontext is known, provide a
  pointer to it here and we'll check that this tracker agrees. If you know which
  cache entry is supposed to be associated with this tracker, supply a pointer
  to that and we'll check it (trackers that are not associated with a real cache
  entry are still associated with the CacheEntryValue::dummy()). In addition we
  check for other internal inconsistencies.
  @throws std::exception for anything that goes wrong, with an appropriate
                         explanatory message. */
  void ThrowIfBadDependencyTracker(
      const internal::ContextMessageInterface* owning_subcontext = nullptr,
      const CacheEntryValue* cache_value = nullptr) const;
  //@}

 private:
  friend class DependencyGraph;

  // (Private because trackers should only be created by DependencyGraph.)
  // Constructs a tracker with a given ticket number, a human-readable
  // description and an optional CacheEntryValue object that should be marked
  // out-of-date when a prerequisite changes. The description should be of the
  // associated value only, like "input port 3"; don't include "tracker". If
  // the owning_subcontext is non-null we can complete the initialization,
  // but when this is used as part of Context cloning we'll have to defer the
  // final steps until the pointer fixup phase.
  DependencyTracker(DependencyTicket ticket, std::string description,
                    const internal::ContextMessageInterface* owning_subcontext,
                    CacheEntryValue* cache_value)
      : ticket_(ticket),
        description_(std::move(description)),
        owning_subcontext_(owning_subcontext),  // may be nullptr
        has_associated_cache_entry_(cache_value != nullptr),
        cache_value_(cache_value) {
    // If we can, connect non-cache tracker to the dummy cache entry value now.
    if (!has_associated_cache_entry_ && owning_subcontext != nullptr)
      cache_value_ = &owning_subcontext->dummy_cache_entry_value();

    DRAKE_LOGGER_DEBUG(
        "Tracker #{} '{}' constructed {} invalidation {:#x}{}.", ticket_,
        description_, has_associated_cache_entry_ ? "with" : "without",
        size_t(cache_value),
        has_associated_cache_entry_
            ? " cache entry " + std::to_string(cache_value->cache_index())
            : "");
  }

  // Copies the current tracker but with all pointers set to null, and all
  // counters reset to their default-constructed values (0 for statistics, an
  // unmatchable value for the last change event).
  std::unique_ptr<DependencyTracker> CloneWithoutPointers() const {
    // Can't use make_unique here because constructor is private.
    std::unique_ptr<DependencyTracker> clone(
        new DependencyTracker(ticket(), description(), nullptr, nullptr));
    clone->has_associated_cache_entry_ = has_associated_cache_entry_;
    // The constructor sets cache_value_ to dummy by default, but that's wrong
    // if there is an associated cache entry. In that case we'll set it later.
    if (has_associated_cache_entry_)
      clone->cache_value_ = nullptr;
    clone->subscribers_.resize(num_subscribers(), nullptr);
    clone->prerequisites_.resize(num_prerequisites(), nullptr);
    return clone;
  }

  // Assumes `this` tracker is a recent clone containing no pointers, sets
  // the pointers here to addresses corresponding to those in the source
  // tracker, with the help of the given map. It is a fatal error if any needed
  // pointer is not present in the map. Performs a sanity check that the
  // resulting tracker looks reasonable.
  void RepairTrackerPointers(
      const DependencyTracker& source,
      const DependencyTracker::PointerMap& tracker_map,
      const internal::ContextMessageInterface* owning_subcontext, Cache* cache);

  // Notifies `this` DependencyTracker that one of its prerequisite values was
  // modified or made available for mutable access. All of our downstream
  // subscribers are notified and the associated cache entry (if any) is
  // invalidated. The unique `change_event` obtained by the initiating value
  // modification should be passed through here. The particular upstream
  // `prerequisite` reporting the change is provided here for enforcing
  // invariants in Debug builds. `depth` measures the notification chain length
  // and is useful for debugging and performance analysis. An initial caller
  // should supply `depth`=0; it is incremented internally.
  void NotePrerequisiteChange(int64_t change_event,
                              const DependencyTracker& prerequisite,
                              int depth) const;

  // Notifies downstream subscribers that they are no longer valid. This may
  // have been initiated by a change to our tracked value or an upstream
  // prerequisite; downstream subscribers can't tell the difference.
  void NotifySubscribers(int64_t change_event, int depth) const;

  std::string GetSystemPathname() const {
    DRAKE_DEMAND(owning_subcontext_!= nullptr);
    return owning_subcontext_->GetSystemPathname();
  }

  // Provides an identifying prefix for error messages.
  std::string FormatName(const char* api) const {
    return "DependencyTracker(" + GetPathDescription() + ")::" + api + "(): ";
  }

  // This tracker's index within its owning DependencyGraph.
  const DependencyTicket ticket_;

  const std::string description_;

  // Pointer to the system name service of the owning subcontext.
  const internal::ContextMessageInterface* owning_subcontext_{nullptr};

  // If false, cache_value_ will be set to point to CacheEntryValue::dummy() so
  // we don't need to check during invalidation sweeps.
  bool has_associated_cache_entry_{false};
  CacheEntryValue* cache_value_{nullptr};

  std::vector<const DependencyTracker*> subscribers_;
  std::vector<const DependencyTracker*> prerequisites_;

  // Used for short-circuiting repeated notifications. Does not otherwise change
  // the result; hence mutable is OK. All legitimate change events must be
  // greater than zero, so this will never match.
  mutable int64_t last_change_event_{-1};

  // Runtime statistics. Does not change behavior at all.
  mutable int64_t num_value_change_notifications_received_{0};
  mutable int64_t num_prerequisite_notifications_received_{0};
  mutable int64_t num_ignored_notifications_{0};
  mutable int64_t num_downstream_notifications_sent_{0};
};

//==============================================================================
//                            DEPENDENCY GRAPH
//==============================================================================
/** Represents the portion of the complete dependency graph that is a subgraph
centered on the owning subcontext, plus some edges leading to other subcontexts.
DependencyTracker objects are the nodes of the graph, and maintain
prerequisite/subscriber edges that interconnect these nodes, and may also
connect to nodes contained in dependency graphs belonging to other subcontexts
within the same complete context tree. Dependencies on the parent (containing
DiagramContext) and children (contained subcontexts) typically arise from
exported input and output ports, while sibling dependencies arise from
output-to-input port connections.

A %DependencyGraph creates and owns all the DependencyTracker objects for a
particular subcontext, organized to allow fast access using a DependencyTicket
as an index. Memory addresses of DependencyTracker objects are stable once
allocated, but DependencyTicket numbers are stable even after a Context has been
copied so should be preferred.

Because DependencyTrackers contain pointers, copying a %DependencyGraph must
always be done as part of copying an entire Context tree. There is a copy
constructor here but it must be followed by a pointer-fixup step so is for
internal use only. */
class DependencyGraph {
 public:
  /** @name  Does not allow move or assignment; copy constructor limited.
  The copy constructor does not copy internal pointers so requires special
  handling. */
  /** @{ */
  DependencyGraph(DependencyGraph&&) = delete;
  DependencyGraph& operator=(const DependencyGraph&) = delete;
  DependencyGraph& operator=(DependencyGraph&&) = delete;
  /** @} */

  /** Constructor creates an empty graph referencing the system pathname
  service of its owning subcontext. The supplied pointer must not be null. */
  explicit DependencyGraph(
      const internal::ContextMessageInterface* owning_subcontext)
      : owning_subcontext_(owning_subcontext) {
    DRAKE_DEMAND(owning_subcontext != nullptr);
  }

  /** Deletes all DependencyTracker objects; no notifications are issued. */
  ~DependencyGraph() = default;

  /** Allocates a new DependencyTracker with an already-known ticket number, the
  given description and an optional cache value to be invalidated. The new
  tracker has no prerequisites or subscribers yet. This may leave gaps in the
  node numbering. Use has_tracker() if you need to know whether there is a
  tracker for a particular ticket. We promise that the returned
  DependencyTracker's location in memory will remain unchanged once created in
  a particular Context, even as more trackers are added. The DependencyTicket
  retains its meaning even after cloning the Context, although of course the
  tracker has a new address in the clone.
  @pre The given ticket must be valid.
  @pre No DependencyTracker is already using the given ticket. */
  DependencyTracker& CreateNewDependencyTracker(
      DependencyTicket known_ticket, std::string description,
      CacheEntryValue* cache_value = nullptr) {
    DRAKE_DEMAND(!has_tracker(known_ticket));
    if (known_ticket >= trackers_size()) graph_.resize(known_ticket + 1);
    // Can't use make_unique here because constructor is private.
    graph_[known_ticket].reset(new DependencyTracker(
        known_ticket, std::move(description), owning_subcontext_, cache_value));
    return *graph_[known_ticket];
  }

  /** Assigns a new ticket number and then allocates a new DependencyTracker
  that can be accessed with that ticket. You may obtain the assigned
  ticket from the returned tracker. See the other signature for details. */
  DependencyTracker& CreateNewDependencyTracker(
      std::string description, CacheEntryValue* cache_value = nullptr) {
    DependencyTicket ticket(trackers_size());
    return CreateNewDependencyTracker(ticket, std::move(description),
                                      cache_value);
  }

  /** Returns true if there is a DependencyTracker in this graph that has the
  given ticket number. */
  bool has_tracker(DependencyTicket ticket) const {
    DRAKE_DEMAND(ticket.is_valid());
    if (ticket >= trackers_size()) return false;
    return graph_[ticket] != nullptr;
  }

  /** Returns the current size of the DependencyTracker container, providing
  for DependencyTicket numbers from `0..trackers_size()-1`. Note that it is
  possible to have empty slots in the container. Use has_tracker() to determine
  if there is a tracker associated with a particular ticket. */
  int trackers_size() const { return static_cast<int>(graph_.size()); }

  /** Returns a const DependencyTracker given a ticket. This is very fast.
  Behavior is undefined if the ticket is out of range [0..num_trackers()-1]. */
  const DependencyTracker& get_tracker(DependencyTicket ticket) const {
    DRAKE_ASSERT(has_tracker(ticket));
    DependencyTracker& tracker = *graph_[ticket];
    DRAKE_ASSERT(tracker.ticket() == ticket);
    return tracker;
  }

  /** Returns a mutable DependencyTracker given a ticket. This is very fast.
  Behavior is undefined if the ticket is out of range [0..num_trackers()-1]. */
  DependencyTracker& get_mutable_tracker(DependencyTicket ticket) {
    return const_cast<DependencyTracker&>(get_tracker(ticket));
  }

  /** (Internal use only) Copy constructor partially duplicates the source
  %DependencyGraph object, with identical structure to the source but
  with all internal pointers set to null, and all counters and statistics set
  to their default-constructed values. Pointers must be set properly using
  RepairTrackerPointers() once all the old-to-new pointer mappings have been
  determined _for the whole Context_, not just the containing subcontext. This
  should only be invoked by Context code as part of copying an entire Context
  tree.
  @see AppendToTrackerPointerMap(), RepairTrackerPointers() */
  DependencyGraph(const DependencyGraph& source) {
    graph_.reserve(source.trackers_size());
    for (DependencyTicket ticket(0); ticket < source.trackers_size();
         ++ticket) {
      graph_.emplace_back(
          source.has_tracker(ticket)
              ? source.get_tracker(ticket).CloneWithoutPointers()
              : nullptr);
    }
  }

  /** (Internal use only) Create a mapping from the memory addresses of the
  trackers contained here to the corresponding ones in `clone`, which must have
  exactly the same number of trackers. The mapping is appended to the supplied
  map, which must not be null. */
  void AppendToTrackerPointerMap(
      const DependencyGraph& clone,
      DependencyTracker::PointerMap* tracker_map) const;

  /** (Internal use only) Assumes `this` %DependencyGraph is a recent clone
  whose trackers do not yet contain subscriber and prerequisite pointers and
  sets the local pointers to point to the `source`-corresponding trackers in the
  new owning context, the appropriate cache entry values in the new cache, and
  to the system name providing service of the new owning Context for logging and
  error reporting. The supplied map should map source pointers to their
  corresponding trackers. It is a fatal error if any old pointer we encounter is
  not present in the map; that would indicate a bug in the Context cloning
  code. */
  void RepairTrackerPointers(
      const DependencyGraph& source,
      const DependencyTracker::PointerMap& tracker_map,
      const internal::ContextMessageInterface* owning_subcontext,
      Cache* new_cache);

 private:
  // The system name service of the subcontext that owns this subgraph.
  const internal::ContextMessageInterface* owning_subcontext_{};

  // All value trackers, indexed by DependencyTicket.
  std::vector<std::unique_ptr<DependencyTracker>> graph_;
};

}  // namespace systems
}  // namespace drake
