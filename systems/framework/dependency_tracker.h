#pragma once

/** @file
Declares DependencyTracker and DependencyGraph which is the container for
trackers. */

#include <algorithm>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/text_logging.h"
#include "drake/common/type_safe_index.h"
#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/framework_common.h"

namespace drake {
namespace systems {

// ContextBase must include this header so we can't include context_base.h.
class ContextBase;

class DependencyGraph;  // defined below

//==============================================================================
//                             DEPENDENCY TRACKER
//==============================================================================
/** Manages value interdependencies for a particular value or set of values in
a Context. Provides notifications of changes to the managed value to downstream
subscribers, and may invalidate an associated cache entry. The "managed value"
can be a source like time or state, or a cached computation. Each
%DependencyTracker ("tracker" for short) manages dependencies for a value or
group of related values upon which some downstream computations may depend, and
maintains lists of downstream dependents (subscribers) and upstream
prerequisites. An optional CacheEntryValue may be registered with a tracker for
marking the managed cache value out of date when one of its prerequisites has
changed.

A single %DependencyTracker can represent interdependencies within its
subcontext, and to and from other subcontexts within the same containing Context
tree. Trackers are always owned by a DependencyGraph that is part of a
particular subcontext. They should not be created except under the careful
supervision of their owning DependencyGraph, which has helpful methods for that
purpose.

%DependencyTracker objects within a Context are nodes in a directed acylic graph
formed by "is-prerequisite-of" edges leading from source values (like time,
state, parameters, and input ports) to dependent cached computations and output
ports. A %DependencyTracker maintains lists of both its downstream subscribers,
and its upstream prerequisites. The entries in both lists are pointers to other
%DependencyTrackers. That requires special handling when cloning a Context,
since the internal pointers in the source must be replaced by their
corresponding pointers in the copy.

%DependencyTrackers may simply group upstream values, without representing a
new value or computation. For example, the three continuous state
subgroups q, v, and z each have their own %DependencyTracker. There is
also a tracker for _any_ continuous variable change `xc≜{q,v,z}`; that tracker
subscribes to the three individual trackers. Similarly individual discrete
variable groups dᵢ collectively determine the discrete state `xd≜{dᵢ}`,
individual abstract state variables aᵢ determine the abstract state `xa≜{aᵢ}`,
and the full state is `x≜{xc,xd,xa}`. Here is a graph showing time and state
trackers and some hypothetical cache entry trackers.
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
the directed edges can be read as "is-prerequisite-of" or "determines".
(Reversed edges would read "subscribes-to" or "depends-on".) Downstream
computations may subscribe to any of the individual or grouped nodes. */

// Implementation notes:
// Invalidation operations occur very frequently at run time and must execute
// very fast. Simultaneous changes (like time and state) should be grouped
// to avoid unnecessary invalidation passes. Also, care must be
// taken to avoid unnecessary repeated invalidations of the same subgraph
// during invalidations; that is handled by generating a unique "change event"
// serial number that is stored in a tracker when it is first invalidated.
// Encountering a node with a matching change event number terminates that
// branch of an invalidation sweep using that change event.
//
// Lots of things can go wrong so we maintain lots of redundant information here
// and check it religiously in Debug builds, less so in Release builds.
//
// For maximum speed, DependencyTracker objects contain pointers to other
// trackers within the same complete Context tree, not necessarily limited to
// trackers within the same subcontext that owns the tracker. There is also
// a back pointer to the containing DependencyGraph. These pointers provide
// great performance but makes these things hard to clone because the pointers
// have to be fixed up to point to corresponding entries in the new copy.
//
// DependencyTrackers for cache entries have to invalidate the associated cache
// value when notified of prerequisite changes. That simply sets a bool to
// `false` in the CacheEntryValue object. It is faster and cleaner to do that
// unconditionally so we have a static dummy CacheEntryValue available for all
// non-cache DependencyTrackers to invalidate.

class DependencyTracker {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DependencyTracker)

  using PointerMap = std::unordered_map<const DependencyTracker*,
                                        const DependencyTracker*>;

  /** (Internal use only) Constructs a tracker with a given ticket number,
  a human-readable description and an optional CacheEntryValue object that
  should be marked out-of-date when a prerequisite changes. The description
  should be of the associated value only, like "input port 3"; don't include
  "tracker". This constructor is only for use by DependencyGraph, and the
  owning DependencyGraph must be supplied here and be non-null. */
  DependencyTracker(const DependencyGraph* owning_subgraph,
                    DependencyTicket ticket, std::string description,
                    CacheEntryValue* cache_value)
      : owning_subgraph_(owning_subgraph),
        ticket_(ticket),
        description_(std::move(description)),
        cache_value_(cache_value ? cache_value : &dummy_cache_value_) {
    DRAKE_DEMAND(owning_subgraph != nullptr);
    SPDLOG_DEBUG(
        log(), "Tracker #{} '{}' constructed {} invalidation {:#x}{}.", ticket_,
        description_, cache_value ? "with" : "without", size_t(cache_value),
        cache_value
            ? " cache entry " + std::to_string(cache_value->cache_index())
            : "");
  }

  /** Notifies `this` %DependencyTracker that its managed value was directly
  modified or made available for mutable access. That is, this is the
  _initiating_ event of a value modification. All of our downstream
  subscribers are notified but the associated cache entry (if any) is _not_
  invalidated, since it would be that cache entry that is initiating this
  change. A unique `change_event` should have been obtained from the owning
  Context and supplied here. */
  void NoteValueChange(int64_t change_event) const;

  /** Subscribes `this` tracker to an upstream prerequisite's tracker. The
  upstream tracker will keep a const pointer back to `this` tracker in its
  subscriber list, and `this` tracker will keep a pointer to the prerequisite
  tracker in its prerequisites list. */
  void SubscribeToPrerequisite(DependencyTracker* prerequisite);

  /** Unsubscribes `this` tracker from an upstream prerequisite tracker to
  which we previously subscribed. Throws an exception if we are not already
  subscribed. Both the prerequisite list in `this` tracker and the subscriber
  list in `prerequisite` are modified. */
  void UnsubscribeFromPrerequisite(DependencyTracker* prerequisite);

  /** Adds a downstream subscriber to `this` %DependencyTracker, which will keep
  a pointer to the subscribing tracker. The subscriber will be notified whenever
  this %DependencyTracker is notified of a value or prerequisite change. This is
  only allowed if the subscriber has already recorded its dependency on this
  tracker in its prerequisite list. */
  void AddDownstreamSubscriber(const DependencyTracker& subscriber);

  /** Removes a downstream subscriber from `this` %DependencyTracker. This is
  only allowed if the subscriber has already removed the dependency on this
  tracker from its prerequisite list. */
  void RemoveDownstreamSubscriber(const DependencyTracker& subscriber);

  /** Returns the human-readable description for this tracker. */
  const std::string& description() const { return description_; }

  /** Returns the description, preceded by the full pathname of the subsystem
  associated with the owning subcontext. */
  std::string GetPathDescription() const;

  /** Returns `true` if the given prerequisite is already listed here. This is
  slow and should not be used in performance-sensitive code. */
  bool HasPrerequisite(const DependencyTracker& prerequisite) const;

  /** Returns `true` if the given subscriber is already listed here. This is
  slow and should not be used in performance-sensitive code. */
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

  /** Returns the total number of notifications received by this tracker.
  This is the sum of managed-value notifications and prerequisite notifications
  received. */
  int64_t num_notifications_received() const {
    return num_value_change_notifications_received_ +
           num_prerequisite_notifications_received_;
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

  /** Returns the ticket number for this DependencyTracker in its containing
  DependencyGraph. */
  DependencyTicket ticket() const { return ticket_; }

  /** Returns the DependencyGraph that owns this tracker. */
  const DependencyGraph& get_owning_subgraph() const {
    return *owning_subgraph_;
  }

  /** Assumes `this` tracker is a recent clone containing no pointers, sets
  the pointers here to addresses corresponding to those in the source tracker,
  with the help of the given map. It is a fatal error if any needed pointer is
  not present in the map. */
  void RepairTrackerPointers(const DependencyTracker& source,
                             const DependencyTracker::PointerMap& tracker_map,
                             Cache* cache);

  /** Copies the current tracker but with all pointers set to null. */
  std::unique_ptr<DependencyTracker> CloneWithoutPointers(
      const DependencyGraph* new_owner) const {
    auto clone = std::make_unique<DependencyTracker>(
        new_owner, ticket(), description(), nullptr);
    clone->SetSizesAndNullPointers(*this);
    return clone;
  }

 private:
  // As part of a cloning operation, makes sizes match and set internal
  // pointers to null.
  void SetSizesAndNullPointers(const DependencyTracker& source) {
    cache_value_ = nullptr;
    subscribers_.resize(source.num_subscribers(), nullptr);
    prerequisites_.resize(source.num_prerequisites(), nullptr);
  }

  // Notifies `this` DependencyTracker that one of its prerequisite values was
  // modified or made available for mutable access. All of our downstream
  // subscribers are notified and the associated cache entry (if any) is
  // invalidated. The unique `change_event` obtained by the initiating value
  // modification should be passed through here. The particular upstream
  // `prerequisite` reporting the change is provided here for enforcing
  // invariants in Debug builds.`depth` measures the notification chain length
  // and is useful for debugging and performance analysis. An initial caller
  // should supply `depth`=0; it is incremented internally.
  void NotePrerequisiteChange(int64_t change_event,
                              const DependencyTracker& prerequisite,
                              int depth) const;

  // Notifies downstream subscribers that they are no longer valid. This may
  // have been initiated by a change to our tracked value or an upstream
  // prerequisite; downstream subscribers can't tell the difference.
  void NotifySubscribers(int64_t change_event, int depth) const;

  // For debugging use, provide an indent of 2*depth characters.
  static std::string Indent(int depth);

  // Back pointer to the subgraph that created and owns this tracker.
  const DependencyGraph* owning_subgraph_{};
  // This tracker's index within its owning DependencyGraph.
  DependencyTicket ticket_;

  std::string description_;

  // Points to the dummy_cache_value below if we're not told otherwise.
  CacheEntryValue* cache_value_{};

  std::vector<const DependencyTracker*> subscribers_;
  std::vector<const DependencyTracker*> prerequisites_;

  // Used for short-circuiting repeated notifications. Does not otherwise change
  // the result; hence mutable is OK.
  mutable int64_t last_change_event_{-1};

  // Runtime statistics. Does not change behavior at all.
  mutable int64_t num_value_change_notifications_received_{0};
  mutable int64_t num_prerequisite_notifications_received_{0};
  mutable int64_t num_ignored_notifications_{0};
  mutable int64_t num_downstream_notifications_sent_{0};

  static CacheEntryValue dummy_cache_value_;
};

//==============================================================================
//                            DEPENDENCY GRAPH
//==============================================================================
/** Represents the portion of the complete dependency graph that is a subgraph
centered on the owning subcontext, plus some edges leading to other subcontexts.
DependencyTracker objects are the nodes of the graph, and maintain
prerequisite/subscriber edges that interconnect these nodes, and may also
connect to nodes contained in dependency graphs belonging to other subcontexts
within the same complete context tree. Parent and child dependencies
typically arise from exported input and output ports, while sibling dependencies
arise from output-to-input port connections.

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

  /** Constructor creates an empty graph referencing its owning subcontext.
  The supplied pointer must not be null. */
  explicit DependencyGraph(const ContextBase* owning_subcontext)
      : owning_subcontext_(owning_subcontext) {
    DRAKE_DEMAND(owning_subcontext != nullptr);
  }

  /** Deletes all DependencyTracker objects; no notifications are issued. */
  ~DependencyGraph() = default;

  /** Allocates a new DependencyTracker with an already-known ticket number, the
  given description and an optional cache value to be invalidated. The new
  tracker has no prerequisites or subscribers yet. Note that the
  DependencyTracker memory address is stable after allocation in a particular
  Context while the DependencyTicket remains stable even after cloning the
  Context. There must be no DependencyTracker already using the given ticket,
  which must be valid. Note that this may leave gaps in the node numbering. Use
  has_tracker() if you need to know whether there is a tracker for a particular
  ticket. */
  DependencyTracker& CreateNewDependencyTracker(
      DependencyTicket known_ticket, std::string description,
      CacheEntryValue* cache_value = nullptr) {
    DRAKE_DEMAND(!has_tracker(known_ticket));
    if (known_ticket >= num_trackers()) graph_.resize(known_ticket + 1);
    graph_[known_ticket] = std::make_unique<DependencyTracker>(
        this, known_ticket, std::move(description), cache_value);
    return *graph_[known_ticket];
  }

  /** Assigns a new ticket number and then allocates a new DependencyTracker
  that can be accessed with that ticket. You may obtain the assigned
  ticket from the returned tracker. See the other signature for details. */
  DependencyTracker& CreateNewDependencyTracker(
      std::string description, CacheEntryValue* cache_value = nullptr) {
    DependencyTicket ticket(num_trackers());
    return CreateNewDependencyTracker(ticket, std::move(description),
                                      cache_value);
  }

  /** Returns true if there is a DependencyTracker in this graph that has the
  given ticket number. */
  bool has_tracker(DependencyTicket ticket) const {
    DRAKE_DEMAND(ticket.is_valid());
    if (ticket >= num_trackers()) return false;
    return graph_[ticket] != nullptr;
  }

  /** Returns the number of DependencyTracker objects currently stored here. */
  int num_trackers() const { return static_cast<int>(graph_.size()); }

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

  /** Returns a reference to the (sub)Context of which this DependencyGraph is
  a member. */
  const ContextBase& get_owning_subcontext() const {
    return *owning_subcontext_;
  }

  /** (Internal use only) Copy constructor partially duplicates the source
  %DependencyGraph object, with identical structure to the source but
  with all internal pointers set to null. These must be set properly in a later
  pass to reference corresponding elements of the new Context. This should
  only be invoked by Context code as part of copying an entire Context tree.
  @see AppendToTrackerPointerMap(), RepairTrackerPointers() */
  DependencyGraph(const DependencyGraph& source) : owning_subcontext_(nullptr) {
    graph_.reserve(source.num_trackers());
    for (DependencyTicket ticket(0); ticket < source.num_trackers(); ++ticket) {
      graph_.emplace_back(
          source.has_tracker(ticket)
              ? source.get_tracker(ticket).CloneWithoutPointers(this)
              : nullptr);
    }
  }

  /** Create a mapping from the memory addresses of the trackers contained
  here to the corresponding ones in `clone`, which must have exactly the same
  number of trackers. The mapping is appended to the supplied map, which
  must not be null. */
  void AppendToTrackerPointerMap(
      const DependencyGraph& clone,
      DependencyTracker::PointerMap* tracker_map) const;

  /** Assumes `this` %DependencyGraph is a recent clone whose trackers do not
  yet contain subscriber and prerequisite pointers and sets the local pointers
  to point to the `source`-corresponding trackers in the new owning context.
  The supplied map should map source pointers to their corresponding trackers.
  It is an error if any old pointer we encounter is not present in the map. */
  void RepairTrackerPointers(const ContextBase* owning_subcontext,
                             const DependencyGraph& source,
                             const DependencyTracker::PointerMap& tracker_map);

 private:
  // Back pointer to the subcontext that owns this subgraph.
  const ContextBase* owning_subcontext_{};

  // All value trackers, indexed by DependencyTicket.
  std::vector<std::unique_ptr<DependencyTracker>> graph_;
};

}  // namespace systems
}  // namespace drake
