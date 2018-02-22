#pragma once

#include <memory>
#include <string>

#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/dependency_tracker.h"

namespace drake {
namespace systems {

/** Provides non-templatized functionality shared by the templatized derived
classes. That includes caching and dependency tracking, and management of
local values for fixed input ports.

Terminology: in general a Drake System is a tree structure composed of
"subsystems", which are themselves System objects. The corresponding Context is
a parallel tree structure composed of "subcontexts", which are themselves
Context objects. There is a one-to-one correspondence between subsystems and
subcontexts. Within a given System (Context), its child subsystems (subcontexts)
are indexed using a SubsystemIndex; there is no separate SubcontextIndex since
the numbering must be identical. */
class ContextBase : public internal::SystemPathnameInterface {
 public:
  /** @name  Does not allow move or assignment; copy is protected. */
  /** @{ */
  // Copy constructor is used only to facilitate implementation of Clone()
  // in derived classes.
  ContextBase(ContextBase&&) = delete;
  ContextBase& operator=(const ContextBase&) = delete;
  ContextBase& operator=(ContextBase&&) = delete;
  /** @} */

  /** Creates an identical copy of the concrete context object. */
  std::unique_ptr<ContextBase> Clone() const;

  virtual ~ContextBase();

  /** (Debugging) Returns the local name of the subsystem for which this is the
  Context. See GetSystemPathname() if you want the full name. */
  // TODO(sherm1) Replace with the real name.
  std::string GetSystemName() const final { return "dummy"; }

  /** (Debugging) Returns the full pathname of the subsystem for which this is
  the Context. See get_system_pathname() if you want to the full name. */
  std::string GetSystemPathname() const final;

  /** Returns a const reference to this subcontext's cache. */
  const Cache& get_cache() const {
    return cache_;
  }

  /** (Advanced) Returns a mutable reference to this subcontext's cache. Note
  that this method is const because the cache is always writable.
  @warning Writing directly to the cache does not automatically propagate
  invalidations to downstream dependents of a contained cache entry, because
  invalidations would normally have been propagated when the cache entry itself
  went out of date. Cache entries are updated automatically when needed via
  their `Calc()` methods; most users should not bypass that mechanism by using
  this method. */
  Cache& get_mutable_cache() const {
    return cache_;
  }

  /** Returns a const reference to a DependencyTracker in this subcontext.
  Advanced users and internal code can use the returned reference to issue value
  change notifications -- mutable access is not required for that purpose. */
  const DependencyTracker& get_tracker(DependencyTicket ticket) const {
    return graph_.get_tracker(ticket);
  }

  /** Returns a mutable reference to a DependencyTracker in this subcontext.
  (You do not need mutable access just to issue value change notifications.) */
  DependencyTracker& get_mutable_tracker(DependencyTicket ticket) {
    return graph_.get_mutable_tracker(ticket);
  }

  /** Returns a const reference to the collection of value trackers within
  this subcontext. Together these form the dependency subgraph for the values
  in this subcontext, plus edges leading to neighboring trackers. */
  const DependencyGraph& get_dependency_graph() const {
    return graph_;
  }

  /** Returns a mutable reference to the dependency graph. */
  DependencyGraph& get_mutable_dependency_graph() {
    return graph_;
  }

 protected:
  /** Default constructor creates an empty ContextBase but initializes all the
  built-in dependency trackers that are the same in every System (like time,
  q, all states, all inputs, etc.). We can't allocate trackers for individual
  discrete & abstract states, parameters, or input ports since we don't yet
  know how many there are. */
  ContextBase() : graph_(this) {
    CreateBuiltInTrackers();
  }

  /** Copy constructor takes care of base class data members, but _does not_ fix
  up base class pointers. Derived classes must implement copy constructors that
  delegate to this one for use in their DoCloneWithoutPointers()
  implementations. The cache and dependency graph are copied, but any pointers
  contained in the source are left null in the copy. */
  ContextBase(const ContextBase& source)
      : cache_(source.cache_), graph_(source.graph_) {
    // Everything else is left default-initialized.
  }

  /** Clones a context but without any of its internal pointers. */
  std::unique_ptr<ContextBase> CloneWithoutPointers() const {
    return DoCloneWithoutPointers();
  }

  /** Derived classes must implement this so that it performs the complete
  deep copy of the context, including all base class members but not fixing
  up base class pointers. To do that, implement a protected copy constructor
  that inherits from the base class copy constructor (which doesn't repair the
  pointers), then implement DoCloneWithoutPointers() as
  `return unique_ptr<ContextBase>(new DerivedType(*this));`. */
  virtual std::unique_ptr<ContextBase> DoCloneWithoutPointers() const = 0;

 private:
  // Fills in the dependency graph with the built-in trackers that are common
  // to every Context (and every System).
  void CreateBuiltInTrackers();

  // Given a new context `clone` with the same dependency graph as this one,
  // create a mapping of all tracker memory addresses from `this` to `clone`.
  // This must be done for the whole Context tree because pointers can point
  // outside of their containing subcontext.
  void BuildTrackerPointerMap(
      const ContextBase& clone,
      DependencyTracker::PointerMap* tracker_map) const {
    // First map the pointers local to this context.
    graph_.AppendToTrackerPointerMap(clone.get_dependency_graph(),
                                     &(*tracker_map));
    // TODO(sherm1) Recursive update of descendents goes here.
  }

  // Assuming `this` is a recently-cloned Context containing stale references
  // to the source Context's trackers, repair those pointers using the given
  // map.
  void FixTrackerPointers(const ContextBase& source,
                          const DependencyTracker::PointerMap& tracker_map) {
    // First repair pointers local to this context.
    graph_.RepairTrackerPointers(source.get_dependency_graph(),
                                 tracker_map, this, &cache_);
    // TODO(sherm1) Recursive update of descendents goes here.
  }

  // The cache of pre-computed values owned by this subcontext.
  mutable Cache cache_;

  // This is the dependency graph for values within this subcontext.
  DependencyGraph graph_;
};

}  // namespace systems
}  // namespace drake
