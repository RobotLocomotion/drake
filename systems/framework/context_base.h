#pragma once

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/unused.h"
#include "drake/systems/framework/dependency_tracker.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

/** Provides non-templatized functionality shared by the templatized derived
classes. That includes caching and dependency tracking. */
// TODO(sherm1) This is a stub for now with just enough to allow us to
// test DependencyTracker. However the code that's here should be reviewed.
class ContextBase {
 public:
  /** @name  Does not allow move or assignment; copy is protected. */
  /** @{ */
  ContextBase(ContextBase&&) = delete;
  ContextBase& operator=(const ContextBase&) = delete;
  ContextBase& operator=(ContextBase&&) = delete;
  /** @} */

  /** Creates an identical copy of the concrete context object. */
  std::unique_ptr<ContextBase> Clone() const {
    std::unique_ptr<ContextBase> clone(CloneWithoutPointers());
    // Create a complete mapping of tracker pointers.
    DependencyTracker::PointerMap tracker_map;
    BuildTrackerPointerMap(*clone, &tracker_map);
    // Then do a pointer fixup pass.
    clone->FixTrackerPointers(*this, tracker_map);
    return clone;
  }

  /** (Internal use only) Clones a context but without any of its internal
  pointers. */
  std::unique_ptr<ContextBase> CloneWithoutPointers() const {
    std::unique_ptr<ContextBase> clone(DoCloneWithoutPointers());
    return clone;
  }

  virtual ~ContextBase() = default;

  /** Returns a const reference to the collection of value trackers within
  this subcontext. Together these form the dependency subgraph for the values
  in this subcontext, plus edges leading to neighboring trackers.  */
  const DependencyGraph& get_dependency_graph() const {
    return graph_;
  }

  /** Returns a mutable reference to the dependency graph. */
  DependencyGraph& get_mutable_dependency_graph() {
    return graph_;
  }

 protected:
  /** Default constructor creates an empty Context but initializes all the
  well-known dependency trackers that are the same in every System (like time,
  q, all states, all inputs, etc.). We can't allocate trackers for individual
  discrete & abstract states, parameters, or input ports since we don't yet
  know how many there are. */
  ContextBase() : graph_(this) {
    CreateWellKnownTrackers();
  }

  /** Copy constructor takes care of base class data members, but _does not_ fix
  up base class pointers. Derived classes must implement copy constructors that
  delegate to this one for use in their DoCloneWithoutPointers()
  implementations. The cache and dependency graph are copied, but the parent
  pointer, and internal tracker pointers, will be null in the copy. */
  // TODO(sherm1) Use reinit_after_copy<T> members to permit default constructor
  // here. Meanwhile, be very careful with these members.
  ContextBase(const ContextBase& source)
      : graph_(source.graph_) {
    CopyInTickets(source);
    // Everything else is left default-initialized.
  }

  /** Derived classes must implement this so that it performs the complete
  deep copy of the context, including all base class members but not fixing
  up base class pointers. To do that, implement a protected copy constructor
  that inherits from the base class copy constructor (which doesn't repair the
  pointers), then implement DoCloneWithoutPointers() as
  `new DerivedType(*this)`. */
  virtual ContextBase* DoCloneWithoutPointers() const = 0;

 private:
  // Fills in the dependency graph with the ubiquitous trackers.
  void CreateWellKnownTrackers();

  // Given a new context `clone` with the same structure as this one, create a
  // mapping of all tracker memory addresses from `this` to `clone`. The mapping
  // is flat, not hierarchical, because dependencies may cross levels.
  void BuildTrackerPointerMap(
      const ContextBase& clone,
      DependencyTracker::PointerMap* tracker_map) const {
    // First map the pointers local to this context.
    graph_.AppendToTrackerPointerMap(clone.get_dependency_graph(),
                                     &(*tracker_map));

    // TODO(sherm1) Subcontext descent stubbed out.
  }

  // Assuming `this` is a recently-cloned Context containing stale references
  // to the source Context's trackers, repair those pointers using the given
  // map.
  void FixTrackerPointers(const ContextBase& source,
                          const DependencyTracker::PointerMap& tracker_map) {
    // First repair pointers local to this context.
    graph_.RepairTrackerPointers(this, source.get_dependency_graph(),
                                 tracker_map);

    // TODO(sherm1) Subcontext descent stubbed out.
  }

  // TODO(sherm1) Handling of port, state, parameter tickets stubbed out.

  void CopyInTickets(const ContextBase& source) {
    // TODO(sherm1) Stubbed out.
    unused(source);
  }

  // TODO(sherm1) Cache goes here.
  // mutable Cache cache_;

  // This is the dependency graph for values within this subcontext.
  DependencyGraph graph_;
};

}  // namespace systems
}  // namespace drake
