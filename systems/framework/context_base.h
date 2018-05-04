#pragma once

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/reset_on_copy.h"
#include "drake/common/unused.h"
#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/dependency_tracker.h"
#include "drake/systems/framework/fixed_input_port_value.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

#ifndef DRAKE_DOXYGEN_CXX
namespace detail {
// This provides SystemBase limited "friend" access to ContextBase.
class SystemBaseContextBaseAttorney;
}  // namespace detail
#endif

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
class ContextBase : public internal::ContextMessageInterface {
 public:
  /** @name     Does not allow copy, move, or assignment. */
  /** @{ */
  // Copy constructor is used only to facilitate implementation of Clone()
  // in derived classes.
  ContextBase(ContextBase&&) = delete;
  ContextBase& operator=(const ContextBase&) = delete;
  ContextBase& operator=(ContextBase&&) = delete;
  /** @} */

  /** Creates an identical copy of the concrete context object. */
  std::unique_ptr<ContextBase> Clone() const;

  ~ContextBase() override;

  /** (Debugging) Disables caching recursively for this context
  and all its subcontexts. Disabling forces every `Eval()` method to perform a
  full calculation rather than returning the cached one. Results should be
  identical with or without caching, except for performance. If they are not,
  there is likely a problem with (a) the specified dependencies for some
  calculation, or (b) a misuse of references into cached values that hides
  modifications from the caching system, or (c) a bug in the caching system. The
  `is_disabled` flags are independent of the `out_of_date` flags, which continue
  to be maintained even when caching is disabled (though they are ignored). */
  void DisableCaching() const;

  /** (Debugging) Re-enables caching recursively for this context and all its
  subcontexts. The `is_disabled` flags are independent of the `out_of_date`
  flags, which continue to be maintained even when caching is disabled (though
  they are ignored). Hence re-enabling the cache with this method may result in
  some entries being already considered up to date. See
  SetAllCacheEntriesOutOfDate() if you want to ensure that caching restarts with
  everything out of date. You might want to do that, for example, for
  repeatability or because you modified something in the debugger and want to
  make sure it gets used. */
  void EnableCaching() const;

  /** (Debugging) Marks all cache entries out of date, recursively for this
  context and all its subcontexts. This forces the next `Eval()` request for
  each cache entry to perform a full calculation rather than returning the
  cached one. After that first recalculation, normal caching behavior resumes
  (assuming the cache is not disabled). Results should be identical whether this
  is called or not, since the caching system should be maintaining this flag
  correctly. If they are not, see the documentation for SetIsCacheDisabled() for
  suggestions. */
  void SetAllCacheEntriesOutOfDate() const;

  /** Returns the local name of the subsystem for which this is the Context.
  This is intended primarily for error messages and logging.
  @see SystemBase::GetSystemName() for details.
  @see GetSystemPathname() if you want the full name. */
  const std::string& GetSystemName() const final {
    return system_name_.empty() ? internal::SystemMessageInterface::no_name()
                                : system_name_;
  }

  /** Returns the full pathname of the subsystem for which this is the Context.
  This is intended primarily for error messages and logging.
  @see SystemBase::GetSystemPathname() for details. */
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

  /** Returns the number of input ports in this context. */
  int get_num_input_ports() const {
    DRAKE_ASSERT(input_port_tickets_.size() == input_port_values_.size());
    return static_cast<int>(input_port_tickets_.size());
  }

  /** Connects the input port at `index` to a FixedInputPortValue with
  the given abstract `value`. Returns a reference to the allocated
  FixedInputPortValue that will remain valid until this input port's value
  source is replaced or the Context is destroyed. You may use that reference to
  modify the input port's value using the appropriate FixedInputPortValue
  method, which will ensure that invalidation notifications are delivered.

  This is the most general way to provide a value (type-erased) for an
  unconnected input port. See `Context<T>` for more-convenient overloads of
  FixInputPort() for vector values with elements of type T.

  @pre `index` selects an existing input port of this Context. */
  FixedInputPortValue& FixInputPort(
      int index, std::unique_ptr<AbstractValue> value);

  /** For input port `index`, returns a const FixedInputPortValue if the port is
  fixed, otherwise nullptr.
  @pre `index` selects an existing input port of this Context. */
  const FixedInputPortValue* MaybeGetFixedInputPortValue(int index) const {
    DRAKE_DEMAND(0 <= index && index < get_num_input_ports());
    return input_port_values_[index].get();
  }

  /** For input port `index`, returns a mutable FixedInputPortValue if the port
  is fixed, otherwise nullptr.
  @pre `index` selects an existing input port of this Context. */
  FixedInputPortValue* MaybeGetMutableFixedInputPortValue(int index) {
    DRAKE_DEMAND(0 <= index && index < get_num_input_ports());
    return input_port_values_[index].get_mutable();
  }

  // For internal use only.
#if !defined(DRAKE_DOXYGEN_CXX)

  // Add the next input port. Expected index is supplied along with the
  // assigned ticket. Subscribe the "all input ports" tracker to this one.
  void AddInputPort(InputPortIndex expected_index, DependencyTicket ticket);

#endif

 protected:
  /** Default constructor creates an empty ContextBase but initializes all the
  built-in dependency trackers that are the same in every System (like time,
  q, all states, all inputs, etc.). We can't allocate trackers for individual
  discrete & abstract states, parameters, or input ports since we don't yet
  know how many there are. */
  ContextBase() : cache_(this), graph_(this) {
    CreateBuiltInTrackers();
  }

  /** Copy constructor takes care of base class data members, but _does not_ fix
  up base class pointers. Derived classes must implement copy constructors that
  delegate to this one for use in their DoCloneWithoutPointers()
  implementations. The cache and dependency graph are copied, but any pointers
  contained in the source are left null in the copy. */
  ContextBase(const ContextBase&) = default;

  /** Clones a context but without copying any of its internal pointers; the
  clone's pointers are set to null. */
  // Structuring this as a static method allows DiagramContext to invoke this
  // protected function on its children.
  static std::unique_ptr<ContextBase> CloneWithoutPointers(
      const ContextBase& source) {
    return source.DoCloneWithoutPointers();
  }

  /** Declares that `parent` is the context of the enclosing Diagram.
  Aborts if the parent has already been set to something else. */
  // Use static method so DiagramContext can invoke this on behalf of a child.
  // Output argument is listed first because it is serving as the 'this'
  // pointer here.
  static void set_parent(ContextBase* child, const ContextBase* parent) {
    DRAKE_DEMAND(child != nullptr);
    child->set_parent(parent);
  }

  /** Derived classes must implement this so that it performs the complete
  deep copy of the context, including all base class members but not fixing
  up base class pointers. To do that, implement a protected copy constructor
  that inherits from the base class copy constructor (which doesn't repair the
  pointers), then implement DoCloneWithoutPointers() as
  `return unique_ptr<ContextBase>(new DerivedType(*this));`. */
  virtual std::unique_ptr<ContextBase> DoCloneWithoutPointers() const = 0;

 private:
  friend class detail::SystemBaseContextBaseAttorney;

  void set_parent(const ContextBase* parent) {
    DRAKE_DEMAND(parent_ == nullptr || parent_ == parent);
    parent_ = parent;
  }

  // Returns the parent Context or `nullptr` if this is the root Context.
  const ContextBase* get_parent_base() const { return parent_; }

  // Records the name of the system whose context this is.
  void set_system_name(const std::string& name) { system_name_ = name; }

  // Fixes the input port at `index` to the internal value source `port_value`.
  // If the port wasn't previously fixed, assigns a ticket and tracker for the
  // `port_value`, then subscribes the input port to the source's tracker.
  // If the port was already fixed, we just use the existing tracker and
  // subscription but replace the value. Notifies the port's downstream
  // subscribers that the value has changed. Aborts if `index` is out of range,
  // or the given `port_value` is null or already belongs to a context.
  void SetFixedInputPortValue(
      InputPortIndex index,
      std::unique_ptr<FixedInputPortValue> port_value);

  // Fills in the dependency graph with the built-in trackers that are common
  // to every Context (and every System).
  void CreateBuiltInTrackers();

  // Given a new context `clone` with the same dependency graph as this one,
  // create a mapping of all tracker memory addresses from `this` to `clone`.
  // This must be done for the whole Context tree because pointers can point
  // outside of their containing subcontext.
  void BuildTrackerPointerMap(
      const ContextBase& clone,
      DependencyTracker::PointerMap* tracker_map) const;

  // Assuming `this` is a recently-cloned Context that has yet to have its
  // internal pointers updated, set those pointers now. The given map is used
  // to update tracker pointers.
  void FixContextPointers(const ContextBase& source,
                          const DependencyTracker::PointerMap& tracker_map);

  // TODO(sherm1) Use these tickets to reconstruct the dependency graph when
  // cloning or transmogrifying a Context without a System present.

  // Index by InputPortIndex.
  std::vector<DependencyTicket> input_port_tickets_;

  // TODO(sherm1) Output port, state, and parameter tickets go here.

  // For each input port, the fixed value or null if the port is connected to
  // something else (in which case we need System help to get the value).
  // Semantically, these are identical to Parameters.
  // Each non-null FixedInputPortValue has a ticket and associated
  // tracker.
  // Index with InputPortIndex.
  std::vector<copyable_unique_ptr<FixedInputPortValue>>
      input_port_values_;

  // The cache of pre-computed values owned by this subcontext.
  mutable Cache cache_;

  // This is the dependency graph for values within this subcontext.
  DependencyGraph graph_;

  // The Context of the enclosing Diagram. Null/invalid when this is the root
  // context.
  reset_on_copy<const ContextBase*> parent_;

  // Name of the subsystem whose subcontext this is.
  std::string system_name_;
};

#ifndef DRAKE_DOXYGEN_CXX
class SystemBase;
namespace detail {

// This is an attorney-client pattern class providing SystemBase with access to
// certain specific ContextBase private methods, and nothing else.
class SystemBaseContextBaseAttorney {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SystemBaseContextBaseAttorney);
  SystemBaseContextBaseAttorney() = delete;

 private:
  friend class drake::systems::SystemBase;
  static void set_system_name(ContextBase* context, const std::string& name) {
    DRAKE_DEMAND(context != nullptr);
    context->set_system_name(name);
  }
  static const ContextBase* get_parent_base(const ContextBase& context) {
    return context.get_parent_base();
  }
};

}  // namespace detail
#endif

}  // namespace systems
}  // namespace drake
