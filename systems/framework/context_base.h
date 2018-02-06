#pragma once

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/unused.h"
#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/dependency_tracker.h"
#include "drake/systems/framework/input_port_value.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

/** Provides non-templatized functionality shared by the templatized derived
classes. That includes caching and dependency tracking, and management of
local values for fixed input ports. */
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

  /** (Debugging) Disables or re-enables caching, recursively for this context
  and all its subcontexts. Disabling forces every `Eval()` method to perform a
  full calculation rather than returning the cached one. Results should be
  identical with or without caching, except for performance. If they are not,
  there is likely a problem with (a) the specified dependencies for some
  calculation, or (b) a misuse of references into cached values that hides
  modifications from the caching system, or (c) a bug in the caching system. The
  `is_disabled` flag is independent of the `is_up_to_date` flag, which continues
  to be maintained even when caching is disabled (though it is ignored). Hence
  re-enabling the cache with this method may result in some entries already
  considered up to date. See SetAllCacheEntriesOutOfDate() if you want to ensure
  that caching starts with everything out of date. */
  void SetIsCacheDisabled(bool is_disabled) const {
    cache_.SetIsCacheDisabled(is_disabled);
    const int n = do_num_subcontexts();
    for (SubsystemIndex i(0); i < n; ++i)
      do_get_subcontext(i).SetIsCacheDisabled(is_disabled);
  }

  /** (Debugging) Marks all cache entries out-of-date, recursively for this
  context and all its subcontexts. This forces the next `Eval()` request for
  each cache entry to perform a full calculation rather than returning the
  cached one. After that, normal caching behavior resumes. Results should be
  identical whether this is called or not, since the caching system should be
  maintaining this flag correctly. If they are not, there is likely a problem
  with (a) the specified dependencies for some calculation, or (b) a misuse of
  references into cached values that hides modifications from the caching
  system, or (c) a bug in the caching system. */
  void SetAllCacheEntriesOutOfDate() const {
    cache_.SetAllEntriesOutOfDate();
    const int n = do_num_subcontexts();
    for (SubsystemIndex i(0); i < n; ++i)
      do_get_subcontext(i).SetAllCacheEntriesOutOfDate();
  }

  /** (Debugging) Returns the local name of the subsystem for which this is the
  Context. See GetSystemPathname() if you want to the full name. */
  std::string GetSystemName() const final { return system_name_; }

  /** (Debugging) Returns the full pathname of the subsystem for which this is
  the Context. See get_system_pathname() if you want to the full name. */
  std::string GetSystemPathname() const final;

  /** Returns a const reference to this subcontext's cache. */
  const Cache& get_cache() const {
    return cache_;
  }

  /** Returns a mutable reference to this subcontext's cache. Note that this
  method is const because the cache is always writable. Be careful. */
  Cache& get_mutable_cache() const {
    return cache_;
  }

  /** Returns a const reference to a DependencyTracker in this subcontext.
  Value change notifications can be delivered to a const tracker. */
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

  /** Returns the number of input ports represented in this context. */
  int get_num_input_ports() const {
    return static_cast<int>(input_port_tickets_.size());
  }

  /** Returns the number of output ports represented in this context. */
  int get_num_output_ports() const {
    return static_cast<int>(output_port_tickets_.size());
  }

  /** Returns the number of direct child subcontexts this context has.
  A leaf context will return 0. */
  int num_subcontexts() const { return do_num_subcontexts(); }

  /** Gets const access to a particular subcontext of this diagram context.
  The index must be in range [0..num_subsystems()-1]. */
  const ContextBase& get_subcontext(SubsystemIndex index) const {
    return do_get_subcontext(index);
  }

  /** Gets const access to a particular subcontext of this diagram context.
  The index must be in range [0..num_subsystems()-1]. */
  ContextBase& get_mutable_subcontext(SubsystemIndex index) {
    return const_cast<ContextBase&>(get_subcontext(index));
  }

  /** Returns a const pointer to the %ContextBase of the enclosing Diagram's
  Context, or `nullptr` if this is the root Context. */
  const ContextBase* get_parent_base() const { return parent_; }

  /** Returns the index of this subcontext within its parent supercontext.
  It is an error to call this on a root subcontext. */
  SubsystemIndex index_in_parent() const {
    DRAKE_ASSERT(!is_root_context());
    DRAKE_ASSERT(index_in_parent_.is_valid());
    return index_in_parent_;
  }

  /** Returns `true` if this subcontext is the root of the Context tree that
  it is in. If so, it has a null parent context. */
  bool is_root_context() const { return !parent_; }

  /** Returns a const reference to the %ContextBase of the root Context of the
  tree containing this subcontext. */
  // TODO(sherm1) Consider precalculating this for faster access.
  const ContextBase& get_root_context_base() const {
    const ContextBase* node = this;
    while (node->get_parent_base()) node = node->get_parent_base();
    return *node;
  }

  /** Returns a mutable reference to the root Context of the tree containing
  this subcontext. */
  ContextBase& get_mutable_root_context_base() {
    return const_cast<ContextBase&>(get_root_context_base());
  }

  /** Starts a new change event and returns the event number which is unique
  for this entire Context tree, not just this subcontext. */
  int64_t start_new_change_event() {
    return get_mutable_root_context_base()
        .increment_local_change_event_counter();
  }

  /** Force invalidation of any time-dependent computation. */
  void NoteTimeChanged(int64_t change_event) {
    get_tracker(DependencyTicket(internal::kTimeTicket))
        .NoteValueChange(change_event);
  }

  /** Force invalidation of any accuracy-dependent computation. */
  void NoteAccuracyChanged(int64_t change_event) {
    get_tracker(DependencyTicket(internal::kAccuracyTicket))
        .NoteValueChange(change_event);
  }

  /** Force invalidation of any state-dependent computation. */
  void NoteAllStateChanged(int64_t change_event) {
    NoteAllContinuousStateChanged(change_event);
    NoteAllDiscreteStateChanged(change_event);
    NoteAllAbstractStateChanged(change_event);
  }

  /** Force invalidation of any continuous state-dependent computation. */
  void NoteAllContinuousStateChanged(int64_t change_event) {
    NoteAllQChanged(change_event);
    NoteAllVChanged(change_event);
    NoteAllZChanged(change_event);
  }

  /** Force invalidation of any q-dependent computation. */
  void NoteAllQChanged(int64_t change_event) {
    get_tracker(DependencyTicket(internal::kQTicket))
        .NoteValueChange(change_event);
  }

  /** Force invalidation of any v-dependent computation. */
  void NoteAllVChanged(int64_t change_event) {
    get_tracker(DependencyTicket(internal::kVTicket))
        .NoteValueChange(change_event);
  }

  /** Force invalidation of any z-dependent computation. */
  void NoteAllZChanged(int64_t change_event) {
    get_tracker(DependencyTicket(internal::kZTicket))
        .NoteValueChange(change_event);
  }

  /** Force invalidation of any discrete state-dependent computation. */
  void NoteAllDiscreteStateChanged(int64_t change_event) {
    for (auto ticket : discrete_state_tickets_)
      get_tracker(ticket).NoteValueChange(change_event);
  }

  /** Force invalidation of any abstract state-dependent computation. */
  void NoteAllAbstractStateChanged(int64_t change_event) {
    for (auto ticket : abstract_state_tickets_)
      get_tracker(ticket).NoteValueChange(change_event);
  }

  /** Force invalidation of any parameter-dependent computation. */
  void NoteAllParametersChanged(int64_t change_event) {
    NoteAllNumericParametersChanged(change_event);
    NoteAllAbstractParametersChanged(change_event);
  }

  /** Force invalidation of any numeric parameter-dependent computation. */
  void NoteAllNumericParametersChanged(int64_t change_event) {
    for (auto ticket : numeric_parameter_tickets_)
      get_tracker(ticket).NoteValueChange(change_event);
  }

  /** Force invalidation of any abstract parameter-dependent computation. */
  void NoteAllAbstractParametersChanged(int64_t change_event) {
    for (auto ticket : abstract_parameter_tickets_)
      get_tracker(ticket).NoteValueChange(change_event);
  }

  /** Connects the input port at `index` to a FreestandingInputPortValue with
  the given abstract `value`. Asserts if `index` is out of range.Returns a
  reference to the allocated FreestandingInputPortValue that will remain valid
  until this input port's value source is replaced or the Context is destroyed.
  You may use that reference to modify the input port's value using the
  appropriate FreestandingInputPortValue method, which will ensure that
  invalidation notifications are delivered.

  This is the most general way to provide a value (type-erased) for an
  unconnected input port. See `Context<T>` for more-convenient overloads of
  FixInputPort() for vector values with elements of type T. */
  FreestandingInputPortValue& FixInputPort(
      int index, std::unique_ptr<AbstractValue> value) {
    auto freestanding =
        std::make_unique<FreestandingInputPortValue>(std::move(value));
    FreestandingInputPortValue& freestanding_ref = *freestanding;
    SetFixedInputPortValue(InputPortIndex(index), std::move(freestanding));
    return freestanding_ref;
  }

  // These are for internal use only.
  #if !defined(DRAKE_DOXYGEN_CXX)
  // Declares that `parent` is the context of the enclosing
  // Diagram. Aborts if the parent has already been set to something else.
  void set_parent(const ContextBase* parent, SubsystemIndex index) {
    DRAKE_DEMAND(parent_ == nullptr || parent_ == parent);
    parent_ = parent;
    index_in_parent_ = index;
  }

  // Add the next input port. Expected index is supplied along with the
  // assigned ticket. Subscribe the "all input ports" tracker to this one.
  void AddInputPort(InputPortIndex expected_index, DependencyTicket ticket);


  // For input port `index`, returns the FreestandingInputPortValue if the
  // port is freestanding, otherwise nullptr. Asserts if `index` is out of
  // range.
  const FreestandingInputPortValue* GetInputPortValue(
      InputPortIndex index) const {
    DRAKE_DEMAND(0 <= index && index < get_num_input_ports());
    return input_port_values_[index].get();
  }

  // Records the name of the system whose context this is.
  void set_system_name(const std::string& name) { system_name_ = name; }

  // These provide access to the memorized tickets for various resources.
  // These are set by SystemBase::AllocateContext().

  std::vector<DependencyTicket>& input_port_tickets() {
    return input_port_tickets_;
  }
  std::vector<DependencyTicket>& output_port_tickets() {
    return output_port_tickets_;
  }
  std::vector<DependencyTicket>& discrete_state_tickets() {
    return discrete_state_tickets_;
  }
  std::vector<DependencyTicket>& abstract_state_tickets() {
    return abstract_state_tickets_;
  }
  std::vector<DependencyTicket>& numeric_parameter_tickets() {
    return numeric_parameter_tickets_;
  }
  std::vector<DependencyTicket>& abstract_parameter_tickets() {
    return abstract_parameter_tickets_;
  }
  #endif

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
      : cache_(source.cache_), graph_(source.graph_) {
    CopyInTickets(source);
    CopyInFixedInputs(source);
    // Everything else is left default-initialized.
  }

  /** (Internal use only) */
  // Gets the value of change event counter stored in this subcontext. This
  // should only be used when this subcontext is serving as root.
  int64_t get_local_change_event_counter() const {
    return current_change_event_;
  }

  /** (Internal use only) */
  // Increments the change event counter for this subcontext and returns the
  // new value. This should only be used when this subcontext is serving
  // as root.
  int64_t increment_local_change_event_counter() {
    return ++current_change_event_;
  }

  /** Derived classes must implement this so that it performs the complete
  deep copy of the context, including all base class members but not fixing
  up base class pointers. To do that, implement a protected copy constructor
  that inherits from the base class copy constructor (which doesn't repair the
  pointers), then implement DoCloneWithoutPointers() as
  `new DerivedType(*this)`. */
  virtual ContextBase* DoCloneWithoutPointers() const = 0;

  /** DiagramContext must override this to return the actual number
  of immediate child subcontexts it contains. The default is 0. */
  virtual int do_num_subcontexts() const { return 0; }

  /** DiagramContext must override this to provide access to its contained
  subcontexts. The default implementation throws a logic error. */
  virtual const ContextBase& do_get_subcontext(
      SubsystemIndex index) const {
    unused(index);
    throw std::logic_error(
        "ContextBase::do_get_subcontext: called on a leaf context.");
  }

 private:
  // Fixes the input port at `index` to the internal value source `port_value`.
  // If the port wasn't previously fixed, assigns a ticket and tracker for the
  // `port_value`, then subscribes the input port to the source's tracker.
  // If the port was already fixed, we just use the existing tracker and
  // subscription but replace the value. Notifies the port's downstream
  // subscribers that the value has changed. Aborts if `index` is out of range,
  // or the given `port_value` is null or already belongs to a context.
  void SetFixedInputPortValue(
      InputPortIndex index,
      std::unique_ptr<FreestandingInputPortValue> port_value);

  // Fills in the dependency graph with the ubiquitous trackers.
  void CreateWellKnownTrackers();

  // Given a new context `clone` with the same dependency graph as this one,
  // create a mapping of all tracker memory addresses from `this` to `clone`.
  // The mapping is flat, not hierarchical, because dependencies may cross
  // levels.
  void BuildTrackerPointerMap(
      const ContextBase& clone,
      DependencyTracker::PointerMap* tracker_map) const {
    // First map the pointers local to this context.
    graph_.AppendToTrackerPointerMap(clone.get_dependency_graph(),
                                     &(*tracker_map));
    // Then recursively ask our descendants to add their information to the map.
    for (SubsystemIndex i(0); i < num_subcontexts(); ++i)
      get_subcontext(i).BuildTrackerPointerMap(clone.get_subcontext(i),
                                               &(*tracker_map));
  }

  // Assuming `this` is a recently-cloned Context containing stale references
  // to the source Context's trackers, repair those pointers using the given
  // map.
  void FixTrackerPointers(const ContextBase& source,
                          const DependencyTracker::PointerMap& tracker_map) {
    // First repair pointers local to this context.
    graph_.RepairTrackerPointers(source.get_dependency_graph(),
                                 tracker_map, this, &cache_);
    // Then recursively ask our descendants to repair their pointers.
    for (SubsystemIndex i(0); i < num_subcontexts(); ++i)
      get_mutable_subcontext(i).FixTrackerPointers(source.get_subcontext(i),
                                                   tracker_map);
  }

  void CopyInTickets(const ContextBase& source) {
    input_port_tickets_         = source.input_port_tickets_;
    output_port_tickets_        = source.output_port_tickets_;
    discrete_state_tickets_     = source.discrete_state_tickets_;
    abstract_state_tickets_     = source.abstract_state_tickets_;
    numeric_parameter_tickets_  = source.numeric_parameter_tickets_;
    abstract_parameter_tickets_ = source.abstract_parameter_tickets_;
  }

  // FreestandingInputPortValue contains a backpointer to the ContextBase which
  // must be replaced when copying.
  void CopyInFixedInputs(const ContextBase& source) {
    DRAKE_DEMAND(input_port_values_.empty());
    const int n = source.get_num_input_ports();
    input_port_values_.reserve(n);
    for (InputPortIndex i(0); i < n; ++i) {
      const FreestandingInputPortValue* value = source.GetInputPortValue(i);
      if (value == nullptr)
        input_port_values_.emplace_back(nullptr);
      else
        input_port_values_.emplace_back(value->CloneForNewContext(this, i));
    }
  }

  // We record tickets so we can reconstruct the dependency graph when cloning
  // or transmogrifying a Context without a System present.

  // Index by InputPortIndex.
  std::vector<DependencyTicket> input_port_tickets_;
  // Index by OutputPortIndex.
  std::vector<DependencyTicket> output_port_tickets_;
  // Index by DiscreteStateIndex.
  std::vector<DependencyTicket> discrete_state_tickets_;
  // Index by AbstractStateIndex.
  std::vector<DependencyTicket> abstract_state_tickets_;
  // Index by NumericParameterIndex.
  std::vector<DependencyTicket> numeric_parameter_tickets_;
  // Index by AbstractParameterIndex.
  std::vector<DependencyTicket> abstract_parameter_tickets_;

  // For each input port, the fixed value or null if the port is connected to
  // something else (in which case we need System help to get the value).
  // Semantically, these are identical to Parameters.
  // Each non-null FreestandingInputPortValue has a ticket and associated
  // tracker.
  // Index with InputPortIndex.
  std::vector<std::unique_ptr<FreestandingInputPortValue>>
      input_port_values_;

  // The cache of pre-computed values owned by this subcontext.
  mutable Cache cache_;

  // This is the dependency graph for values within this subcontext.
  DependencyGraph graph_;

  // This is used only when this subcontext is serving as the root
  // of a context tree.
  int64_t current_change_event_{0};

  // The Context of the enclosing Diagram, and the index of this subcontext
  // within that Diagram. Null/invalid when this is the root context.
  const ContextBase* parent_{nullptr};
  SubsystemIndex index_in_parent_;

  // Name of the subsystem whose subcontext this is.
  std::string system_name_;
};

}  // namespace systems
}  // namespace drake
