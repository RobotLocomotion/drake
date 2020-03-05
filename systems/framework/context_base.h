#pragma once

#include <algorithm>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/reset_on_copy.h"
#include "drake/common/unused.h"
#include "drake/common/value.h"
#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/dependency_tracker.h"
#include "drake/systems/framework/fixed_input_port_value.h"

namespace drake {
namespace systems {

#ifndef DRAKE_DOXYGEN_CXX
namespace internal {
// This provides SystemBase limited "friend" access to ContextBase.
class SystemBaseContextBaseAttorney;
}  // namespace internal
#endif

/** Provides non-templatized Context functionality shared by the templatized
derived classes. That includes caching, dependency tracking, and management
of local values for fixed input ports.

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

  /** Creates an identical copy of the concrete context object.
  @throws std::logic_error if this is not the root context. */
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
  void DisableCaching() const {
    PropagateCachingChange(*this, &Cache::DisableCaching);
  }

  /** (Debugging) Re-enables caching recursively for this context and all its
  subcontexts. The `is_disabled` flags are independent of the `out_of_date`
  flags, which continue to be maintained even when caching is disabled (though
  they are ignored). Hence re-enabling the cache with this method may result in
  some entries being already considered up to date. See
  SetAllCacheEntriesOutOfDate() if you want to ensure that caching restarts with
  everything out of date. You might want to do that, for example, for
  repeatability or because you modified something in the debugger and want to
  make sure it gets used. */
  void EnableCaching() const {
    PropagateCachingChange(*this, &Cache::EnableCaching);
  }

  /** (Debugging) Marks all cache entries out of date, recursively for this
  context and all its subcontexts. This forces the next `Eval()` request for
  each cache entry to perform a full calculation rather than returning the
  cached one. After that first recalculation, normal caching behavior resumes
  (assuming the cache is not disabled). Results should be identical whether this
  is called or not, since the caching system should be maintaining this flag
  correctly. If they are not, see the documentation for SetIsCacheDisabled() for
  suggestions. */
  void SetAllCacheEntriesOutOfDate() const {
    PropagateCachingChange(*this, &Cache::SetAllEntriesOutOfDate);
  }

  /** (Advanced) Freezes the cache at its current contents, preventing any
  further cache updates. When frozen, accessing an out-of-date cache entry
  causes an exception to be throw. This is applied recursively to this
  %Context and all its subcontexts, but _not_ to its parent or siblings so
  it is most useful when called on the root %Context. If the cache was already
  frozen this method does nothing but waste a little time. */
  void FreezeCache() const {
    PropagateCachingChange(*this, &Cache::freeze_cache);
  }

  /** (Advanced) Unfreezes the cache if it was previously frozen. This is
  applied recursively to this %Context and all its subcontexts, but _not_
  to its parent or siblings. If the cache was not frozen, this does nothing
  but waste a little time. */
  void UnfreezeCache() const {
    PropagateCachingChange(*this, &Cache::unfreeze_cache);
  }

  /** (Advanced) Reports whether this %Context's cache is currently frozen.
  This checks only locally; it is possible that parent, child, or sibling
  subcontext caches are in a different state than this one. */
  bool is_cache_frozen() const final {
    return get_cache().is_cache_frozen();
  }

  /** Returns the local name of the subsystem for which this is the Context.
  This is intended primarily for error messages and logging.
  @see SystemBase::GetSystemName() for details.
  @see GetSystemPathname() if you want the full name. */
  const std::string& GetSystemName() const final {
    return system_name_.empty() ? internal::SystemMessageInterface::no_name()
                                : system_name_;
  }

  /** (Internal) Gets the id of the subsystem that created this context. */
  internal::SystemId get_system_id() const { return system_id_; }

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
  int num_input_ports() const {
    DRAKE_ASSERT(input_port_tickets_.size() == input_port_values_.size());
    return static_cast<int>(input_port_tickets_.size());
  }

  /** Returns the number of output ports represented in this context. */
  int num_output_ports() const {
    return static_cast<int>(output_port_tickets_.size());
  }

  /** Returns the dependency ticket associated with a particular input port. */
  DependencyTicket input_port_ticket(InputPortIndex port_num) {
    DRAKE_DEMAND(port_num < num_input_ports());
    return input_port_tickets_[port_num];
  }

  /** Returns the dependency ticket associated with a particular output port. */
  DependencyTicket output_port_ticket(OutputPortIndex port_num) {
    DRAKE_DEMAND(port_num < num_output_ports());
    return output_port_tickets_[port_num];
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

  @note Calling this method on an already connected input port, i.e., an
  input port that has previously been passed into a call to
  DiagramBuilder::Connect(), causes FixedInputPortValue to override any other
  value present on that port.

  @pre `index` selects an existing input port of this Context. */
  FixedInputPortValue& FixInputPort(
      int index, std::unique_ptr<AbstractValue> value);

  /** Same as above method but the value is passed by const reference instead
  of by unique_ptr. The port will contain a copy of the `value` (not retain a
  pointer to the `value`).

  @note Calling this method on an already connected input port, i.e., an
  input port that has previously been passed into a call to
  DiagramBuilder::Connect(), causes FixedInputPortValue to override any other
  value present on that port.

  @exclude_from_pydrake_mkdoc{The prior overload's docstring is better, and we
  only need one of the two -- overloading on ownership doesn't make sense for
  pydrake.} */
  FixedInputPortValue& FixInputPort(int index, const AbstractValue& value) {
    return FixInputPort(index, value.Clone());
  }

  /** For input port `index`, returns a const FixedInputPortValue if the port is
  fixed, otherwise nullptr.
  @pre `index` selects an existing input port of this Context. */
  const FixedInputPortValue* MaybeGetFixedInputPortValue(int index) const {
    DRAKE_DEMAND(0 <= index && index < num_input_ports());
    return input_port_values_[index].get();
  }

  /** For input port `index`, returns a mutable FixedInputPortValue if the port
  is fixed, otherwise nullptr.
  @pre `index` selects an existing input port of this Context. */
  FixedInputPortValue* MaybeGetMutableFixedInputPortValue(int index) {
    DRAKE_DEMAND(0 <= index && index < num_input_ports());
    return input_port_values_[index].get_mutable();
  }

  /** (Internal use only) Returns the next change event serial number that is
  unique for this entire Context tree, not just this subcontext. This number
  is not reset after a Context is copied but continues to count up. */
  int64_t start_new_change_event() {
    // First search up to find the root Context (typically not far).
    // TODO(sherm1) Consider precalculating this for faster access.
    ContextBase* context = this;
    while (context->parent_) {
      // Only a root context has a non-negative change event value.
      DRAKE_ASSERT(context->current_change_event_ == -1);
      context = context->parent_;
    }
    DRAKE_ASSERT(context->current_change_event_ >= 0);
    return ++context->current_change_event_;
  }

  /** Returns true if this context has no parent. */
  bool is_root_context() const { return parent_ == nullptr; }

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

  /** @name      Add dependency tracking resources (Internal use only)
  Methods in this group are used by SystemBase and unit testing while creating
  a Context that can track dependencies for a given System. Although these
  methods are protected, SystemBase is granted permission to invoke them (via
  an attorney class). */
  //@{

  /** Adds the next input port. Expected index is supplied along with the
  assigned ticket. Subscribes the "all input ports" tracker to this one.
  The fixed_input_type_checker will be used for validation when setting a fixed
  input, or may be null when no validation should be performed.  Typically the
  fixed_input_type_checker is created by System::MakeFixInputPortTypeChecker.
  The fixed_input_type_checker lifetime will be the same as this ContextBase,
  so it should not depend on pointers that may go out of scope.  Most acutely,
  the function must not depend on any captured SystemBase pointers. */
  void AddInputPort(
      InputPortIndex expected_index, DependencyTicket ticket,
      std::function<void(const AbstractValue&)> fixed_input_type_checker);

  /** Adds the next output port. Expected index is supplied along with the
  assigned ticket. */
  void AddOutputPort(
      OutputPortIndex expected_index, DependencyTicket ticket,
      const internal::OutputPortPrerequisite& prerequisite);

  /** Adds a ticket to the list of discrete state tickets. */
  void AddDiscreteStateTicket(DependencyTicket ticket) {
    discrete_state_tickets_.push_back(ticket);
  }

  /** Adds a ticket to the list of abstract state tickets. */
  void AddAbstractStateTicket(DependencyTicket ticket) {
    abstract_state_tickets_.push_back(ticket);
  }

  /** Adds a ticket to the list of numeric parameter tickets. */
  void AddNumericParameterTicket(DependencyTicket ticket) {
    numeric_parameter_tickets_.push_back(ticket);
  }

  /** Adds a ticket to the list of abstract parameter tickets. */
  void AddAbstractParameterTicket(DependencyTicket ticket) {
    abstract_parameter_tickets_.push_back(ticket);
  }
  //@}

  /// @anchor context_base_change_notification_methods
  /** @name         Change notification methods (Internal use only)
  These "Note" methods are used by framework-internal derived classes to effect
  change notifications that propagate down from a DiagramContext (where the
  change is initiated) through all its subcontexts, recursively. Such
  notification sweeps result in the "out of date" flag being set in each of
  the affected cache entry values. Each of these "Note" methods methods affects
  only the local context, but all have identical signatures so can be passed
  down the context tree to operate on every subcontext. The `change_event`
  argument should be the result of the start_new_change_event() method. */
  //@{

  /** Notifies the local time tracker that time may have changed. */
  void NoteTimeChanged(int64_t change_event) {
    get_tracker(DependencyTicket(internal::kTimeTicket))
        .NoteValueChange(change_event);
  }

  /** Notifies the local accuracy tracker that the accuracy setting
  may have changed. */
  void NoteAccuracyChanged(int64_t change_event) {
    get_tracker(DependencyTicket(internal::kAccuracyTicket))
        .NoteValueChange(change_event);
  }

  /** Notifies the local continuous, discrete, and abstract state trackers that
  each of them may have changed, likely because someone has asked to modify the
  whole state x. */
  void NoteAllStateChanged(int64_t change_event) {
    NoteAllContinuousStateChanged(change_event);
    NoteAllDiscreteStateChanged(change_event);
    NoteAllAbstractStateChanged(change_event);
  }

  /** Notifies the local q, v, and z trackers that each of them may have
  changed, likely because someone has asked to modify continuous state xc. */
  void NoteAllContinuousStateChanged(int64_t change_event) {
    NoteAllQChanged(change_event);
    NoteAllVZChanged(change_event);
  }

  /** Notifies the local v and z trackers that each of them may have
  changed, likely because someone has asked to modify just the first-order
  state variables in xc. */
  void NoteAllVZChanged(int64_t change_event) {
    NoteAllVChanged(change_event);
    NoteAllZChanged(change_event);
  }

  /** Notifies the local q tracker that the q's may have changed. */
  void NoteAllQChanged(int64_t change_event) {
    get_tracker(DependencyTicket(internal::kQTicket))
        .NoteValueChange(change_event);
  }

  /** Notifies the local v tracker that the v's may have changed. */
  void NoteAllVChanged(int64_t change_event) {
    get_tracker(DependencyTicket(internal::kVTicket))
        .NoteValueChange(change_event);
  }

  /** Notifies the local z tracker that the z's may have changed. */
  void NoteAllZChanged(int64_t change_event) {
    get_tracker(DependencyTicket(internal::kZTicket))
        .NoteValueChange(change_event);
  }

  /** Notifies each local discrete state group tracker that the value of
  the discrete state group it manages may have changed. If there are no discrete
  state groups owned by this context, nothing happens. A DiagramContext does
  not own any discrete state groups. */
  void NoteAllDiscreteStateChanged(int64_t change_event) {
    for (auto ticket : discrete_state_tickets_)
      get_tracker(ticket).NoteValueChange(change_event);
  }

  /** Notifies each local abstract state variable tracker that the value of the
  abstract state variable it manages may have changed. If there are no abstract
  state variables owned by this context, nothing happens. A DiagramContext does
  not own any abstract state variables. */
  void NoteAllAbstractStateChanged(int64_t change_event) {
    for (auto ticket : abstract_state_tickets_)
      get_tracker(ticket).NoteValueChange(change_event);
  }

  /** Notifies the local numeric and abstract parameter trackers that each of
  them may have changed, likely because someone asked to modify all the
  parameters. */
  void NoteAllParametersChanged(int64_t change_event) {
    NoteAllNumericParametersChanged(change_event);
    NoteAllAbstractParametersChanged(change_event);
  }

  /** Notifies each local numeric parameter tracker that the value of the
  parameter it manages may have changed. If there are no numeric parameters
  owned by this context, nothing happens. A DiagramContext does not own any
  parameters. */
  void NoteAllNumericParametersChanged(int64_t change_event) {
    for (auto ticket : numeric_parameter_tickets_)
      get_tracker(ticket).NoteValueChange(change_event);
  }

  /** Notifies each local abstract parameter tracker that the value of the
  parameter it manages may have changed. If there are no abstract parameters
  owned by this context, nothing happens. A DiagramContext does not own any
  parameters. */
  void NoteAllAbstractParametersChanged(int64_t change_event) {
    for (auto ticket : abstract_parameter_tickets_)
      get_tracker(ticket).NoteValueChange(change_event);
  }
  //@}

  /** (Internal use only) Returns true if this context provides resources for
  its own individual state variables or parameters. That means those variables
  or parameters were declared by this context's corresponding System. Currently
  only leaf systems may declare variables and parameters; diagram contexts
  can use this method to check that invariant. */
  bool owns_any_variables_or_parameters() const {
    return !(discrete_state_tickets_.empty() &&
             abstract_state_tickets_.empty() &&
             numeric_parameter_tickets_.empty() &&
             abstract_parameter_tickets_.empty());
  }

  /** (Internal use only) Clones a context but without copying any of its
  internal pointers; the clone's pointers are set to null. */
  // Structuring this as a static method allows DiagramContext to invoke this
  // protected function on its children.
  static std::unique_ptr<ContextBase> CloneWithoutPointers(
      const ContextBase& source) {
    std::unique_ptr<ContextBase> result = source.DoCloneWithoutPointers();

    // Verify that the most-derived Context didn't forget to override
    // DoCloneWithoutPointers().
    ContextBase& clone = *result;
    DRAKE_THROW_UNLESS(typeid(source) == typeid(clone));

    return result;
  }

  /** (Internal use only) Given a new context `clone` containing an
  identically-structured dependency graph as the one in `source`, creates a
  mapping of all tracker memory addresses from `source` to `clone`. This must be
  done for the whole Context tree because pointers can point outside of their
  containing subcontext. */
  // Structuring this as a static method allows DiagramContext to invoke this
  // protected function on its children.
  static void BuildTrackerPointerMap(
      const ContextBase& source, const ContextBase& clone,
      DependencyTracker::PointerMap* tracker_map);

  /** (Internal use only) Assuming `clone` is a recently-cloned Context that
  has yet to have its internal pointers updated, sets those pointers now. The
  given map is used to update tracker pointers. */
  // Structuring this as a static method allows DiagramContext to invoke this
  // protected function on its children.
  static void FixContextPointers(
      const ContextBase& source,
      const DependencyTracker::PointerMap& tracker_map,
      ContextBase* clone);

  /** (Internal use only) Applies the given caching-change notification method
  to `context`, and propagates the notification to subcontexts if `context` is
  a DiagramContext. Used, for example, to enable and disable the cache. The
  supplied `context` is const so depends on the cache being mutable. */
  // Structuring this as a static method allows DiagramContext to invoke this
  // protected method on its children.
  static void PropagateCachingChange(const ContextBase& context,
                                     void (Cache::*caching_change)()) {
    (context.get_mutable_cache().*caching_change)();
    context.DoPropagateCachingChange(caching_change);
  }

  /** (Internal use only) Applies the given bulk-change notification method
  to the given `context`, and propagates the notification to subcontexts if this
  is a DiagramContext. */
  // Structuring this as a static method allows DiagramContext to invoke this
  // protected method on its children.
  static void PropagateBulkChange(
      ContextBase* context, int64_t change_event,
      void (ContextBase::*note_bulk_change)(int64_t change_event)) {
    (context->*note_bulk_change)(change_event);
    context->DoPropagateBulkChange(change_event, note_bulk_change);
  }

  /** (Internal use only) This is a convenience method for invoking the
  eponymous static method on `this` context (which occurs frequently). */
  void PropagateBulkChange(
      int64_t change_event,
      void (ContextBase::*note_bulk_change)(int64_t change_event)) {
    PropagateBulkChange(this, change_event, note_bulk_change);
  }

  /** Declares that `parent` is the context of the enclosing Diagram.
  Aborts if the parent has already been set or is null. */
  // Use static method so DiagramContext can invoke this on behalf of a child.
  // Output argument is listed first because it is serving as the 'this'
  // pointer here.
  static void set_parent(ContextBase* child, ContextBase* parent) {
    DRAKE_DEMAND(child != nullptr);
    DRAKE_DEMAND(parent != nullptr);
    DRAKE_DEMAND(child->parent_ == nullptr);
    child->parent_ = parent;
    // This field is only used by the root context so set to an invalid
    // value here.
    child->current_change_event_ = -1;
  }

  /** Derived classes must implement this so that it performs the complete
  deep copy of the context, including all base class members but not fixing
  up base class pointers. To do that, implement a protected copy constructor
  that inherits from the base class copy constructor (which doesn't repair the
  pointers), then implement DoCloneWithoutPointers() as
  `return std::unique_ptr<ContextBase>(new DerivedType(*this));`. */
  virtual std::unique_ptr<ContextBase> DoCloneWithoutPointers() const = 0;

  /** DiagramContext must implement this to invoke BuildTrackerPointerMap() on
  each of its subcontexts. The default implementation does nothing which is
  fine for a LeafContext. */
  virtual void DoPropagateBuildTrackerPointerMap(
      const ContextBase& clone,
      DependencyTracker::PointerMap* tracker_map) const {
    unused(clone, tracker_map);
  }

  /** DiagramContext must implement this to invoke FixContextPointers() on
  each of its subcontexts. The default implementation does nothing which is
  fine for a LeafContext. */
  virtual void DoPropagateFixContextPointers(
      const ContextBase& source,
      const DependencyTracker::PointerMap& tracker_map) {
    unused(source, tracker_map);
  }

  /** DiagramContext must implement this to invoke a caching behavior change on
  each of its subcontexts. The default implementation does nothing which is
  fine for a LeafContext. */
  virtual void DoPropagateCachingChange(void (Cache::*caching_change)()) const {
    unused(caching_change);
  }

  /** DiagramContext must implement this to invoke PropagateBulkChange()
  on its subcontexts, passing along the indicated method that specifies the
  particular bulk change (e.g. whole state, all parameters, all discrete state
  variables, etc.). The default implementation does nothing which is fine for
  a LeafContext. */
  virtual void DoPropagateBulkChange(
      int64_t change_event,
      void (ContextBase::*note_bulk_change)(int64_t change_event)) {
    unused(change_event, note_bulk_change);
  }

 private:
  friend class internal::SystemBaseContextBaseAttorney;

  // Returns the parent Context or `nullptr` if this is the root Context.
  const ContextBase* get_parent_base() const { return parent_; }

  // Records the name of the system whose context this is.
  void set_system_name(const std::string& name) { system_name_ = name; }

  // Records the id of the subsystem that created this context.
  void set_system_id(internal::SystemId id) { system_id_ = id; }

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
  // Each non-null FixedInputPortValue has a ticket and associated
  // tracker.
  // Index with InputPortIndex.
  std::vector<copyable_unique_ptr<FixedInputPortValue>>
      input_port_values_;

  // For each input port, the type checker function will be used for validation
  // when setting a fixed input.
  std::vector<std::function<void(const AbstractValue&)>>
      input_port_type_checkers_;

  // The cache of pre-computed values owned by this subcontext.
  mutable Cache cache_;

  // This is the dependency graph for values within this subcontext.
  DependencyGraph graph_;

  // This is used only when this subcontext is serving as the root of a context
  // tree, in which case it will be initialized to zero as shown. In any
  // non-root context, it will be reset to -1 when the parent pointer is
  // assigned and must never change from that value.
  // Note that it does *not* get reset when copied.
  int64_t current_change_event_{0};

  // The Context of the enclosing Diagram. Null/invalid when this is the root
  // context.
  reset_on_copy<ContextBase*> parent_;

  // Name of the subsystem whose subcontext this is.
  std::string system_name_;

  // Unique id of the subsystem whose subcontext this is.
  internal::SystemId system_id_;

  // Used to validate that System-derived classes didn't forget to invoke the
  // SystemBase method that properly sets up the ContextBase.
  bool is_context_base_initialized_{false};
};

#ifndef DRAKE_DOXYGEN_CXX
class DiagramContextTest;
class LeafContextTest;
class SystemBase;
namespace internal {

// This is an attorney-client pattern class providing SystemBase with access to
// certain specific ContextBase private methods, and nothing else.
class SystemBaseContextBaseAttorney {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SystemBaseContextBaseAttorney)
  SystemBaseContextBaseAttorney() = delete;

 private:
  friend class drake::systems::DiagramContextTest;
  friend class drake::systems::LeafContextTest;
  friend class drake::systems::SystemBase;

  static void set_system_name(ContextBase* context, const std::string& name) {
    DRAKE_DEMAND(context != nullptr);
    context->set_system_name(name);
  }
  static void set_system_id(ContextBase* context, internal::SystemId id) {
    DRAKE_DEMAND(context != nullptr);
    context->set_system_id(id);
  }
  static const ContextBase* get_parent_base(const ContextBase& context) {
    return context.get_parent_base();
  }

  static void AddInputPort(
      ContextBase* context, InputPortIndex expected_index,
      DependencyTicket ticket,
      std::function<void(const AbstractValue&)> type_checker) {
    DRAKE_DEMAND(context != nullptr);
    context->AddInputPort(expected_index, ticket, std::move(type_checker));
  }

  static void AddOutputPort(
      ContextBase* context, OutputPortIndex expected_index,
      DependencyTicket ticket,
      const internal::OutputPortPrerequisite& prerequisite) {
    DRAKE_DEMAND(context != nullptr);
    context->AddOutputPort(expected_index, ticket, prerequisite);
  }

  // Provide SystemBase mutable access to the ticket lists.
  static void AddDiscreteStateTicket(ContextBase* context,
                                     DependencyTicket ticket) {
    DRAKE_DEMAND(context != nullptr);
    context->AddDiscreteStateTicket(ticket);
  }

  static void AddAbstractStateTicket(ContextBase* context,
                                     DependencyTicket ticket) {
    DRAKE_DEMAND(context != nullptr);
    context->AddAbstractStateTicket(ticket);
  }

  static void AddNumericParameterTicket(ContextBase* context,
                                        DependencyTicket ticket) {
    DRAKE_DEMAND(context != nullptr);
    context->AddNumericParameterTicket(ticket);
  }

  static void AddAbstractParameterTicket(ContextBase* context,
                                         DependencyTicket ticket) {
    DRAKE_DEMAND(context != nullptr);
    context->AddAbstractParameterTicket(ticket);
  }

  static bool is_context_base_initialized(const ContextBase& context) {
    return context.is_context_base_initialized_;
  }

  // SystemBase should invoke this when ContextBase has been successfully
  // initialized.
  static void mark_context_base_initialized(ContextBase* context) {
    DRAKE_DEMAND(context);
    DRAKE_DEMAND(!context->is_context_base_initialized_);
    context->is_context_base_initialized_ = true;
  }
};

}  // namespace internal
#endif

}  // namespace systems
}  // namespace drake
