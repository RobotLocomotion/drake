#pragma once

#include <memory>
#include <utility>

#include "drake/common/drake_optional.h"
#include "drake/common/drake_throw.h"
#include "drake/common/pointer_cast.h"
#include "drake/systems/framework/context_base.h"
#include "drake/systems/framework/parameters.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

/// Contains information about the independent variable including time and
/// step number.
// TODO(sherm1) Add step information.
template <typename T>
struct StepInfo {
  /// The time, in seconds. For typical T implementations based on
  /// doubles, time resolution will gradually degrade as time increases.
  // TODO(sherm1): Consider whether this is sufficiently robust.
  T time_sec{0.0};
};

/// %Context is an abstract class template that represents all the typed values
/// that are used in a System's computations: time, numeric-valued input ports,
/// numerical state, and numerical parameters. There are also type-erased
/// abstract state variables, abstract-valued input ports, abstract parameters,
/// and a double accuracy setting. The framework provides two concrete
/// subclasses of %Context: LeafContext (for leaf Systems) and DiagramContext
/// (for composite System Diagrams). Users are forbidden to extend
/// DiagramContext and are discouraged from subclassing LeafContext.
///
/// @tparam T The mathematical type of the context, which must be a valid Eigen
///           scalar.
template <typename T>
class Context : public ContextBase {
 public:
  /// @name  Does not allow copy, move, or assignment.
  //@{
  // Copy constructor is protected for use in implementing Clone().
  Context(Context&&) = delete;
  Context& operator=(const Context&) = delete;
  Context& operator=(Context&&) = delete;
  //@}

  /// @name           Accessors for locally-stored values
  /// Methods in this group provide `const` access to values stored locally in
  /// this %Context. The available values are:
  /// - time
  /// - state
  /// - parameters
  /// - accuracy
  ///
  /// Fixed input port values and cached values (including output port values)
  /// are also stored in the %Context but are accessed indirectly via methods
  /// like SystemBase::EvalInputValue(), through CacheEntry and OutputPort
  /// objects, or via the FixedInputPortValue object that was returned when
  /// an input port value was set.
  /// @see FixInputPort()
  //@{

  /// Returns the current time in seconds.
  const T& get_time() const { return get_step_info().time_sec; }

  /// Returns a const reference to the whole State.
  const State<T>& get_state() const {
    return do_access_state();
  }

  /// Returns true if the Context has no state.
  bool is_stateless() const {
    const int nxc = get_continuous_state().size();
    const int nxd = get_num_discrete_state_groups();
    const int nxa = get_num_abstract_states();
    return nxc == 0 && nxd == 0 && nxa == 0;
  }

  /// Returns true if the Context has continuous state, but no discrete or
  /// abstract state.
  bool has_only_continuous_state() const {
    const int nxc = get_continuous_state().size();
    const int nxd = get_num_discrete_state_groups();
    const int nxa = get_num_abstract_states();
    return nxc > 0 && nxd == 0 && nxa == 0;
  }

  /// Returns true if the Context has discrete state, but no continuous or
  /// abstract state.
  bool has_only_discrete_state() const {
    const int nxc = get_continuous_state().size();
    const int nxd = get_num_discrete_state_groups();
    const int nxa = get_num_abstract_states();
    return nxd > 0 && nxc == 0 && nxa == 0;
  }

  /// Returns the total dimension of all of the basic vector states (as if they
  /// were muxed).
  /// @throws std::runtime_error if the system contains any abstract state.
  int get_num_total_states() const {
    DRAKE_THROW_UNLESS(get_num_abstract_states() == 0);
    int count = get_continuous_state().size();
    for (int i = 0; i < get_num_discrete_state_groups(); i++)
      count += get_discrete_state(i).size();
    return count;
  }

  /// Returns a const reference to the continuous component of the state,
  /// which may be of size zero.
  const ContinuousState<T>& get_continuous_state() const {
    return get_state().get_continuous_state();
  }

  /// Returns a reference to the continuous state vector, devoid of second-order
  /// structure. The vector may be of size zero.
  const VectorBase<T>& get_continuous_state_vector() const {
    return get_continuous_state().get_vector();
  }

  /// Returns the number of vectors (groups) in the discrete state.
  int get_num_discrete_state_groups() const {
    return get_state().get_discrete_state().num_groups();
  }

  /// Returns a reference to the entire discrete state, which may consist of
  /// multiple discrete state vectors (groups).
  const DiscreteValues<T>& get_discrete_state() const {
    return get_state().get_discrete_state();
  }

  /// Returns a reference to the _only_ discrete state vector. The vector may be
  /// of size zero.
  /// @pre There is only one discrete state group.
  const BasicVector<T>& get_discrete_state_vector() const {
    return get_discrete_state().get_vector();
  }

  /// Returns a const reference to group (vector) @p index of the discrete
  /// state.
  /// @pre @p index must identify an existing group.
  const BasicVector<T>& get_discrete_state(int index) const {
    const DiscreteValues<T>& xd = get_state().get_discrete_state();
    return xd.get_vector(index);
  }

  /// Returns the number of elements in the abstract state.
  int get_num_abstract_states() const {
    return get_state().get_abstract_state().size();
  }

  /// Returns a const reference to the abstract component of the state, which
  /// may be of size zero.
  const AbstractValues& get_abstract_state() const {
    return get_state().get_abstract_state();
  }

  /// Returns a const reference to the abstract component of the
  /// state at @p index.
  /// @pre @p index must identify an existing element.
  /// @pre the abstract state's type must match the template argument.
  template <typename U>
  const U& get_abstract_state(int index) const {
    const AbstractValues& xa = get_state().get_abstract_state();
    return xa.get_value(index).GetValue<U>();
  }

  /// Returns the accuracy setting (if any). Note that the return type is
  /// `optional<double>` rather than the double value itself.
  /// @see set_accuracy() for details.
  const optional<double>& get_accuracy() const { return accuracy_; }

  /// Returns a const reference to this %Context's parameters.
  const Parameters<T>& get_parameters() const { return *parameters_; }

  /// Returns the number of vector-valued parameters.
  int num_numeric_parameters() const {
    return parameters_->num_numeric_parameters();
  }

  /// Returns a const reference to the vector-valued parameter at @p index.
  /// @pre @p index must identify an existing parameter.
  const BasicVector<T>& get_numeric_parameter(int index) const {
    return parameters_->get_numeric_parameter(index);
  }

  /// Returns the number of abstract-valued parameters.
  int num_abstract_parameters() const {
    return get_parameters().num_abstract_parameters();
  }

  /// Returns a const reference to the abstract-valued parameter at @p index.
  /// @pre @p index must identify an existing parameter.
  const AbstractValue& get_abstract_parameter(int index) const {
    return get_parameters().get_abstract_parameter(index);
  }
  //@}

  /// @name           Methods for changing locally-stored values
  /// Methods in this group allow changes to the values of quantities stored
  /// locally in this %Context. The changeable quantities are:
  /// - time
  /// - state
  /// - parameters
  /// - accuracy
  /// - fixed input port values
  ///
  /// Expensive computations may be performed that depend on the current values
  /// of some or all of the above quantities. For efficiency, we save the
  /// results of such computations in the %Context so that we can reuse
  /// those results without unnecessary recomputation.
  ///
  /// <h3>Terminology</h3>
  /// We call a quantity whose value is needed in order to perform a particular
  /// computation a _prerequisite_ of that computation, and we say that the
  /// computation is a _dependent_ of that prerequisite. If a prerequisite's
  /// value changes, a result computed using an earlier value is _invalid_; we
  /// say that the prerequisite change _invalidates_ that result, and that the
  /// result is _out of date_ with respect to its prerequisites. It is important
  /// to note that the result of one computation can serve as a prerequisite to
  /// another computation; we call the dependent computation a _downstream_
  /// computation and the prerequisite an _upstream_ computation.
  ///
  /// <h3>Caching</h3>
  /// Drake provides a caching system that is responsible for
  /// - storing computed results in the %Context's _cache_, and
  /// - ensuring that any cached result that _may_ be invalid is marked
  ///   "out of date".
  ///
  /// The correctness of results reported by Drake depends critically on _every_
  /// cached result being correctly flagged as up to date or out of date with
  /// respect to its prerequisites. Only when it is known _for certain_ that
  /// a result is valid can it be marked up to date. Access to cached results
  /// is performed through `Eval()` methods that return up to date results
  /// immediately but initiate recomputation first for results marked out of
  /// date. The methods in the group below are responsible for ensuring that
  /// cached results are marked out of date whenever a prerequisite value may
  /// have changed. These methods _do not_ initiate such recomputation
  /// themselves.
  ///
  /// <h3>Invalidation and "out of date" notification</h3>
  /// Each method in this group provides the ability to change a particular
  /// subset of the available quantities listed above. This triggers "out of
  /// date" notifications to the cached results for all computations for which
  /// any element of that subset is a prerequisite. Such notifications
  /// propagate to downstream computations, whose cached results may reside
  /// anywhere in the full Diagram context tree of which this %Context is a
  /// part. That ensures that "out of date" flags are set correctly for the
  /// cached results of all computations that could be made invalid by a value
  /// change to any of the affected subset of quantities. We call this
  /// process a _notification sweep_.
  ///
  /// <h3>Which method to use</h3>
  /// Choose the most-specific method in this group that permits you to perform
  /// the modifications you need. For example, if you need to modify only
  /// continuous state variables, don't use a method that provides mutable
  /// access to the whole state. That provides two performance advantages:
  /// - fewer downstream computations need to be marked out of date, making the
  ///   notification sweep faster, and
  /// - fewer results will need to be recomputed later.
  ///
  /// The methods below may be grouped into "safe" methods that begin with
  /// `set` or `Set` and "dangerous" methods that begin with `get_mutable`
  /// and return a mutable reference. In addition the `FixInputPort` methods
  /// return an object that has `get_mutable` methods with the same dangers
  /// (see FixedInputPortValue). Prefer the safe methods when possible.
  ///
  /// <h4>Safe "set" methods</h4>
  /// The `set` and `Set` methods are safe in the sense that they perform both
  /// the "mark as out of date" notification sweep through dependent cached
  /// results and the update to the local quantity's value. They do not return
  /// a reference to the value object. Using these methods ensures that no value
  /// modification can occur without an appropriate notification sweep. Also,
  /// these methods can be used to perform multiple changes at once (say time
  /// and state), requiring only a single notification sweep, and _may_ perform
  /// optimizations to avoid notifications in case some of the new values are
  /// the same as the old ones.
  ///
  /// <h4>Dangerous "get_mutable" methods</h4>
  /// The `get_mutable` methods return a mutable reference to the local value
  /// object within this %Context. The notification sweep is done prior to
  /// returning that reference. You can then use the reference to make the
  /// desired change. Note that with these methods we do not actually know
  /// whether dependent computations are invalid; that depends what you do with
  /// the reference once you have it. Nevertheless you will pay the cost of
  /// the notifications sweep immediately and the cost of recomputation later
  /// when you ask for the value. So don't call one of these methods unless
  /// you are certain you will be writing through the returned reference.
  ///
  /// You _must not_ hold on to the returned reference expecting
  /// to be able to make subsequent changes, because those changes can't be
  /// seen by the framework and thus will not cause the necessary notification
  /// sweep to occur. Instead, request the mutable reference again when
  /// you need it so that the necessary notification sweep can be performed.
  ///
  /// <h3>Implementation note</h3>
  /// Each method in the group below guarantees to mark as out of date any
  /// dependents of the quantities to which it permits modification, including
  /// all downstream dependents. However, the current implementations may also
  /// perform some unnecessary notifications. If so, that is noted in the method
  /// documentation. You should still use the most-specific available method so
  /// that you will benefit from later improvements that result in fewer
  /// notifications.
  //@{

  // TODO(sherm1) All these methods perform invalidation sweeps so aren't
  // entitled to lower_case_names. Deprecate and replace (see #9205).

  /// Sets the current time in seconds. Sends out of date notifications for all
  /// time-dependent computations (at least if the time has actually changed).
  /// Time must have the same value in every subcontext within the same Diagram
  /// context tree so may only be modified at the root context of the tree.
  /// @throws std::logic_error if this is not the root context.
  // TODO(sherm1) Consider whether this should avoid the notification sweep
  // if the new time is the same as the old time.
  void set_time(const T& time_sec) {
    ThrowIfNotRootContext(__func__, "Time");
    const int64_t change_event = this->start_new_change_event();
    PropagateTimeChange(this, time_sec, change_event);
  }

  /// Returns a mutable reference to the whole State, potentially invalidating
  /// _all_ state-dependent computations so requiring out of date notifications
  /// to be made for all such computations. If you don't mean to change the
  /// whole state, use more focused methods to modify only a portion of the
  /// state. See class documentation for more information.
  State<T>& get_mutable_state() {
    const int64_t change_event = this->start_new_change_event();
    PropagateBulkChange(change_event, &Context<T>::NoteAllStateChanged);
    return do_access_mutable_state();
  }

  /// Returns a mutable reference to the continuous component of the state,
  /// which may be of size zero. Sends out of date notifications for all
  /// continuous-state-dependent computations.
  ContinuousState<T>& get_mutable_continuous_state() {
    const int64_t change_event = this->start_new_change_event();
    PropagateBulkChange(change_event,
                        &Context<T>::NoteAllContinuousStateChanged);
    return do_access_mutable_state().get_mutable_continuous_state();
  }

  /// Returns a mutable reference to the continuous state vector, devoid
  /// of second-order structure. The vector may be of size zero. Sends out of
  /// date notifications for all continuous-state-dependent computations.
  VectorBase<T>& get_mutable_continuous_state_vector() {
    return get_mutable_continuous_state().get_mutable_vector();
  }

  // TODO(sherm1) Add more-specific state "set" methods for smaller
  // state groupings (issue #9205).

  /// Sets the continuous state to @p xc, including q, v, and z partitions.
  /// The supplied vector must be the same size as the existing continuous
  /// state. Sends out of date notifications for all continuous-state-dependent
  /// computations.
  void SetContinuousState(const Eigen::Ref<const VectorX<T>>& xc) {
    get_mutable_continuous_state().SetFromVector(xc);
  }

  /// Sets time to @p time_sec and continuous state to @p xc. Performs a single
  /// notification sweep to avoid duplicate notifications for computations that
  /// depend on both time and state.
  /// @throws std::logic_error if this is not the root context.
  // TODO(sherm1) Consider whether this should avoid invalidation of
  // time-dependent quantities if the new time is the same as the old time.
  void SetTimeAndContinuousState(const T& time_sec,
                                 const Eigen::Ref<const VectorX<T>>& xc) {
    ThrowIfNotRootContext(__func__, "Time");
    const int64_t change_event = this->start_new_change_event();
    PropagateTimeChange(this, time_sec, change_event);
    PropagateBulkChange(change_event,
                        &Context<T>::NoteAllContinuousStateChanged);
    do_access_mutable_state().get_mutable_continuous_state().SetFromVector(xc);
  }

  /// Returns a mutable reference to the discrete component of the state,
  /// which may be of size zero. Sends out of date notifications for all
  /// discrete-state-dependent computations.
  DiscreteValues<T>& get_mutable_discrete_state() {
    const int64_t change_event = this->start_new_change_event();
    PropagateBulkChange(change_event,
                        &Context<T>::NoteAllDiscreteStateChanged);
    return do_access_mutable_state().get_mutable_discrete_state();
  }

  /// Returns a mutable reference to the _only_ discrete state vector.
  /// Sends out of date notifications for all discrete-state-dependent
  /// computations.
  /// @sa get_discrete_state_vector().
  /// @pre There is only one discrete state group.
  BasicVector<T>& get_mutable_discrete_state_vector() {
    return get_mutable_discrete_state().get_mutable_vector();
  }

  /// Returns a mutable reference to group (vector) @p index of the discrete
  /// state. Sends out of date notifications for all computations that depend
  /// on this discrete state group.
  /// @pre @p index must identify an existing group.
  /// @bug Currently notifies dependents of _all_ groups.
  // TODO(sherm1) Invalidate only dependents of this one discrete group.
  BasicVector<T>& get_mutable_discrete_state(int index) {
    DiscreteValues<T>& xd = get_mutable_discrete_state();
    return xd.get_mutable_vector(index);
  }

  /// Returns a mutable reference to the abstract component of the state,
  /// which may be of size zero. Sends out of date notifications for all
  /// abstract-state-dependent computations.
  AbstractValues& get_mutable_abstract_state() {
    const int64_t change_event = this->start_new_change_event();
    PropagateBulkChange(change_event,
                        &Context<T>::NoteAllAbstractStateChanged);
    return do_access_mutable_state().get_mutable_abstract_state();
  }

  /// Returns a mutable reference to element @p index of the abstract state.
  /// Sends out of date notifications for all computations that depend on this
  /// abstract state variable.
  /// @pre @p index must identify an existing element.
  /// @pre the abstract state's type must match the template argument.
  /// @bug Currently notifies dependents of _any_ abstract state variable.
  // TODO(sherm1) Invalidate only dependents of this one abstract variable.
  template <typename U>
  U& get_mutable_abstract_state(int index) {
    AbstractValues& xa = get_mutable_abstract_state();
    return xa.get_mutable_value(index).GetMutableValue<U>();
  }

  /// Returns a mutable reference to this %Context's parameters. Sends out of
  /// date notifications for all parameter-dependent computations. If you don't
  /// mean to change all the parameters, use the indexed methods to modify only
  /// some of the parameters so that fewer computations are invalidated and
  /// fewer notifications need be sent.
  Parameters<T>& get_mutable_parameters() {
    const int64_t change_event = this->start_new_change_event();
    PropagateBulkChange(change_event, &Context<T>::NoteAllParametersChanged);
    return *parameters_;
  }

  /// Returns a mutable reference to element @p index of the vector-valued
  /// (numeric) parameters. Sends out of date notifications for all computations
  /// dependent on this parameter.
  /// @pre @p index must identify an existing numeric parameter.
  /// @bug Currently notifies dependents of _all_ numeric parameters.
  // TODO(sherm1) Invalidate only dependents of this one parameter.
  BasicVector<T>& get_mutable_numeric_parameter(int index) {
    const int64_t change_event = this->start_new_change_event();
    PropagateBulkChange(change_event,
                        &Context<T>::NoteAllNumericParametersChanged);
    return parameters_->get_mutable_numeric_parameter(index);
  }

  /// Returns a mutable reference to element @p index of the abstract-valued
  /// parameters. Sends out of date notifications for all computations dependent
  /// on this parameter.
  /// @pre @p index must identify an existing abstract parameter.
  /// @bug Currently notifies dependents of _all_ abstract parameters.
  // TODO(sherm1) Invalidate only dependents of this one parameter.
  AbstractValue& get_mutable_abstract_parameter(int index) {
    const int64_t change_event = this->start_new_change_event();
    PropagateBulkChange(change_event,
                        &Context<T>::NoteAllAbstractParametersChanged);
    return parameters_->get_mutable_abstract_parameter(index);
  }

  /// Sets this context's time, accuracy, state, and parameters from the
  /// `double` values in @p source, regardless of this context's scalar type.
  /// Sends out of date notifications for all dependent computations in this
  /// context.
  /// @throws std::logic_error if this is not the root context.
  /// @bug Currently does not copy fixed input port values from `source`.
  /// See System::FixInputPortsFrom() if you want to copy those.
  // TODO(sherm1) Should treat fixed input port values same as parameters.
  // TODO(sherm1) Change the name of this method to be more inclusive since it
  //              also copies accuracy (now) and fixed input port values
  //              (pending above TODO).
  void SetTimeStateAndParametersFrom(const Context<double>& source) {
    ThrowIfNotRootContext(__func__, "Time");
    // A single change event for all these changes is faster than doing
    // each separately.
    const int64_t change_event = this->start_new_change_event();

    // These two both set the value and perform notifications.
    PropagateTimeChange(this, T(source.get_time()), change_event);
    PropagateAccuracyChange(this, source.get_accuracy(), change_event);

    // Notification is separate from the actual value change for bulk changes.
    PropagateBulkChange(change_event, &Context<T>::NoteAllStateChanged);
    do_access_mutable_state().SetFrom(source.get_state());

    PropagateBulkChange(change_event, &Context<T>::NoteAllParametersChanged);
    parameters_->SetFrom(source.get_parameters());

    // TODO(sherm1) Fixed input copying goes here.
  }

  // Allow access to the base class method (takes an AbstractValue).
  using ContextBase::FixInputPort;

  /// Connects the input port at @p index to a FixedInputPortValue with
  /// the given vector @p vec. Aborts if @p index is out of range.
  /// Returns a reference to the allocated FixedInputPortValue. The
  /// reference will remain valid until this input port's value source is
  /// replaced or the %Context is destroyed. You may use that reference to
  /// modify the input port's value using the appropriate
  /// FixedInputPortValue method, which will ensure that invalidation
  /// notifications are delivered.
  FixedInputPortValue& FixInputPort(int index, const BasicVector<T>& vec) {
    return ContextBase::FixInputPort(
        index, std::make_unique<Value<BasicVector<T>>>(vec.Clone()));
  }

  /// Same as above method but starts with an Eigen vector whose contents are
  /// used to initialize a BasicVector in the FixedInputPortValue.
  FixedInputPortValue& FixInputPort(
      int index, const Eigen::Ref<const VectorX<T>>& data) {
    return FixInputPort(index, BasicVector<T>(data));
  }

  /// Same as the above method that takes a `const BasicVector<T>&`, but here
  /// the vector is passed by unique_ptr instead of by const reference.  The
  /// caller must not retain any aliases to `vec`; within this method, `vec`
  /// is cloned and then deleted.
  /// @note This overload will become deprecated in the future, because it can
  /// mislead users to believe that they can retain an alias of `vec` to mutate
  /// the fixed value during a simulation.  Callers should prefer to use one of
  /// the other overloads instead.
  FixedInputPortValue& FixInputPort(
      int index, std::unique_ptr<BasicVector<T>> vec) {
    DRAKE_THROW_UNLESS(vec.get() != nullptr);
    return FixInputPort(index, *vec);
  }

  /// Records the user's requested accuracy. If no accuracy is requested,
  /// computations are free to choose suitable defaults, or to refuse to
  /// proceed without an explicit accuracy setting. Any accuracy-dependent
  /// computation in this Context and its subcontexts may be invalidated
  /// by a change to the accuracy setting, so out of date notifications are
  /// sent to all such computations (at least if the accuracy setting has
  /// actually changed). Accuracy must have the same value in every subcontext
  /// within the same context tree so may only be modified at the root context
  /// of a tree.
  ///
  /// @throws std::logic_error if this is not the root context.
  ///
  /// Requested accuracy is stored in the %Context for two reasons:
  /// - It permits all computations performed over a System to see the _same_
  ///   accuracy request since accuracy is stored in one shared place, and
  /// - it allows us to notify accuracy-dependent cached results that they are
  ///   out of date when the accuracy setting changes.
  ///
  /// The accuracy of a complete simulation or other numerical study depends on
  /// the accuracy of _all_ contributing computations, so it is important that
  /// each computation is done in accordance with the overall requested
  /// accuracy. Some examples of where this is needed:
  /// - Error-controlled numerical integrators use the accuracy setting to
  ///   decide what step sizes to take.
  /// - The Simulator employs a numerical integrator, but also uses accuracy to
  ///   decide how precisely to isolate witness function zero crossings.
  /// - Iterative calculations reported as results or cached internally depend
  ///   on accuracy to decide how strictly to converge the results. Examples of
  ///   these are: constraint projection, calculation of distances between
  ///   smooth shapes, and deformation calculations for soft contact.
  ///
  /// The common thread among these examples is that they all share the
  /// same %Context, so by keeping accuracy here it can be used effectively to
  /// control all accuracy-dependent computations.
  // TODO(sherm1) Consider whether to avoid invalidation if the new value is
  // the same as the old one.
  void set_accuracy(const optional<double>& accuracy) {
    ThrowIfNotRootContext(__func__, "Accuracy");
    const int64_t change_event = this->start_new_change_event();
    PropagateAccuracyChange(this, accuracy, change_event);
  }
  //@}

  /// @name             Miscellaneous public methods
  //@{

  /// Returns a deep copy of this Context.
  // This is just an intentional shadowing of the base class method to return
  // a more convenient type.
  std::unique_ptr<Context<T>> Clone() const {
    return dynamic_pointer_cast_or_throw<Context<T>>(ContextBase::Clone());
  }

  /// Returns a deep copy of this Context's State.
  std::unique_ptr<State<T>> CloneState() const {
    return DoCloneState();
  }
  //@}

 protected:
  Context() = default;

  /// Copy constructor takes care of base class and `Context<T>` data members.
  /// Derived classes must implement copy constructors that delegate to this
  /// one for use in their DoCloneWithoutPointers() implementations.
  // Default implementation invokes the base class copy constructor and then
  // the local member copy constructors.
  Context(const Context<T>&) = default;

  // Structuring these methods as statics permits a DiagramContext to invoke
  // the protected functionality on its children.

  /// (Internal use only) Sets a new time and notifies time-dependent
  /// quantities that they are now invalid, as part of a given change event.
  static void PropagateTimeChange(Context<T>* context, const T& time_sec,
                                  int64_t change_event) {
    DRAKE_ASSERT(context != nullptr);
    context->NoteTimeChanged(change_event);
    context->step_info_.time_sec = time_sec;
    context->DoPropagateTimeChange(time_sec, change_event);
  }

  /// (Internal use only) Sets a new accuracy and notifies accuracy-dependent
  /// quantities that they are now invalid, as part of a given change event.
  static void PropagateAccuracyChange(Context<T>* context,
                                      const optional<double>& accuracy,
                                      int64_t change_event) {
    DRAKE_ASSERT(context != nullptr);
    context->NoteAccuracyChanged(change_event);
    context->accuracy_ = accuracy;
    context->DoPropagateAccuracyChange(accuracy, change_event);
  }

  /// (Internal use only) Returns a reference to mutable parameters _without_
  /// invalidation notifications. Use get_mutable_parameters() instead for
  /// normal access.
  static Parameters<T>& access_mutable_parameters(Context<T>* context) {
    DRAKE_ASSERT(context);
    return *context->parameters_;
  }

  /// (Internal use only) Returns a reference to a mutable state _without_
  /// invalidation notifications. Use get_mutable_state() instead for normal
  /// access.
  static State<T>& access_mutable_state(Context<T>* context) {
    DRAKE_ASSERT(context);
    return context->do_access_mutable_state();
  }

  /// (Internal use only) Clones a context but without any of its internal
  /// pointers.
  // This is just an intentional shadowing of the base class method to return a
  // more convenient type.
  static std::unique_ptr<Context<T>> CloneWithoutPointers(
      const Context<T>& source) {
    return dynamic_pointer_cast_or_throw<Context<T>>(
        ContextBase::CloneWithoutPointers(source));
  }

  /// Returns a const reference to its concrete State object.
  virtual const State<T>& do_access_state() const = 0;

  /// Returns a mutable reference to its concrete State object _without_ any
  /// invalidation. We promise not to allow user access to this object without
  /// invalidation.
  virtual State<T>& do_access_mutable_state() = 0;

  /// Returns the appropriate concrete State object to be returned by
  /// CloneState().
  virtual std::unique_ptr<State<T>> DoCloneState() const = 0;

  /// Invokes PropagateTimeChange() on all subcontexts of this Context. The
  /// default implementation does nothing, which is suitable for leaf contexts.
  /// Diagram contexts must override.
  virtual void DoPropagateTimeChange(const T& time_sec, int64_t change_event) {
    unused(time_sec, change_event);
  }

  /// Invokes PropagateAccuracyChange() on all subcontexts of this Context. The
  /// default implementation does nothing, which is suitable for leaf contexts.
  /// Diagram contexts must override.
  virtual void DoPropagateAccuracyChange(const optional<double>& accuracy,
                                         int64_t change_event) {
    unused(accuracy, change_event);
  }

  /// Returns a const reference to current time and step information.
  const StepInfo<T>& get_step_info() const { return step_info_; }

  /// (Internal use only) Sets the continuous state to @p xc, deleting whatever
  /// was there before.
  /// @warning Does _not_ invalidate state-dependent computations.
  void init_continuous_state(std::unique_ptr<ContinuousState<T>> xc) {
    do_access_mutable_state().set_continuous_state(std::move(xc));
  }

  /// (Internal use only) Sets the discrete state to @p xd, deleting whatever
  /// was there before.
  /// @warning Does _not_ invalidate state-dependent computations.
  void init_discrete_state(std::unique_ptr<DiscreteValues<T>> xd) {
    do_access_mutable_state().set_discrete_state(std::move(xd));
  }

  /// (Internal use only) Sets the abstract state to @p xa, deleting whatever
  /// was there before.
  /// @warning Does _not_ invalidate state-dependent computations.
  void init_abstract_state(std::unique_ptr<AbstractValues> xa) {
    do_access_mutable_state().set_abstract_state(std::move(xa));
  }

  /// (Internal use only) Sets the parameters to @p params, deleting whatever
  /// was there before. You must supply a Parameters object; null is not
  /// acceptable.
  /// @warning Does _not_ invalidate parameter-dependent computations.
  void init_parameters(std::unique_ptr<Parameters<T>> params) {
    DRAKE_DEMAND(params != nullptr);
    parameters_ = std::move(params);
  }

 private:
  // Call with arguments like (__func__, "Time"), capitalized as shown.
  void ThrowIfNotRootContext(const char* func_name,
                             const char* quantity) const {
    if (!is_root_context()) {
      throw std::logic_error(
          fmt::format("{}(): {} change allowed only in the root Context.",
                      func_name, quantity));
    }
  }

  // Current time and step information.
  StepInfo<T> step_info_;

  // Accuracy setting.
  optional<double> accuracy_;

  // The parameter values (p) for this Context; this is never null.
  copyable_unique_ptr<Parameters<T>> parameters_{
      std::make_unique<Parameters<T>>()};
};

}  // namespace systems
}  // namespace drake
