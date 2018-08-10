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

  /// Returns a deep copy of this Context.
  // This is just an intentional shadowing of the base class method to return
  // a more convenient type.
  std::unique_ptr<Context<T>> Clone() const {
    return dynamic_pointer_cast_or_throw<Context<T>>(ContextBase::Clone());
  }

  ~Context() override = default;

  // =========================================================================
  // Accessors and Mutators for Time.

  /// Returns the current time in seconds.
  const T& get_time() const { return get_step_info().time_sec; }

  /// Set the current time in seconds. If this is a time change, invalidates all
  /// time-dependent quantities in this context and its subcontexts.
  void set_time(const T& time_sec) {
    if (time_sec != get_time()) {
      const int64_t change_event = this->start_new_change_event();
      PropagateTimeChange(time_sec, change_event);
    }
  }

  // =========================================================================
  // Accessors and Mutators for State.

  /// Returns a const reference to the whole State.
  const State<T>& get_state() const {
    return do_access_state();
  }

  /// Returns a mutable reference to the whole State, invalidating _all_
  /// state-dependent computations. If you don't mean to change the whole
  /// thing, use more focused methods to get mutable access to only a portion
  /// of the state so that fewer computations are invalidated.
  State<T>& get_mutable_state() {
    const int64_t change_event = this->start_new_change_event();
    PropagateBulkChange(change_event, &Context<T>::NoteAllStateChanged);
    return do_access_mutable_state();
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

  /// Returns a mutable reference to the continuous component of the state,
  /// which may be of size zero. Invalidates all continuous state-dependent
  /// computations in this context and its subcontexts, recursively.
  ContinuousState<T>& get_mutable_continuous_state() {
    const int64_t change_event = this->start_new_change_event();
    PropagateBulkChange(change_event,
                        &Context<T>::NoteAllContinuousStateChanged);
    return do_access_mutable_state().get_mutable_continuous_state();
  }

  /// Returns a mutable reference to the continuous state vector, devoid
  /// of second-order structure. The vector may be of size zero. Invalidates all
  /// continuous state-dependent computations in this context and its
  /// subcontexts, recursively.
  VectorBase<T>& get_mutable_continuous_state_vector() {
    return get_mutable_continuous_state().get_mutable_vector();
  }

  /// Sets the continuous state to @p xc, including q, v, and z partitions.
  /// The supplied vector must be the same size as the existing continuous
  /// state. Invalidates all continuous state-dependent computations in this
  /// context and its subcontexts, recursively.
  void SetContinuousState(const Eigen::Ref<const VectorX<T>>& xc) {
    get_mutable_continuous_state().SetFromVector(xc);
  }

  /// Sets time to @p t_sec and continuous state to @p xc. Performs a single
  /// invalidation pass to avoid duplicate invalidations for computations that
  /// depend on both time and state.
  void SetTimeAndContinuousState(const T& t_sec,
                                 const Eigen::Ref<const VectorX<T>>& xc) {
    const int64_t change_event = this->start_new_change_event();
    if (t_sec != get_time())
      PropagateTimeChange(t_sec, change_event);
    PropagateBulkChange(change_event,
                        &Context<T>::NoteAllContinuousStateChanged);
    do_access_mutable_state().get_mutable_continuous_state().SetFromVector(xc);
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

  /// Returns a mutable reference to the discrete component of the state,
  /// which may be of size zero. Invalidates all discrete state-dependent
  /// computations in this context and its subcontexts, recursively.
  DiscreteValues<T>& get_mutable_discrete_state() {
    const int64_t change_event = this->start_new_change_event();
    PropagateBulkChange(change_event,
                        &Context<T>::NoteAllDiscreteStateChanged);
    return do_access_mutable_state().get_mutable_discrete_state();
  }

  /// Returns a mutable reference to the _only_ discrete state vector.
  /// @sa get_discrete_state_vector().
  /// @pre There is only one discrete state group.
  BasicVector<T>& get_mutable_discrete_state_vector() {
    return get_mutable_discrete_state().get_mutable_vector();
  }

  /// Returns a mutable reference to group (vector) @p index of the discrete
  /// state. Invalidates all computations that depend (directly or indirectly)
  /// on this discrete state group.
  /// @pre @p index must identify an existing group.
  BasicVector<T>& get_mutable_discrete_state(int index) {
    // TODO(sherm1) Invalidate only dependents of this one discrete group.
    DiscreteValues<T>& xd = get_mutable_discrete_state();
    return xd.get_mutable_vector(index);
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

  /// Returns a mutable reference to the abstract component of the state,
  /// which may be of size zero. Invalidates all abstract state-dependent
  /// computations in this context and its subcontexts, recursively.
  AbstractValues& get_mutable_abstract_state() {
    const int64_t change_event = this->start_new_change_event();
    PropagateBulkChange(change_event,
                        &Context<T>::NoteAllAbstractStateChanged);
    return do_access_mutable_state().get_mutable_abstract_state();
  }

  /// Returns a mutable reference to element @p index of the abstract state.
  /// Invalidates all computations that depend (directly or indirectly) on this
  /// abstract state variable.
  /// @pre @p index must identify an existing element.
  template <typename U>
  U& get_mutable_abstract_state(int index) {
    // TODO(sherm1) Invalidate only dependents of this one abstract variable.
    AbstractValues& xa = get_mutable_abstract_state();
    return xa.get_mutable_value(index).GetMutableValue<U>();
  }

  /// Returns a const reference to the abstract component of the
  /// state at @p index.
  /// @pre @p index must identify an existing element.
  template <typename U>
  const U& get_abstract_state(int index) const {
    const AbstractValues& xa = get_state().get_abstract_state();
    return xa.get_value(index).GetValue<U>();
  }

  // =========================================================================
  // Accessors and Mutators for Input.

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

  // =========================================================================
  // Accessors and Mutators for Parameters.

  /// Returns a const reference to this %Context's parameters.
  const Parameters<T>& get_parameters() const { return *parameters_; }

  /// Returns a mutable reference to this %Context's parameters after
  /// invalidating all parameter-dependent computations in this context and
  /// all its subcontexts, recursively. This is likely to be a _lot_ of
  /// invalidation -- if you are really just changing a single parameter use
  /// one of the indexed methods instead.
  Parameters<T>& get_mutable_parameters() {
    const int64_t change_event = this->start_new_change_event();
    PropagateBulkChange(change_event, &Context<T>::NoteAllParametersChanged);
    return *parameters_;
  }

  /// Returns the number of vector-valued parameters.
  int num_numeric_parameters() const {
    return parameters_->num_numeric_parameters();
  }

  /// Returns a const reference to the vector-valued parameter at @p index.
  /// @pre @p index must identify an existing parameter.
  const BasicVector<T>& get_numeric_parameter(int index) const {
    return parameters_->get_numeric_parameter(index);
  }

  /// Returns a mutable reference to element @p index of the vector-valued
  /// parameters. Invalidates all computations dependent on this parameter.
  /// @pre @p index must identify an existing parameter.
  BasicVector<T>& get_mutable_numeric_parameter(int index) {
    // TODO(sherm1) Invalidate only dependents of this one parameter.
    const int64_t change_event = this->start_new_change_event();
    PropagateBulkChange(change_event,
                        &Context<T>::NoteAllNumericParametersChanged);
    return parameters_->get_mutable_numeric_parameter(index);
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

  /// Returns a mutable reference to element @p index of the abstract-valued
  /// parameters. Invalidates all computations dependent on this parameter.
  /// @pre @p index must identify an existing parameter.
  AbstractValue& get_mutable_abstract_parameter(int index) {
    // TODO(sherm1) Invalidate only dependents of this one parameter.
    const int64_t change_event = this->start_new_change_event();
    PropagateBulkChange(change_event,
                        &Context<T>::NoteAllAbstractParametersChanged);
    return get_mutable_parameters().get_mutable_abstract_parameter(index);
  }

  // =========================================================================
  // Accessors and Mutators for Accuracy.

  /// Records the user's requested accuracy. If no accuracy is requested,
  /// computations are free to choose suitable defaults, or to refuse to
  /// proceed without an explicit accuracy setting. If this is a change to
  /// the current accuracy setting, all accuracy-dependent computations in this
  /// Context and its subcontexts are invalidated.
  ///
  /// Requested accuracy is stored in the %Context for two reasons:
  /// - It permits all computations performed over a System to see the _same_
  ///   accuracy request since accuracy is stored in one shared place, and
  /// - it allows us to invalidate accuracy-dependent cached computations when
  ///   the requested accuracy has changed.
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
  void set_accuracy(const optional<double>& accuracy) {
    if (accuracy != get_accuracy()) {
      const int64_t change_event = this->start_new_change_event();
      PropagateAccuracyChange(accuracy, change_event);
    }
  }

  /// Returns the accuracy setting (if any). Note that the return type is
  /// `optional<double>` rather than the double value itself.
  /// @see set_accuracy() for details.
  const optional<double>& get_accuracy() const { return accuracy_; }

  // =========================================================================
  // Miscellaneous Public Methods

  /// Returns a deep copy of this Context's State.
  std::unique_ptr<State<T>> CloneState() const {
    return DoCloneState();
  }

  /// Initializes this context's time, state, and parameters from the real
  /// values in @p source, regardless of this context's scalar type.
  /// Requires a constructor T(double).
  // TODO(sherm1) Should treat fixed input port values same as parameters.
  void SetTimeStateAndParametersFrom(const Context<double>& source) {
    // A single change event for all these changes is much faster than doing
    // each separately.
    const int64_t change_event = this->start_new_change_event();

    // These two both set the value and perform invalidations.
    PropagateTimeChange(T(source.get_time()), change_event);
    PropagateAccuracyChange(source.get_accuracy(), change_event);

    // Invalidation is separate from the actual value change for bulk changes.
    PropagateBulkChange(change_event, &Context<T>::NoteAllStateChanged);
    do_access_mutable_state().SetFrom(source.get_state());

    PropagateBulkChange(change_event, &Context<T>::NoteAllParametersChanged);
    parameters_->SetFrom(source.get_parameters());

    // TODO(sherm1) Fixed input copying goes here.
  }

  /// (Internal use only) Sets a new time and notifies time-dependent
  /// quantities that they are now invalid, as part of a given change event.
  void PropagateTimeChange(const T& time_sec, int64_t change_event) {
    NoteTimeChanged(change_event);
    step_info_.time_sec = time_sec;
    DoPropagateTimeChange(time_sec, change_event);
  }

  /// (Internal use only) Sets a new accuracy and notifies accuracy-dependent
  /// quantities that they are now invalid, as part of a given change event.
  void PropagateAccuracyChange(const optional<double>& accuracy,
                               int64_t change_event) {
    NoteAccuracyChanged(change_event);
    accuracy_ = accuracy;
    DoPropagateAccuracyChange(accuracy, change_event);
  }

 protected:
  Context() = default;

  /// Copy constructor takes care of base class and `Context<T>` data members.
  /// Derived classes must implement copy constructors that delegate to this
  /// one for use in their DoCloneWithoutPointers() implementations.
  // Default implementation invokes the base class copy constructor and then
  // the local member copy constructors.
  Context(const Context<T>&) = default;

  // Structuring these next three methods as statics permits a DiagramContext to
  // invoke the protected functionality on its children.

  /// (Internal use only) Returns a reference to mutable parameters _without_
  /// invalidation notifications. Use get_mutable_state() instead for normal
  /// access.
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

  /// Derived context class should return a const reference to its concrete
  /// State object.
  virtual const State<T>& do_access_state() const = 0;

  /// Derived context class should return a mutable reference to its concrete
  /// State object _without_ any invalidation. We promise not to allow user
  /// access to this object without invalidation.
  virtual State<T>& do_access_mutable_state() = 0;

  /// Override to return the appropriate concrete State class to be returned
  /// by CloneState().
  virtual std::unique_ptr<State<T>> DoCloneState() const = 0;

  /// Diagram contexts should override this to invoke PropagateTimeChange()
  /// on their subcontexts. The default implementation does nothing.
  virtual void DoPropagateTimeChange(const T& time_sec, int64_t change_event) {
    unused(time_sec, change_event);
  }

  /// Diagram contexts should override this to invoke PropagateAccuracyChange()
  /// on their subcontexts. The default implementation does nothing.
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
