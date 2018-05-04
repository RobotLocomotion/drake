#pragma once

#include <memory>
#include <utility>

#include "drake/common/drake_optional.h"
#include "drake/common/drake_throw.h"
#include "drake/systems/framework/context_base.h"
#include "drake/systems/framework/input_port_value.h"
#include "drake/systems/framework/parameters.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

#ifndef DRAKE_DOXYGEN_CXX
namespace detail {
// This provides System<T> access to a Context's parent.
template<typename T>
class SystemContextAttorney;
}  // namespace detail
#endif

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
    std::unique_ptr<ContextBase> clone_base(ContextBase::Clone());
    DRAKE_DEMAND(dynamic_cast<Context<T>*>(clone_base.get()) != nullptr);
    return std::unique_ptr<Context<T>>(
        static_cast<Context<T>*>(clone_base.release()));
  }

  ~Context() override = default;

  // =========================================================================
  // Accessors and Mutators for Time.

  /// Returns the current time in seconds.
  const T& get_time() const { return get_step_info().time_sec; }

  /// Set the current time in seconds.
  virtual void set_time(const T& time_sec) {
    get_mutable_step_info()->time_sec = time_sec;
  }

  // =========================================================================
  // Accessors and Mutators for State.

  virtual const State<T>& get_state() const = 0;
  virtual State<T>& get_mutable_state() = 0;

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

  /// Sets the continuous state to @p xc, deleting whatever was there before.
  void set_continuous_state(std::unique_ptr<ContinuousState<T>> xc) {
    get_mutable_state().set_continuous_state(std::move(xc));
  }

  /// Returns a mutable reference to the continuous component of the state,
  /// which may be of size zero.
  ContinuousState<T>& get_mutable_continuous_state() {
    return get_mutable_state().get_mutable_continuous_state();
  }

  /// Returns a mutable reference to the continuous state vector, devoid
  /// of second-order structure. The vector may be of size zero.
  VectorBase<T>& get_mutable_continuous_state_vector() {
    return get_mutable_continuous_state().get_mutable_vector();
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
  /// which may be of size zero.
  DiscreteValues<T>& get_mutable_discrete_state() {
    return get_mutable_state().get_mutable_discrete_state();
  }

  /// Returns a mutable reference to the _only_ discrete state vector.
  /// @sa get_discrete_state_vector().
  /// @pre There is only one discrete state group.
  BasicVector<T>& get_mutable_discrete_state_vector() {
    return get_mutable_discrete_state().get_mutable_vector();
  }

  /// Returns a mutable reference to group (vector) @p index of the discrete
  /// state.
  /// @pre @p index must identify an existing group.
  BasicVector<T>& get_mutable_discrete_state(int index) {
    DiscreteValues<T>& xd = get_mutable_discrete_state();
    return xd.get_mutable_vector(index);
  }

  /// Sets the discrete state to @p xd, deleting whatever was there before.
  void set_discrete_state(std::unique_ptr<DiscreteValues<T>> xd) {
    get_mutable_state().set_discrete_state(std::move(xd));
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
  /// which may be of size zero.
  AbstractValues& get_mutable_abstract_state() {
    return get_mutable_state().get_mutable_abstract_state();
  }

  /// Returns a mutable reference to element @p index of the abstract state.
  /// @pre @p index must identify an existing element.
  template <typename U>
  U& get_mutable_abstract_state(int index) {
    AbstractValues& xa = get_mutable_abstract_state();
    return xa.get_mutable_value(index).GetMutableValue<U>();
  }

  /// Sets the abstract state to @p xa, deleting whatever was there before.
  void set_abstract_state(std::unique_ptr<AbstractValues> xa) {
    get_mutable_state().set_abstract_state(std::move(xa));
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

  /// Returns the number of input ports.
  virtual int get_num_input_ports() const = 0;

  /// Connects the input port at @p index to a FreestandingInputPortValue with
  /// the given abstract @p value. Aborts if @p index is out of range.
  /// Returns a reference to the allocated FreestandingInputPortValue. The
  /// reference will remain valid until this input port's value source is
  /// replaced or the %Context is destroyed. You may use that reference to
  /// modify the input port's value using the appropriate
  /// FreestandingInputPortValue method, which will ensure that invalidation
  /// notifications are delivered.
  FreestandingInputPortValue& FixInputPort(
      int index, std::unique_ptr<AbstractValue> value) {
    auto free_value_ptr =
        std::make_unique<FreestandingInputPortValue>(std::move(value));
    FreestandingInputPortValue& free_value = *free_value_ptr;
    SetInputPortValue(index, std::move(free_value_ptr));
    return free_value;
  }

  /// Connects the input port at @p index to a FreestandingInputPortValue with
  /// the given vector @p vec. Otherwise same as above method.
  FreestandingInputPortValue& FixInputPort(
      int index, std::unique_ptr<BasicVector<T>> vec) {
    return FixInputPort(
        index, std::make_unique<Value<BasicVector<T>>>(std::move(vec)));
  }

  /// Same as above method but starts with an Eigen vector whose contents are
  /// used to initialize a BasicVector in the FreestandingInputPortValue.
  FreestandingInputPortValue& FixInputPort(
      int index, const Eigen::Ref<const VectorX<T>>& data) {
    auto vec = std::make_unique<BasicVector<T>>(data);
    return FixInputPort(index, std::move(vec));
  }

  /// Returns the FreestandingInputPortValue for the given input port if it
  /// is a fixed (freestanding) input port, otherwise nullptr.
  // TODO(sherm1) Move this T-independent method to ContextBase.
  const FreestandingInputPortValue* MaybeGetFixedInputPortValue(
      int index) const {
    // TODO(sherm1) Get rid of this trickery and simply return the freestanding
    // value or null. See caching branch near context_base.h:287.
    const InputPortValue* port_value = GetInputPortValue(index);
    if (port_value == nullptr) return nullptr;  // Unconnected port.

    // TODO(sherm1) This inappropriate use of dynamic_cast as a runtime
    // type-test violates the GSG. Eliminate with above fix.
    auto const free_port_value =
        dynamic_cast<const FreestandingInputPortValue*>(port_value);

    return free_port_value;  // The value, or nullptr if not freestanding.
  }

  // =========================================================================
  // Accessors and Mutators for Parameters.

  virtual const Parameters<T>& get_parameters() const = 0;
  virtual Parameters<T>& get_mutable_parameters() = 0;

  /// Returns the number of vector-valued parameters.
  int num_numeric_parameters() const {
    return get_parameters().num_numeric_parameters();
  }

  /// Returns a const reference to the vector-valued parameter at @p index.
  /// Asserts if @p index doesn't exist.
  const BasicVector<T>& get_numeric_parameter(int index) const {
    return get_parameters().get_numeric_parameter(index);
  }

  /// Returns a mutable reference to element @p index of the vector-valued
  /// parameters. Asserts if @p index doesn't exist.
  BasicVector<T>& get_mutable_numeric_parameter(int index) {
    return get_mutable_parameters().get_mutable_numeric_parameter(index);
  }

  /// Returns the number of abstract-valued parameters.
  int num_abstract_parameters() const {
    return get_parameters().num_abstract_parameters();
  }

  /// Returns a const reference to the abstract-valued parameter at @p index.
  /// Asserts if @p index doesn't exist.
  const AbstractValue& get_abstract_parameter(int index) const {
    return get_parameters().get_abstract_parameter(index);
  }

  /// Returns a mutable reference to element @p index of the abstract-valued
  /// parameters. Asserts if @p index doesn't exist.
  AbstractValue& get_mutable_abstract_parameter(int index) {
    return get_mutable_parameters().get_mutable_abstract_parameter(index);
  }

  // =========================================================================
  // Accessors and Mutators for Accuracy.

  /// Records the user's requested accuracy. If no accuracy is requested,
  /// computations are free to choose suitable defaults, or to refuse to
  /// proceed without an explicit accuracy setting.
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
  // TODO(edrumwri) Invalidate all cached accuracy-dependent computations, and
  // propagate accuracy to all subcontexts in a diagram context.
  virtual void set_accuracy(const optional<double>& accuracy) {
    accuracy_ = accuracy;
  }

  /// Returns the accuracy setting (if any).
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
  void SetTimeStateAndParametersFrom(const Context<double>& source) {
    set_time(T(source.get_time()));
    set_accuracy(source.get_accuracy());
    get_mutable_state().SetFrom(source.get_state());
    get_mutable_parameters().SetFrom(source.get_parameters());
  }

  /// Throws an exception unless the given port information matches the inputs
  /// actually connected to this context in shape.
  void VerifyInputPort(InputPortIndex i, PortDataType data_type,
                       int size) const {
    const InputPortValue* port_value = GetInputPortValue(i);
    // If the port isn't connected, we don't have anything else to check.
    if (port_value == nullptr)
      return;
    // TODO(david-german-tri, sherm1): Consider checking sampling here.

    // In the vector-valued case, check the size.
    if (data_type == kVectorValued) {
      const BasicVector<T>* input_vector =
          port_value->template get_vector_data<T>();
      DRAKE_THROW_UNLESS(input_vector != nullptr);
      DRAKE_THROW_UNLESS(input_vector->size() == size);
    }
    // In the abstract-valued case, there is nothing else to check.
  }

 protected:
  Context() = default;

  /// Copy constructor takes care of base class and `Context<T>` data members.
  /// Derived classes must implement copy constructors that delegate to this
  /// one for use in their DoCloneWithoutPointers() implementations.
  // Default implementation invokes the base class copy constructor and then
  // the local member copy constructors.
  Context(const Context<T>&) = default;

  /// Clones a context but without any of its internal pointers.
  // Structuring this as a static method permits a DiagramContext to invoke
  // this protected functionality on its children.
  // This is just an intentional shadowing of the base class method to return a
  // more convenient type.
  static std::unique_ptr<Context<T>> CloneWithoutPointers(
      const Context<T>& source) {
    std::unique_ptr<ContextBase> clone_base(
        ContextBase::CloneWithoutPointers(source));
    DRAKE_DEMAND(dynamic_cast<Context<T>*>(clone_base.get()) != nullptr);
    std::unique_ptr<Context<T>> clone(
        static_cast<Context<T>*>(clone_base.release()));
    return clone;
  }

  /// Override to return the appropriate concrete State class to be returned
  /// by CloneState().
  virtual std::unique_ptr<State<T>> DoCloneState() const = 0;

  /// Returns a const reference to current time and step information.
  const StepInfo<T>& get_step_info() const { return step_info_; }

  /// Provides writable access to time and step information, with the side
  /// effect of invaliding any computation that is dependent on them.
  /// TODO(david-german-tri) Invalidate all cached time- and step-dependent
  /// computations.
  StepInfo<T>* get_mutable_step_info() { return &step_info_; }

  /// Returns the InputPortValue at the given @p index, which may be nullptr if
  /// it has never been set with SetInputPortValue().
  /// Asserts if @p index is out of range.
  virtual const InputPortValue* GetInputPortValue(int index) const = 0;

  /// Allows DiagramContext to invoke the protected method on subcontexts.
  static const InputPortValue* GetInputPortValue(const Context<T>& context,
                                                 int index) {
    return context.GetInputPortValue(index);
  }

  /// Connects the input port at @p index to the value source @p port_value.
  /// Disconnects whatever value source was previously there, and de-registers
  /// it from the output port on which it depends.  In some Context
  /// implementations, may require a recursive search through a tree of
  /// subcontexts. Implementations must abort if @p index is out of range.
  virtual void SetInputPortValue(
      int index, std::unique_ptr<InputPortValue> port_value) = 0;

  /// Allows DiagramContext to invoke the protected method on subcontexts.
  static void SetInputPortValue(Context<T>* context, int index,
                                std::unique_ptr<InputPortValue> port_value) {
    context->SetInputPortValue(index, std::move(port_value));
  }

  /// Declares that @p parent is the context of @p child's enclosing Diagram.
  /// The enclosing Diagram context is needed to evaluate inputs recursively.
  /// Aborts if the parent has already been set to something else.
  // Use static method so DiagramContext can invoke this on behalf of a child.
  // Output argument is listed first because it is serving as the 'this'
  // pointer here.
  static void set_parent(Context<T>* child, const Context<T>* parent) {
    DRAKE_DEMAND(child != nullptr);
    child->set_parent(parent);
  }

 private:
  friend class detail::SystemContextAttorney<T>;

  void set_parent(const Context<T>* parent) {
    DRAKE_DEMAND(parent_ == nullptr || parent_ == parent);
    parent_ = parent;
  }

  // Returns the parent Context or `nullptr` if this is the root Context.
  const Context<T>* get_parent() const {
    return parent_;
  }

  // Current time and step information.
  StepInfo<T> step_info_;

  // Accuracy setting.
  optional<double> accuracy_;

  // The context of the enclosing Diagram, used in EvalInputPort.
  // This pointer MUST be treated as a black box. If you call any substantive
  // methods on it, you are probably making a mistake.
  reset_on_copy<const Context<T>*> parent_;
};

#ifndef DRAKE_DOXYGEN_CXX
template <typename T> class System;
namespace detail {

// This is an attorney-client pattern class providing System<T> with access to
// certain specific Context<T> private methods, which are otherwise private,
// and nothing else.
template <typename T>
class SystemContextAttorney {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SystemContextAttorney);
  SystemContextAttorney() = delete;

 private:
  friend class System<T>;
  static const Context<T>* get_parent(const Context<T>& context) {
    return context.get_parent();
  }
};
#endif

}  // namespace detail

}  // namespace systems
}  // namespace drake
