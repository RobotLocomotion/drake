#pragma once

#include <memory>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/common/drake_throw.h"
#include "drake/systems/framework/input_port_evaluator_interface.h"
#include "drake/systems/framework/input_port_value.h"
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

/// Context is an abstract base class template that represents all
/// the inputs to a System: time, state, and input vectors. The framework
/// provides two concrete subclasses of Context: LeafContext (for
/// leaf Systems) and DiagramContext (for composite Systems). Users are
/// discouraged from creating additional subclasses.
///
/// @tparam T The mathematical type of the context, which must be a valid Eigen
///           scalar.
template <typename T>
class Context {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Context)

  Context() = default;
  virtual ~Context() = default;

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
    const int nxa = get_num_abstract_state_groups();
    return nxc == 0 && nxd == 0 && nxa == 0;
  }

  /// Returns true if the Context has continuous state, but no discrete or
  /// abstract state.
  bool has_only_continuous_state() const {
    const int nxc = get_continuous_state().size();
    const int nxd = get_num_discrete_state_groups();
    const int nxa = get_num_abstract_state_groups();
    return nxc > 0 && nxd == 0 && nxa == 0;
  }

  /// Returns true if the Context has discrete state, but no continuous or
  /// abstract state.
  bool has_only_discrete_state() const {
    const int nxc = get_continuous_state().size();
    const int nxd = get_num_discrete_state_groups();
    const int nxa = get_num_abstract_state_groups();
    return nxd > 0 && nxc == 0 && nxa == 0;
  }

  /// Returns the total dimension of all of the basic vector states (as if they
  /// were muxed).
  /// @throws std::runtime_error if the system contains any abstract state.
  int get_num_total_states() const {
    DRAKE_THROW_UNLESS(get_num_abstract_state_groups() == 0);
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

  /// Returns the number of elements in the discrete state.
  int get_num_discrete_state_groups() const {
    return get_state().get_discrete_state().num_groups();
  }

  const DiscreteValues<T>& get_discrete_state() const {
    return get_state().get_discrete_state();
  }

  /// Returns a reference to the discrete state vector. The vector may be of
  /// size zero.
  const BasicVector<T>& get_discrete_state_vector() const {
    return get_discrete_state().get_vector();
  }

  /// Returns a mutable pointer to the discrete component of the state,
  /// which may be of size zero.
  DiscreteValues<T>& get_mutable_discrete_state() {
    return get_mutable_state().get_mutable_discrete_state();
  }

  /// Returns a mutable pointer to element @p index of the discrete state.
  /// Asserts if @p index doesn't exist.
  BasicVector<T>& get_mutable_discrete_state(int index) {
    DiscreteValues<T>& xd = get_mutable_discrete_state();
    return xd.get_mutable_vector(index);
  }

  /// Sets the discrete state to @p xd, deleting whatever was there before.
  void set_discrete_state(std::unique_ptr<DiscreteValues<T>> xd) {
    get_mutable_state().set_discrete_state(std::move(xd));
  }

  /// Returns a const pointer to the discrete component of the
  /// state at @p index.  Asserts if @p index doesn't exist.
  const BasicVector<T>& get_discrete_state(int index) const {
    const DiscreteValues<T>& xd = get_state().get_discrete_state();
    return xd.get_vector(index);
  }

  /// Returns the number of elements in the abstract state.
  int get_num_abstract_state_groups() const {
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

  /// Returns a mutable pointer to element @p index of the abstract state.
  /// Asserts if @p index doesn't exist.
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
  /// state at @p index.  Asserts if @p index doesn't exist.
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

  /// Evaluates and returns the value of the input port identified by
  /// @p descriptor, using the given @p evaluator, which should be the Diagram
  /// containing the System that allocated this Context. The evaluation will be
  /// performed in this Context's parent. It is a recursive operation that may
  /// invoke long chains of evaluation through all the Systems that are
  /// prerequisites to the specified port.
  ///
  /// Returns nullptr if the port is not connected to a value source. Aborts if
  /// the port does not exist.
  ///
  /// This is a framework implementation detail.  User code should not call it.
  const InputPortValue* EvalInputPort(
      const detail::InputPortEvaluatorInterface<T>* evaluator,
      const InputPortDescriptor<T>& descriptor) const {
    const InputPortValue* port_value =
        GetInputPortValue(descriptor.get_index());
    if (port_value == nullptr) return nullptr;
    if (port_value->requires_evaluation()) {
      DRAKE_DEMAND(evaluator != nullptr);
      evaluator->EvaluateSubsystemInputPort(parent_, descriptor);
    }
    return port_value;
  }

  /// Evaluates and returns the vector value of the input port with the given
  /// @p descriptor. This is a recursive operation that may invoke long chains
  /// of evaluation through all the Systems that are prerequisite to the
  /// specified port.
  ///
  /// Returns nullptr if the port is not connected.
  /// Throws std::bad_cast if the port is not vector-valued.
  /// Aborts if the port does not exist.
  ///
  /// This is a framework implementation detail.  User code should not call it;
  /// consider calling System::EvalVectorInput instead.
  const BasicVector<T>* EvalVectorInput(
      const detail::InputPortEvaluatorInterface<T>* evaluator,
      const InputPortDescriptor<T>& descriptor) const {
    const InputPortValue* port_value = EvalInputPort(evaluator, descriptor);
    if (port_value == nullptr) return nullptr;
    return port_value->template get_vector_data<T>();
  }

  /// Evaluates and returns the abstract value of the input port with the given
  /// @p descriptor. This is a recursive operation that may invoke long chains
  /// of evaluation through all the Systems that are prerequisite to the
  /// specified port.
  ///
  /// Returns nullptr if the port is not connected.
  /// Aborts if the port does not exist.
  ///
  /// This is a framework implementation detail.  User code should not call it;
  /// consider calling System::EvalAbstractInput instead.
  const AbstractValue* EvalAbstractInput(
      const detail::InputPortEvaluatorInterface<T>* evaluator,
      const InputPortDescriptor<T>& descriptor) const {
    const InputPortValue* port_value = EvalInputPort(evaluator, descriptor);
    if (port_value == nullptr) return nullptr;
    return port_value->get_abstract_data();
  }

  /// Evaluates and returns the data of the input port at @p index.
  /// This is a recursive operation that may invoke long chains of evaluation
  /// through all the Systems that are prerequisite to the specified port.
  ///
  /// Returns nullptr if the port is not connected.
  /// Throws std::bad_cast if the port does not have type V.
  /// Aborts if the port does not exist.
  ///
  /// This is a framework implementation detail.  User code should not call it;
  /// consider calling System::EvalInputValue instead.
  ///
  /// @tparam V The type of data expected.
  template <typename V>
  const V* EvalInputValue(
      const detail::InputPortEvaluatorInterface<T>* evaluator,
      const InputPortDescriptor<T>& descriptor) const {
    const AbstractValue* value = EvalAbstractInput(evaluator, descriptor);
    if (value == nullptr) return nullptr;
    return &(value->GetValue<V>());
  }

  // =========================================================================
  // Accessors and Mutators for Parameters.

  virtual const Parameters<T>& get_parameters() const = 0;
  virtual Parameters<T>& get_mutable_parameters() = 0;

  /// Returns the number of vector-valued parameters.
  int num_numeric_parameters() const {
    return get_parameters().num_numeric_parameters();
  }

  /// Returns a const pointer to the vector-valued parameter at @p index.
  /// Asserts if @p index doesn't exist.
  const BasicVector<T>& get_numeric_parameter(int index) const {
    return get_parameters().get_numeric_parameter(index);
  }

  /// Returns a mutable pointer to element @p index of the vector-valued
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
  void set_accuracy(const optional<double>& accuracy) { accuracy_ = accuracy; }

  /// Returns the accuracy setting (if any).
  /// @see set_accuracy() for details.
  const optional<double>& get_accuracy() const { return accuracy_; }

  // =========================================================================
  // Miscellaneous Public Methods

  /// Returns a deep copy of this Context. The clone's input ports will
  /// hold deep copies of the data that appears on this context's input ports
  /// at the time the clone is created.
  std::unique_ptr<Context<T>> Clone() const {
    return std::unique_ptr<Context<T>>(DoClone());
  }

  /// Returns a deep copy of this Context's State.
  std::unique_ptr<State<T>> CloneState() const {
    return std::unique_ptr<State<T>>(DoCloneState());
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

  /// Declares that @p parent is the context of the enclosing Diagram. The
  /// enclosing Diagram context is needed to evaluate inputs recursively.
  /// Aborts if the parent has already been set to something else.
  ///
  /// This is a dangerous implementation detail. Conceptually, a Context
  /// ought to be completely ignorant of its parent Context. However, we
  /// need this pointer so that we can cause our inputs to be evaluated in
  /// EvalInputPort.  See https://github.com/RobotLocomotion/drake/pull/3455.
  void set_parent(const Context<T>* parent) {
    DRAKE_DEMAND(parent_ == nullptr || parent_ == parent);
    parent_ = parent;
  }

  /// Throws an exception unless the given @p descriptor matches the inputs
  /// actually connected to this context in shape.
  /// Supports any scalar type of `descriptor`, but expects T by default.
  ///
  /// @tparam T1 the scalar type of the InputPortDescriptor to check.
  template<typename T1 = T>
  void VerifyInputPort(const InputPortDescriptor<T1>& descriptor) const {
    const int i = descriptor.get_index();
    const InputPortValue* port_value = GetInputPortValue(i);
    // If the port isn't connected, we don't have anything else to check.
    if (port_value == nullptr) { return; }
    // TODO(david-german-tri, sherm1): Consider checking sampling here.

    // In the vector-valued case, check the size.
    if (descriptor.get_data_type() == kVectorValued) {
      const BasicVector<T>* input_vector =
          port_value->template get_vector_data<T>();
      DRAKE_THROW_UNLESS(input_vector != nullptr);
      DRAKE_THROW_UNLESS(input_vector->size() == descriptor.size());
    }
    // In the abstract-valued case, there is nothing else to check.
  }

 protected:
  /// Contains the return-type-covariant implementation of Clone().
  virtual Context<T>* DoClone() const = 0;

  /// Contains the return-type-covariant implementation of CloneState().
  virtual State<T>* DoCloneState() const = 0;

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

  /// Allows derived classes to invoke the protected method on subcontexts.
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

  /// Allows derived classes to invoke the protected method on subcontexts.
  static void SetInputPortValue(Context<T>* context, int index,
                                std::unique_ptr<InputPortValue> port_value) {
    context->SetInputPortValue(index, std::move(port_value));
  }

 private:
  // Current time and step information.
  StepInfo<T> step_info_;

  // Accuracy setting.
  optional<double> accuracy_;

  // The context of the enclosing Diagram, used in EvalInputPort.
  // This pointer MUST be treated as a black box. If you call any substantive
  // methods on it, you are probably making a mistake.
  const Context<T>* parent_ = nullptr;
};

}  // namespace systems
}  // namespace drake
