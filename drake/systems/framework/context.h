#pragma once

#include "drake/common/drake_throw.h"
#include "drake/systems/framework/input_port_evaluator_interface.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/system_input.h"
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
  T time_sec{};
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
  virtual ~Context() {}

  // =========================================================================
  // Accessors and Mutators for Time.

  /// Returns the current time in seconds.
  const T& get_time() const { return get_step_info().time_sec; }

  /// Set the current time in seconds.
  virtual void set_time(const T& time_sec)  {
    get_mutable_step_info()->time_sec = time_sec;
  }

  // =========================================================================
  // Accessors and Mutators for State.

  virtual const State<T>& get_state() const = 0;
  virtual State<T>* get_mutable_state() = 0;

  /// Sets the continuous state to @p xc, deleting whatever was there before.
  void set_continuous_state(std::unique_ptr<ContinuousState<T>> xc) {
    get_mutable_state()->set_continuous_state(std::move(xc));
  }

  /// Returns a mutable pointer to the continuous component of the state,
  /// which may be of size zero.
  ContinuousState<T>* get_mutable_continuous_state() {
    return get_mutable_state()->get_mutable_continuous_state();
  }

  /// Returns a mutable pointer to the continuous state, devoid of second-order
  /// structure. The vector may be of size zero.
  VectorBase<T>* get_mutable_continuous_state_vector() {
    return get_mutable_continuous_state()->get_mutable_vector();
  }

  /// Returns a const pointer to the continuous component of the state,
  /// which may be of size zero.
  const ContinuousState<T>* get_continuous_state() const {
    return get_state().get_continuous_state();
  }

  /// Returns a reference to the continuous state vector, devoid of second-order
  /// structure. The vector may be of size zero.
  const VectorBase<T>& get_continuous_state_vector() const {
    return get_continuous_state()->get_vector();
  }


  /// Returns a mutable pointer to the difference component of the state,
  /// which may be of size zero.
  DifferenceState<T>* get_mutable_difference_state() {
    return get_mutable_state()->get_mutable_difference_state();
  }

  /// Returns a mutable pointer to element @p index of the difference state.
  /// Asserts if @p index doesn't exist.
  BasicVector<T>* get_mutable_difference_state(int index) {
    DifferenceState<T>* xd = get_mutable_difference_state();
    return xd->get_mutable_difference_state(index);
  }

  /// Sets the discrete state to @p xd, deleting whatever was there before.
  void set_difference_state(std::unique_ptr<DifferenceState<T>> xd) {
    get_mutable_state()->set_difference_state(std::move(xd));
  }

  /// Returns a const pointer to the discrete difference component of the
  /// state at @p index.  Asserts if @p index doesn't exist.
  const VectorBase<T>* get_difference_state(int index) const {
    const DifferenceState<T>* xd = get_state().get_difference_state();
    return xd->get_difference_state(index);
  }

  /// Returns a mutable pointer to the modal component of the state,
  /// which may be of size zero.
  ModalState* get_mutable_modal_state() {
    return get_mutable_state()->get_mutable_modal_state();
  }

  /// Returns a mutable pointer to element @p index of the modal state.
  /// Asserts if @p index doesn't exist.
  template <typename U>
  U& get_mutable_modal_state(int index) {
    ModalState* xm = get_mutable_modal_state();
    return xm->get_mutable_modal_state(index).GetMutableValue<U>();
  }

  /// Sets the modal state to @p xm, deleting whatever was there before.
  void set_modal_state(std::unique_ptr<ModalState> xm) {
    get_mutable_state()->set_modal_state(std::move(xm));
  }

  /// Returns a const pointer to the discrete modal component of the
  /// state at @p index.  Asserts if @p index doesn't exist.
  template <typename U>
  const U& get_modal_state(int index) const {
    const ModalState* xm = get_state().get_modal_state();
    return xm->get_modal_state(index).GetValue<U>();
  }

  // =========================================================================
  // Accessors and Mutators for Input.

  /// Connects the input port @p port to this Context at the given @p index.
  /// Disconnects whatever input port was previously there, and deregisters
  /// it from the output port on which it depends.  In some Context
  /// implementations, may require a recursive search through a tree of
  /// subcontexts. Asserts if @p index is out of range.
  virtual void SetInputPort(int index, std::unique_ptr<InputPort> port) = 0;

  /// Returns the number of input ports.
  virtual int get_num_input_ports() const = 0;

  /// Evaluates and returns the input port identified by @p descriptor,
  /// using the given @p evaluator, which should be the Diagram containing
  /// the System that allocated this Context. The evaluation will be performed
  /// in this Context's parent. It is a recursive operation that may invoke
  /// long chains of evaluation through all the Systems that are prerequisites
  /// to the specified port.
  ///
  /// Returns nullptr if the port is not connected. Aborts if the port does
  /// not exist.
  ///
  /// This is a framework implementation detail.  User code should not call it.
  const InputPort* EvalInputPort(
      const detail::InputPortEvaluatorInterface<T>* evaluator,
      const SystemPortDescriptor<T>& descriptor) const {
    const InputPort* port = GetInputPort(descriptor.get_index());
    if (port == nullptr) return nullptr;
    if (port->requires_evaluation()) {
      DRAKE_DEMAND(evaluator != nullptr);
      evaluator->EvaluateSubsystemInputPort(parent_, descriptor);
    }
    return port;
  }

  /// Evaluates and returns the vector data of the input port with the given
  /// @p descriptor. This is a recursive operation that may invoke long chains
  /// of evaluation through all the Systems that are prerequisite to the
  /// specified port.
  ///
  /// Returns nullptr if the port is not connected.
  /// Throws std::bad_cast if the port is not vector-valued.
  /// Aborts if the port does not exist.
  ///
  /// This is a framework implementation detail.  User code should not call it.
  const BasicVector<T>* EvalVectorInput(
      const detail::InputPortEvaluatorInterface<T>* evaluator,
      const SystemPortDescriptor<T>& descriptor) const {
    const InputPort* port = EvalInputPort(evaluator, descriptor);
    if (port == nullptr) return nullptr;
    return port->template get_vector_data<T>();
  }

  /// Evaluates and returns the abstract data of the input port at @p index.
  /// This is a recursive operation that may invoke long chains of evaluation
  /// through all the Systems that are prerequisite to the specified port.
  ///
  /// Returns nullptr if the port is not connected.
  /// Aborts if the port does not exist.
  ///
  /// This is a framework implementation detail.  User code should not call it.
  const AbstractValue* EvalAbstractInput(
      const detail::InputPortEvaluatorInterface<T>* evaluator,
      const SystemPortDescriptor<T>& descriptor) const {
    const InputPort* port = EvalInputPort(evaluator, descriptor);
    if (port == nullptr) return nullptr;
    return port->get_abstract_data();
  }

  /// Evaluates and returns the data of the input port at @p index.
  /// This is a recursive operation that may invoke long chains of evaluation
  /// through all the Systems that are prerequisite to the specified port.
  ///
  /// Returns nullptr if the port is not connected.
  /// Throws std::bad_cast if the port does not have type V.
  /// Aborts if the port does not exist.
  ///
  /// This is a framework implementation detail.  User code should not call it.
  ///
  /// @tparam V The type of data expected.
  template <typename V>
  const V* EvalInputValue(
      const detail::InputPortEvaluatorInterface<T>* evaluator,
      const SystemPortDescriptor<T>& descriptor) const {
    const AbstractValue* value = EvalAbstractInput(evaluator, descriptor);
    if (value == nullptr) return nullptr;
    return &(value->GetValue<V>());
  }

  // =========================================================================
  // Miscellaneous Public Methods

  /// Returns a deep copy of this Context. The clone's input ports will
  /// hold deep copies of the data that appears on this context's input ports
  /// at the time the clone is created.
  std::unique_ptr<Context<T>> Clone() const {
    return std::unique_ptr<Context<T>>(DoClone());
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

  // Throws an exception unless the given @p descriptor matches this context.
  void VerifyInputPort(const SystemPortDescriptor<T>& descriptor) const {
    const int i = descriptor.get_index();
    const InputPort* port = GetInputPort(i);
    DRAKE_THROW_UNLESS(port != nullptr);
    // TODO(david-german-tri, sherm1): Consider checking sampling here.

    // In the vector-valued case, check the size.
    if (descriptor.get_data_type() == kVectorValued) {
      const BasicVector<T>* input_vector = port->template get_vector_data<T>();
      DRAKE_THROW_UNLESS(input_vector != nullptr);
      DRAKE_THROW_UNLESS(input_vector->size() == descriptor.get_size());
    }
    // In the abstract-valued case, there is nothing else to check.
  }

 protected:
  /// Contains the return-type-covariant implementation of Clone().
  virtual Context<T>* DoClone() const = 0;

  /// Returns a const reference to current time and step information.
  const StepInfo<T>& get_step_info() const { return step_info_; }

  /// Provides writable access to time and step information, with the side
  /// effect of invaliding any computation that is dependent on them.
  /// TODO(david-german-tri) Invalidate all cached time- and step-dependent
  /// computations.
  StepInfo<T>* get_mutable_step_info() { return &step_info_; }

  /// Returns the InputPort at the given @p index, which may be nullptr if
  /// it has never been set with SetInputPort.
  /// Asserts if @p index is out of range.
  virtual const InputPort* GetInputPort(int index) const = 0;

  /// Returns the InputPort at the given @p index from the given @p context.
  /// Returns nullptr if the given port has never been set with SetInputPort.
  /// Asserts if @p index is out of range.
  static const InputPort* GetInputPort(const Context<T>& context, int index) {
    return context.GetInputPort(index);
  }

 private:
  // Current time and step information.
  StepInfo<T> step_info_;

  // The context of the enclosing Diagram, used in EvalInputPort.
  // This pointer MUST be treated as a black box. If you call any substantive
  // methods on it, you are probably making a mistake.
  const Context<T>* parent_ = nullptr;
};

}  // namespace systems
}  // namespace drake

