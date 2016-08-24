#pragma once

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

  /// Returns the current time in seconds.
  const T& get_time() const { return get_step_info().time_sec; }

  /// Set the current time in seconds.
  virtual void set_time(const T& time_sec)  {
    get_mutable_step_info()->time_sec = time_sec;
  }

  /// Connects the input port @p port to this Context at the given @p index.
  /// Disconnects whatever input port was previously there, and deregisters
  /// it from the output port on which it depends.
  virtual void SetInputPort(int index, std::unique_ptr<InputPort> port) = 0;

  /// Returns the number of input ports.
  virtual int get_num_input_ports() const = 0;

  /// Returns the vector data of the input port at @p index. Returns nullptr
  /// if that port is not a vector-valued port, or if it is not connected.
  /// Throws std::out_of_range if that port does not exist.
  virtual const BasicVector<T>* get_vector_input(int index) const = 0;

  /// Returns the abstract data of the input port at @p index. Returns nullptr
  /// if that port is not connected. Throws std::out_of_range if that port
  /// does not exist.
  virtual const AbstractValue* get_abstract_input(int index) const = 0;

  /// Returns the data of the input port at @p index, or nullptr if that port
  /// is not connected.
  ///
  /// @tparam V The type of data expected.
  template <typename V>
  const V* get_input_value(int index) const {
    const AbstractValue* value = get_abstract_input(index);
    if (value == nullptr) {
      return nullptr;
    }
    return &(value->GetValue<V>());
  }

  virtual const State<T>& get_state() const = 0;

  /// Returns writable access to the State. No cache invalidation occurs until
  /// mutable access is requested for particular blocks of state variables.
  virtual State<T>* get_mutable_state() = 0;

  /// Returns a mutable pointer to the continuous component of the state.
  ContinuousState<T>* get_mutable_continuous_state() {
    return get_mutable_state()->continuous_state.get();
  }

  /// Returns a const reference to the continuous component of the state.
  const ContinuousState<T>& get_continuous_state() const {
    return *get_state().continuous_state;
  }

  /// Returns a deep copy of this Context. The clone's input ports will
  /// hold deep copies of the data that appears on this context's input ports
  /// at the time the clone is created.
  std::unique_ptr<Context<T>> Clone() const {
    return std::unique_ptr<Context<T>>(DoClone());
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

 private:
  // Current time and step information.
  StepInfo<T> step_info_;
};

}  // namespace systems
}  // namespace drake
