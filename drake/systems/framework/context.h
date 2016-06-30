#pragma once

#include <memory>
#include <vector>

#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/leaf_state_vector.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/system_input.h"
#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {

template <typename T>
struct TimeStep {
  /// The time, in seconds. For typical T implementations based on
  /// doubles, time resolution will gradually degrade as time increases.
  /// TODO(sherm1): Consider whether this is sufficiently robust.
  T time_sec{};
};

/// The Context is a container for all of the data necessary to compute the
/// dynamics of any System. Specifically, a Context contains and owns the
/// State, and also contains (but does not own) pointers to the Input, as
/// well as the simulation time and the cache.
///
/// Context may be subclassed within the framework to support specialized kinds
/// of Systems, such as Diagrams, but should not be subclassed by users.
///
/// TODO(david-german-tri): Manage cache invalidation.
///
/// @tparam T The mathematical type of the context, which must be a valid Eigen
///           scalar.
template <typename T>
class Context {
 public:
  Context() {}
  virtual ~Context() {}

  /// Returns the current time in seconds.
  const T& get_time() const { return get_time_step().time_sec; }

  /// Set the current time in seconds, with the side effect that all cached
  /// time-dependent computations are invalidated.
  void set_time(const T& time_sec) {
    get_mutable_time_step()->time_sec = time_sec;
  }

  /// Returns a const reference to current time and step information.
  const TimeStep<T>& get_time_step() const { return time_step_; }

  /// Provides writable access to time and step information, with the side
  /// effect of invaliding any computation that is dependent on them.
  TimeStep<T>* get_mutable_time_step() { return &time_step_; }

  /// Connects the input port @p port to this Context at the given @p index.
  /// Disconnects whatever input port was previously there, and deregisters
  /// it from the output port on which it depends.
  void SetInputPort(int index, std::unique_ptr<InputPort<T>> port) {
    if (index < 0 || index >= get_num_input_ports()) {
      throw std::out_of_range("Input port out of range.");
    }
    // TODO(david-german-tri): Set invalidation callbacks.
    inputs_[index] = std::move(port);
  }

  /// Removes all the input ports, and deregisters them from the output ports
  /// on which they depend.
  void ClearInputPorts() {
    inputs_.clear();
  }

  /// Clears the input ports and allocates @p n new input ports, not connected
  /// to anything.
  void SetNumInputPorts(int n) {
    ClearInputPorts();
    inputs_.resize(n);
  }

  int get_num_input_ports() const { return static_cast<int>(inputs_.size()); }

  /// Returns the vector data of the input port at @p index. Returns nullptr
  /// if that port is not a vector-valued port, or if it is not connected.
  /// Throws std::out_of_range if that port does not exist.
  const VectorInterface<T>* get_vector_input(int index) const {
    if (index >= get_num_input_ports()) {
      throw std::out_of_range("Input port out of range.");
    }
    if (inputs_[index] == nullptr) {
      return nullptr;
    }
    return inputs_[index]->get_vector_data();
  }

  const State<T>& get_state() const { return state_; }

  /// Returns writable access to the State. No cache invalidation occurs until
  /// mutable access is requested for particular blocks of state variables.
  State<T>* get_mutable_state() { return &state_; }

  /// Returns a const reference to the Cache, which is expected to contain
  /// precalculated values of interest. Use this only to access known-valid
  /// cache entries; use `get_mutable_cache()` if computations may be needed.
  const Cache<T>& get_cache() const { return &cache_; }

  /// Access to the cache is always read-write, and is permitted even on
  /// const references to the Context. No invalidation of downstream dependents
  /// occurs until mutable access is requested for a particular cache entry.
  Cache<T>* get_mutable_cache() const { return &cache_; }

  /// Returns a deep copy of this Context. The clone's input ports will hold
  /// deep copies of the data that appears on this context's port at the time
  /// the clone is created.
  std::unique_ptr<Context<T>> Clone() const {
    return std::unique_ptr<Context<T>>(DoClone());
  }

 private:
  /// The Context implementation for Diagrams must override this method, since
  /// the state of a Diagram will not be a LeafStateVector. The caller owns the
  /// returned memory.
  virtual Context<T>* DoClone() const {
    Context<T>* context = new Context<T>();

    // Make a deep copy of the state using LeafStateVector::Clone().
    const ContinuousState<T>& xc = *this->get_state().continuous_state;
    const int num_q = xc.get_generalized_position().size();
    const int num_v = xc.get_generalized_velocity().size();
    const int num_z = xc.get_misc_continuous_state().size();
    const LeafStateVector<T>& xc_vector =
        dynamic_cast<const LeafStateVector<T>&>(xc.get_state());
    context->get_mutable_state()->continuous_state.reset(
        new ContinuousState<T>(xc_vector.Clone(), num_q, num_v, num_z));

    // Make deep copies of the inputs into FreestandingInputPorts.
    // TODO(david-german-tri): Preserve version numbers as well.
    for (const auto& port : this->inputs_) {
      context->inputs_.emplace_back(
          new FreestandingInputPort<T>(
              port->get_vector_data()->Clone(), port->get_sample_time_sec()));
    }

    // Make deep copies of everything else using the default copy constructors.
    *context->get_mutable_time_step() = this->get_time_step();
    *context->get_mutable_cache() = *this->get_mutable_cache();
    return context;
  }

  // Context objects are neither copyable nor moveable.
  Context(const Context& other) = delete;
  Context& operator=(const Context& other) = delete;
  Context(Context&& other) = delete;
  Context& operator=(Context&& other) = delete;

  // Current time and step information.
  TimeStep<T> time_step_;

  // The external inputs to the System.
  std::vector<std::unique_ptr<InputPort<T>>> inputs_;

  // The internal state of the System.
  State<T> state_;

  /// The cache. The System may insert arbitrary key-value pairs, and configure
  /// invalidation on a per-line basis.
  mutable Cache<T> cache_;
};

}  // namespace systems
}  // namesapce drake
