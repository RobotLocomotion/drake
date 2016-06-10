#pragma once

#include <memory>
#include <vector>

#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/leaf_state_vector.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {

template <typename T>
struct InputPort {
  /// The port data, if the port is vector-valued.
  /// TODO(david-german-tri): Add abstract-valued ports.
  const VectorInterface<T>* vector_input = nullptr;

  /// The rate at which this port is sampled, in seconds.
  /// If zero, the port is continuous.
  double sample_time_sec{};
};

/// The Input is a container for pointers to all the data that is connected to
/// a particular System from other Systems. The input data itself is not owned
/// by the Input struct.
template <typename T>
struct Input {
  std::vector<InputPort<T>> ports;
};

/// The TimeStep holds the current time and may include other information about
/// the current solver step.
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

  /// Returns a const reference to the Input object, which references the
  /// current input values.
  const Input<T>& get_input() const { return input_; }

  /// Provides writable access to the Input object. This can be used to set
  /// the input specifications (for example, number of ports) but cannot be
  /// used to set the input values, which are provided externally.
  Input<T>* get_mutable_input() { return &input_; }

  /// Returns a const reference to the State, which contains the current values
  /// for continuous and discrete state variables.
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
  /// pointers to the same output ports as the original Context.
  ///
  /// The Context implementation for Diagrams must override this method, since
  /// the state of a Diagram will not be a LeafStateVector.
  virtual std::unique_ptr<Context<T>> Clone() const {
    std::unique_ptr<Context<T>> context(new Context);

    // Make a deep copy of the state using LeafStateVector::Clone().
    const ContinuousState<T>& xc = *this->get_state().continuous_state;
    const int64_t num_q = xc.get_generalized_position().size();
    const int64_t num_v = xc.get_generalized_velocity().size();
    const int64_t num_z = xc.get_misc_continuous_state().size();
    const LeafStateVector<T>& xc_vector =
        dynamic_cast<const LeafStateVector<T>&>(xc.get_state());
    context->get_mutable_state()->continuous_state.reset(
        new ContinuousState<T>(xc_vector.Clone(), num_q, num_v, num_z));

    // Make deep copies of everything else using the default copy constructors.
    *context->get_mutable_time_step() = this->get_time_step();
    *context->get_mutable_input() = this->get_input();
    *context->get_mutable_cache() = *this->get_mutable_cache();
    return context;
  }

 private:
  // Context objects are neither copyable nor moveable.
  Context(const Context& other) = delete;
  Context& operator=(const Context& other) = delete;
  Context(Context&& other) = delete;
  Context& operator=(Context&& other) = delete;

  // Current time and step information.
  TimeStep<T> time_step_;

  // The external inputs to the System.
  Input<T> input_;

  // The internal state of the System.
  State<T> state_;

  // The cache. This holds computations that were performed using the
  // time, input, and state values above.
  mutable Cache<T> cache_;
};

}  // namespace systems
}  // namesapce drake
