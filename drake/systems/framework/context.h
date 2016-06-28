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

template <typename T>
struct Time {
  /// The time, in seconds.  For typical T implementations based on
  /// doubles, time precision will gradually degrade as time increases.
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

  const Time<T>& get_time() const { return time_; }
  Time<T>* get_mutable_time() { return &time_; }

  const Input<T>& get_input() const { return input_; }
  Input<T>* get_mutable_input() { return &input_; }

  const State<T>& get_state() const { return state_; }
  State<T>* get_mutable_state() { return &state_; }

  /// Access to the cache is always read-write, and is permitted even on
  /// const references to the Context.
  Cache<T>* get_mutable_cache() const { return &cache_; }

  /// Returns a deep copy of this Context. The clone's input ports will hold
  /// pointers to the same output ports as the original Context.
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

    // Make deep copies of everything else using the default copy constructors.
    *context->get_mutable_time() = this->get_time();
    *context->get_mutable_input() = this->get_input();
    *context->get_mutable_cache() = *this->get_mutable_cache();
    return context;
  }

  // Context objects are neither copyable nor moveable.
  Context(const Context& other) = delete;
  Context& operator=(const Context& other) = delete;
  Context(Context&& other) = delete;
  Context& operator=(Context&& other) = delete;

  /// The current time.
  Time<T> time_;

  /// The external inputs to the System.
  Input<T> input_;

  /// The internal state of the System.
  State<T> state_;

  /// The cache. The System may insert arbitrary key-value pairs, and configure
  /// invalidation on a per-line basis.
  mutable Cache<T> cache_;
};

}  // namespace systems
}  // namesapce drake
