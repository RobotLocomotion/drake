#pragma once

#include <memory>
#include <vector>

#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {

template <typename T>
struct InputPort {
  /// The port data, if the port is vector-valued.
  /// TODO(david-german-tri): Add abstract-valued ports.
  VectorInterface<T>* input = nullptr;

  /// The rate at which this port is sampled, in seconds.
  /// If zero, the port is continuous.
  double sample_time_sec;
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
  T time_sec;
};

/// The Context is a container for all of the data necessary to compute the
/// dynamics of any System. Specifically, a Context contains and owns the
/// State, and also contains (but does not own) pointers to the Input, as
/// well as the simulation time and the cache.
///
/// TODO(david-german-tri): Manage cache invalidation.
///
/// @tparam T The mathematical type of the context (e.g. double).
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

 private:
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
  Cache<T> cache_;
};

}  // namespace systems
}  // namesapce drake
