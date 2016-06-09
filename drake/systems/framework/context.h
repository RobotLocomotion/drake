#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/system_input.h"
#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {

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

  const SystemInput<T>& get_input() const { return input_; }
  SystemInput<T>* get_mutable_input() { return &input_; }

  ptrdiff_t get_num_input_ports() const { return input_.ports.size(); }

  /// Returns the vector data of the input port at @p index. Returns nullptr
  /// if that port is not a vector-valued port, or if it is not connected.
  /// Throws std::out_of_range if that port does not exist.
  const VectorInterface<T>* get_vector_input(ptrdiff_t index) const {
    if (index >= get_num_input_ports()) {
      throw std::out_of_range("Input port out of range.");
    }
    if (input_.ports[index] == nullptr) {
      return nullptr;
    }
    return input_.ports[index]->get_vector_data();
  }

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
  SystemInput<T> input_;

  /// The internal state of the System.
  State<T> state_;

  /// The cache. The System may insert arbitrary key-value pairs, and configure
  /// invalidation on a per-line basis.
  Cache<T> cache_;
};

}  // namespace systems
}  // namesapce drake
