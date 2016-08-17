#pragma once

#include <memory>
#include <vector>

#include "drake/systems/framework/context_base.h"
#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/leaf_state_vector.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/system_input.h"
#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {

/// The Context is a container for all of the data necessary to uniquely
/// determine the computations performed by a leaf System. Specifically, a
/// Context contains and owns the State, and also contains (but does not own)
/// pointers to the value sources for Inputs, as well as the simulation time
/// and the cache.
///
/// @tparam T The mathematical type of the context, which must be a valid Eigen
///           scalar.
// TODO(david-german-tri): Manage cache invalidation.
template <typename T>
class Context : public ContextBase<T> {
 public:
  Context() {}
  virtual ~Context() {}

  void SetInputPort(int index, std::unique_ptr<InputPort<T>> port) override {
    if (index < 0 || index >= get_num_input_ports()) {
      throw std::out_of_range("Input port out of range.");
    }
    // TODO(david-german-tri): Set invalidation callbacks.
    inputs_[index] = std::move(port);
  }

  /// Removes all the input ports, and deregisters them from the output ports
  /// on which they depend.
  void ClearInputPorts() { inputs_.clear(); }

  /// Clears the input ports and allocates @p n new input ports, not connected
  /// to anything.
  void SetNumInputPorts(int n) {
    ClearInputPorts();
    inputs_.resize(n);
  }

  int get_num_input_ports() const override {
    return static_cast<int>(inputs_.size());
  }

  const VectorInterface<T>* get_vector_input(int index) const override {
    if (index >= get_num_input_ports()) {
      throw std::out_of_range("Input port out of range.");
    }
    if (inputs_[index] == nullptr) {
      return nullptr;
    }
    return inputs_[index]->get_vector_data();
  }

  const State<T>& get_state() const override { return state_; }

  State<T>* get_mutable_state() override { return &state_; }

  /// Returns a const reference to the Cache, which is expected to contain
  /// precalculated values of interest. Use this only to access known-valid
  /// cache entries; use `get_mutable_cache()` if computations may be needed.
  const Cache<T>& get_cache() const { return cache_; }

  /// Access to the cache is always read-write, and is permitted even on
  /// const references to the Context. No invalidation of downstream dependents
  /// occurs until mutable access is requested for a particular cache entry.
  Cache<T>* get_mutable_cache() const { return &cache_; }

 protected:
  /// The caller owns the returned memory.
  ContextBase<T>* DoClone() const override {
    Context<T>* context = new Context<T>();

    // Make a deep copy of the state using LeafStateVector::Clone().
    if (this->get_state().continuous_state != nullptr) {
      const ContinuousState<T>& xc = *this->get_state().continuous_state;
      const int num_q = xc.get_generalized_position().size();
      const int num_v = xc.get_generalized_velocity().size();
      const int num_z = xc.get_misc_continuous_state().size();
      const LeafStateVector<T>& xc_vector =
          dynamic_cast<const LeafStateVector<T>&>(xc.get_state());
      context->get_mutable_state()->continuous_state.reset(
          new ContinuousState<T>(xc_vector.Clone(), num_q, num_v, num_z));
    }

    // Make deep copies of the inputs into FreestandingInputPorts.
    // TODO(david-german-tri): Preserve version numbers as well.
    for (const auto& port : this->inputs_) {
      if (port == nullptr) {
        context->inputs_.emplace_back(nullptr);
      } else {
        context->inputs_.emplace_back(
            new FreestandingInputPort<T>(
                port->get_vector_data()->CloneVector()));
      }
    }

    // Make deep copies of everything else using the default copy constructors.
    *context->get_mutable_step_info() = this->get_step_info();
    *context->get_mutable_cache() = this->get_cache();
    return context;
  }

 private:
  // Context objects are neither copyable nor moveable.
  Context(const Context& other) = delete;
  Context& operator=(const Context& other) = delete;
  Context(Context&& other) = delete;
  Context& operator=(Context&& other) = delete;

  // The external inputs to the System.
  std::vector<std::unique_ptr<InputPort<T>>> inputs_;

  // The internal state of the System.
  State<T> state_;

  // The cache. The System may insert arbitrary key-value pairs, and configure
  // invalidation on a per-line basis.
  mutable Cache<T> cache_;
};

}  // namespace systems
}  // namespace drake
