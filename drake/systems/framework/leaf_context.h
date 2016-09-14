#pragma once

#include <memory>
#include <vector>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/system_input.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {

/// %LeafContext is a container for all of the data necessary to uniquely
/// determine the computations performed by a leaf System. Specifically, a
/// %LeafContext contains and owns the State, and also contains (but does not
/// own) pointers to the value sources for Inputs, as well as the simulation
/// time and the cache.
///
/// @tparam T The mathematical type of the context, which must be a valid Eigen
///           scalar.
// TODO(david-german-tri): Manage cache invalidation.
template <typename T>
class LeafContext : public Context<T> {
 public:
  LeafContext() {}
  virtual ~LeafContext() {}

  void SetInputPort(int index, std::unique_ptr<InputPort> port) override {
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

  const BasicVector<T>* get_vector_input(int index) const override {
    DRAKE_DEMAND(index >= 0 && index < get_num_input_ports());
    if (inputs_[index] == nullptr) {
      return nullptr;
    }
    return inputs_[index]->template get_vector_data<T>();
  }

  const AbstractValue* get_abstract_input(int index) const override {
    DRAKE_DEMAND(index >= 0 && index < get_num_input_ports());
    if (inputs_[index] == nullptr) {
      return nullptr;
    }
    return inputs_[index]->get_abstract_data();
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
  Context<T>* DoClone() const override {
    LeafContext<T>* context = new LeafContext<T>();

    // Make a deep copy of the state using BasicVector::Clone().
    if (this->get_state().continuous_state != nullptr) {
      const ContinuousState<T>& xc = *this->get_state().continuous_state;
      const int num_q = xc.get_generalized_position().size();
      const int num_v = xc.get_generalized_velocity().size();
      const int num_z = xc.get_misc_continuous_state().size();
      const BasicVector<T>& xc_vector =
          dynamic_cast<const BasicVector<T>&>(xc.get_state());
      context->get_mutable_state()->continuous_state.reset(
          new ContinuousState<T>(xc_vector.Clone(), num_q, num_v, num_z));
    }

    // Make deep copies of the inputs into FreestandingInputPorts.
    // TODO(david-german-tri): Preserve version numbers as well.
    for (const auto& port : this->inputs_) {
      if (port == nullptr) {
        context->inputs_.emplace_back(nullptr);
      } else {
        context->inputs_.emplace_back(new FreestandingInputPort(
            port->template get_vector_data<T>()->Clone()));
      }
    }

    // Make deep copies of everything else using the default copy constructors.
    *context->get_mutable_step_info() = this->get_step_info();
    *context->get_mutable_cache() = this->get_cache();
    return context;
  }

 private:
  // LeafContext objects are neither copyable nor moveable.
  LeafContext(const LeafContext& other) = delete;
  LeafContext& operator=(const LeafContext& other) = delete;
  LeafContext(LeafContext&& other) = delete;
  LeafContext& operator=(LeafContext&& other) = delete;

  // The external inputs to the System.
  std::vector<std::unique_ptr<InputPort>> inputs_;

  // The internal state of the System.
  State<T> state_;

  // The cache. The System may insert arbitrary key-value pairs, and configure
  // invalidation on a per-line basis.
  mutable Cache<T> cache_;
};

}  // namespace systems
}  // namespace drake
