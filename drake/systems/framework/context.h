#pragma once

#include <functional>
#include <memory>
#include <set>
#include <vector>

#include "drake/systems/framework/context_base.h"
#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/leaf_state_vector.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/system_input.h"
#include "drake/systems/framework/value.h"
#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {

template <typename T>
struct ContextShape {
  T time;
  std::vector<T> input_ports;
  StateShape<T> state;
};

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
  explicit Context(int num_input_ports) : cache_(new Cache()) {
    SetNumInputPorts(num_input_ports);
    root_cache_tickets_.time = cache_->MakeCacheTicket({});
    root_cache_tickets_.state.continuous_state = cache_->MakeCacheTicket({});
  }

  virtual ~Context() {}

  void SetInputPort(int index, std::unique_ptr<InputPort<T>> port) override {
    if (index < 0 || index >= get_num_input_ports()) {
      throw std::out_of_range("Input port out of range.");
    }
    auto callback = std::bind(&Context<T>::InvalidateInputPort, this, index);
    port->set_invalidation_callback(callback);
    inputs_[index] = std::move(port);
    // When we plug a new wire into an input port, all cache lines that depend
    // on that input port are invalidated.
    InvalidateInputPort(index);
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

  /// Returns a mutable pointer to the State. Invalidates all cache lines that
  /// depend upon State.
  ///
  /// Callers MUST NOT hold the returned pointer: it is vital to call this
  /// function whenever the state is modified. Otherwise, cache invalidation
  /// will not occur, and the System to which this Context belongs may read
  /// cache entries that are out-of-date.
  State<T>* get_mutable_state() override {
    InvalidateState();
    return &state_;
  }

  // Returns a ContextShape<bool> with the same dimensions as this Context,
  // and all fields set to false.
  ContextShape<bool> MakeContextShape() const {
    ContextShape<bool> shape;
    shape.input_ports.resize(get_num_input_ports());
    return shape;
  }

  CacheTicket CreateCacheEntry(
      const ContextShape<bool>& context_dependencies,
      const std::set<CacheTicket>& cache_dependencies) {
    const int num_inputs = context_dependencies.input_ports.size();
    DRAKE_ASSERT(get_num_input_ports() == num_inputs);
    std::set<CacheTicket> upstream = cache_dependencies;
    if (context_dependencies.time) {
      upstream.insert(root_cache_tickets_.time);
    }
    if (context_dependencies.state.continuous_state) {
      upstream.insert(root_cache_tickets_.state.continuous_state);
    }
    for (int i = 0; i < num_inputs; ++i) {
      if (context_dependencies.input_ports[i]) {
        upstream.insert(root_cache_tickets_.input_ports[i]);
      }
    }
    return cache_->MakeCacheTicket(upstream);
  }

  AbstractValue* SetCachedItem(CacheTicket ticket,
                               std::unique_ptr<AbstractValue> value) {
    return cache_->Set(ticket, std::move(value));
  }

  AbstractValue* GetCachedItem(CacheTicket ticket) {
    return cache_->Get(ticket);
  }

  void InvalidateTime() override {
    cache_->Invalidate(root_cache_tickets_.time);
  }

  void InvalidateState() override {
    cache_->Invalidate(root_cache_tickets_.state.continuous_state);
  }
  void InvalidateInputPort(int index) override {
    cache_->Invalidate(root_cache_tickets_.input_ports[index]);
  }

 protected:
  /// The caller owns the returned memory.
  ContextBase<T>* DoClone() const override {
    Context<T>* context = new Context<T>(get_num_input_ports());

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
    for (int i = 0; i < get_num_input_ports(); ++i) {
      InputPort<T>* port = inputs_[i].get();
      if (port == nullptr) {
        context->inputs_[i].reset(nullptr);
      } else {
        context->inputs_[i].reset(new FreestandingInputPort<T>(
            port->get_vector_data()->CloneVector()));
      }
    }

    // Make a deep copy of the Cache.
    context->cache_ = cache_->Clone();

    // Make a deep copy of the time.
    *context->get_mutable_step_info() = this->get_step_info();
    return context;
  }

 private:
  // Allocates n new input ports, not connected to anything. This function
  // may only be called once, because modifying the number of input ports
  // after context creation would destroy the semantics of cache tickets.
  void SetNumInputPorts(int n) {
    DRAKE_ASSERT(inputs_.empty());
    inputs_.resize(n);
    root_cache_tickets_.input_ports.resize(n);
    for (int i = 0; i < n; ++i) {
      root_cache_tickets_.input_ports[i] = cache_->MakeCacheTicket({});
    }
  }

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
  std::unique_ptr<Cache> cache_;

  // Cache tickets representing dependencies on the fields of this context.
  ContextShape<CacheTicket> root_cache_tickets_;
};

}  // namespace systems
}  // namespace drake
