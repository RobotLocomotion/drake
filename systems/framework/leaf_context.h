#pragma once

#include <memory>
#include <set>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/input_port_evaluator_interface.h"
#include "drake/systems/framework/input_port_value.h"
#include "drake/systems/framework/parameters.h"
#include "drake/systems/framework/state.h"
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
  // LeafContext objects are neither copyable nor moveable.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LeafContext)

  LeafContext()
      : state_(std::make_unique<State<T>>()),
        parameters_(std::make_unique<Parameters<T>>()) {}
  ~LeafContext() override {}

  /// Removes all the input ports, and deregisters them from the output ports
  /// on which they depend.
  void ClearInputPorts() { input_values_.clear(); }

  /// Clears the input ports and allocates @p n new input ports, not connected
  /// to anything.
  void SetNumInputPorts(int n) {
    ClearInputPorts();
    input_values_.resize(n);
  }

  int get_num_input_ports() const override {
    return static_cast<int>(input_values_.size());
  }

  const State<T>& get_state() const final {
    DRAKE_ASSERT(state_ != nullptr);
    return *state_;
  }

  State<T>& get_mutable_state() final {
    DRAKE_ASSERT(state_ != nullptr);
    return *state_.get();
  }

  // =========================================================================
  // Accessors and Mutators for Parameters.

  /// Sets the parameters to @p params, deleting whatever was there before.
  void set_parameters(std::unique_ptr<Parameters<T>> params) {
    parameters_ = std::move(params);
  }

  /// Returns the entire Parameters object.
  const Parameters<T>& get_parameters() const final {
    return *parameters_;
  }

  /// Returns the entire Parameters object.
  Parameters<T>& get_mutable_parameters() final {
    return *parameters_;
  }

 protected:
  /// The caller owns the returned memory.
  Context<T>* DoClone() const override {
    LeafContext<T>* clone = new LeafContext<T>();

    // Make a deep copy of the state.
    clone->state_ = this->CloneState();

    // Make deep copies of the parameters.
    clone->set_parameters(parameters_->Clone());

    // Make deep copies of the inputs into FreestandingInputPortValues.
    // TODO(david-german-tri): Preserve version numbers as well.
    for (const auto& port : this->input_values_) {
      if (port == nullptr) {
        clone->input_values_.emplace_back(nullptr);
      } else {
        clone->input_values_.emplace_back(new FreestandingInputPortValue(
            port->get_abstract_data()->Clone()));
      }
    }

    // Make deep copies of everything else using the default copy constructors.
    *clone->get_mutable_step_info() = this->get_step_info();
    clone->set_accuracy(this->get_accuracy());
    return clone;
  }

  /// The caller owns the returned memory.
  State<T>* DoCloneState() const override {
    State<T>* clone = new State<T>();

    // Make a deep copy of the continuous state using BasicVector::Clone().
    const ContinuousState<T>& xc = this->get_continuous_state();
    const int num_q = xc.get_generalized_position().size();
    const int num_v = xc.get_generalized_velocity().size();
    const int num_z = xc.get_misc_continuous_state().size();
    const BasicVector<T>& xc_vector =
        dynamic_cast<const BasicVector<T>&>(xc.get_vector());
    clone->set_continuous_state(std::make_unique<ContinuousState<T>>(
        xc_vector.Clone(), num_q, num_v, num_z));

    // Make deep copies of the discrete and abstract states.
    clone->set_discrete_state(get_state().get_discrete_state().Clone());
    clone->set_abstract_state(get_state().get_abstract_state().Clone());

    return clone;
  }

  const InputPortValue* GetInputPortValue(int index) const override {
    DRAKE_ASSERT(index >= 0 && index < get_num_input_ports());
    return input_values_[index].get();
  }

 private:
  void SetInputPortValue(int index,
                         std::unique_ptr<InputPortValue> port) final {
    DRAKE_DEMAND(index >= 0 && index < get_num_input_ports());
    input_values_[index] = std::move(port);
  }

  // The external inputs to the System.
  std::vector<std::unique_ptr<InputPortValue>> input_values_;

  // The internal state of the System.
  std::unique_ptr<State<T>> state_;

  // The parameters of the system.
  std::unique_ptr<Parameters<T>> parameters_;
};

}  // namespace systems
}  // namespace drake
