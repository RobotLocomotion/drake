#pragma once

#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/symbolic_formula.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {

using std::make_unique;
using std::move;
using std::shared_ptr;
using std::unique_ptr;

/**
 * ModalSubsystem represents the mode of a hybrid system.
 *
 * Let P(X) denote the power set (set of all subsets) of X and let R denote the
 * space of real numbers.  Define a hybrid automaton as a tuple HA = (Q, Q₀, X,
 * U, Y, f, g, Init, Invar, E, Guard, Reset), where:
 *
 * - Q = {q₁, q₂, …, qᵣ} is a set of discrete-valued states (modes);
 * - Q₀ ⊆ Q is a set of initial modes;
 * - X = Rⁿ is a set of continuous-valued states;
 * - U = Rᵐ is a set of continuous-valued inputs;
 * - Y = Rᵖ is a set of continuous-valued outputs;
 * - f(·, ·, ·) : Q × X × U → Rⁿ is a vector field representing the system's
 *   dynamics;
 * - g(·, ·, ·) : Q × X × U → Rᵖ is a mapping from states and inputs to system
 *   outputs;
 * - Init ⊆ Q × X is a set of initial states;
 * - Invar(·) : Q → P(X) is an invariant defining the domain over which
 *   f(·, ·, ·) holds;
 * - E ⊆ Q × Q is a set of edges describing discrete transitions between modes;
 * - Guard(·) : E → P(X) is a guard condition defining the conditions where it
 *   is possible to make a discrete mode transition;
 * - Reset(·, ·, ·) : E × X × U → P(X) is a reset map taking, for each edge, the
 *   state and input and producing new states following the mode transition.
 *
 * Note that the sets Invar(q) ⊆ Rⁿ and Init are assigned to each discrete state
 * q ∈ Q.  We refer to q ∈ Q as the mode of the HA.
 *
 * In the context of Drake's System framework, ModalSubsystem captures, for each
 * mode, the System (f, g) for each mode, the Init and Invar for System, and the
 * list of inputs and outputs of the System.
 */
template <typename T>
class ModalSubsystem {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ModalSubsystem)

  typedef int ModeId;
  typedef int PortId;

  // Constructor
  ModalSubsystem(ModeId mode_id, shared_ptr<System<T>> system,
                 std::vector<symbolic::Expression> invariant,
                 std::vector<symbolic::Expression> initial_conditions,
                 std::vector<PortId> input_port_ids,
                 std::vector<PortId> output_port_ids)
      : mode_id_(mode_id),
        system_(move(system)),
        invariant_(invariant),
        initial_conditions_(initial_conditions),
        input_port_ids_(input_port_ids),
        output_port_ids_(output_port_ids) {
    CreateSymbolicStatesAndInputs();
  }

  ModalSubsystem(ModeId mode_id, shared_ptr<System<T>> system,
                 std::vector<PortId> input_port_ids,
                 std::vector<PortId> output_port_ids)
      : mode_id_(mode_id),
        system_(move(system)),
        input_port_ids_(input_port_ids),
        output_port_ids_(output_port_ids) {
    CreateSymbolicStatesAndInputs();
  }

  ModalSubsystem(ModeId mode_id, shared_ptr<System<T>> system)
      : mode_id_(mode_id), system_(move(system)) {
    CreateSymbolicStatesAndInputs();
    PopulateDefaultPorts();
  }

  /// Accessors for the underlying data.
  ModeId get_mode_id() const { return mode_id_; }
  System<T>* get_system() const { return system_.get(); }
  const std::vector<PortId> get_input_port_ids() const {
    return input_port_ids_;
  }
  std::vector<PortId>* get_mutable_input_port_ids() { return &input_port_ids_; }
  PortId get_input_port_id(const int index) const {
    DRAKE_DEMAND(index >= 0 &&
                 index < static_cast<int>(input_port_ids_.size()));
    return input_port_ids_[index];
  }
  int get_num_input_ports() const {
    return static_cast<int>(input_port_ids_.size());
  }
  const std::vector<PortId> get_output_port_ids() const {
    return output_port_ids_;
  }
  std::vector<PortId>* get_mutable_output_port_ids() {
    return &output_port_ids_;
  }
  PortId get_output_port_id(const int index) const {
    DRAKE_DEMAND(index >= 0 &&
                 index < static_cast<int>(output_port_ids_.size()));
    return output_port_ids_[index];
  }
  int get_num_output_ports() const {
    return static_cast<int>(output_port_ids_.size());
  }

  /// Accessors for the symbolic::Expressions for the invariants and initial
  /// condition state sets for this ModalSubsystem. Their defining sets are
  /// semialgebraic: a state assignment is within the set iff it evaluates to a
  /// non-negative value.
  const std::vector<symbolic::Expression> get_invariant() const {
    return invariant_;
  }
  std::vector<symbolic::Expression>* get_mutable_invariant() {
    return &invariant_;
  }
  const std::vector<symbolic::Expression> get_initial_conditions() const {
    return initial_conditions_;
  }
  std::vector<symbolic::Expression>* get_mutable_initial_conditions() {
    return &initial_conditions_;
  }

  /// Accessors for the auto-generated symbolic states.
  const std::vector<symbolic::Variable>& get_symbolic_continuous_states()
      const {
    return symbolic_variables_.at("xc")[0];
  }
  const std::vector<symbolic::Variable>& get_symbolic_discrete_states_at(
      const int index) const {
    return symbolic_variables_.at("xd")[index];
  }
  int get_num_symbolic_discrete_states() const {
    return symbolic_variables_.at("xd").size();
  }

  /// Returns a clone that includes a deep copy of all the underlying data.
  unique_ptr<ModalSubsystem<T>> Clone() const {
    DRAKE_DEMAND(system_ != nullptr);
    shared_ptr<System<T>> sys = system_;
    ModalSubsystem<T>* clone =
        new ModalSubsystem<T>(mode_id_, sys, invariant_, initial_conditions_,
                              input_port_ids_, output_port_ids_);
    DRAKE_DEMAND(clone != nullptr);
    return unique_ptr<ModalSubsystem<T>>(clone);
  }

 private:
  // Create symbolic variables based on the expected context for this subsystem.
  void CreateSymbolicStatesAndInputs() {
    // Create a temporary context to extract the needed dimensions.
    unique_ptr<Context<T>> context = system_->AllocateContext();

    // Create symbolic variables for the continuous and discrete states.
    CreateSymbolicVariables("xc",
                            context->get_continuous_state_vector().size());
    for (int i = 0; i < context->get_num_discrete_state_groups(); ++i) {
      CreateSymbolicVariables("xd", context->get_discrete_state(i)->size());
    }
    context.reset();
  }

  // Creates symbolic variables according to the variable_type key word.
  void CreateSymbolicVariables(const std::string variable_type,
                               const int size) {
    std::vector<std::vector<symbolic::Variable>> sym{};
    if (symbolic_variables_.find(variable_type) != symbolic_variables_.end()) {
      sym = symbolic_variables_.at(variable_type);
    }
    std::vector<symbolic::Variable> row;
    for (int i = 0; i < size; ++i) {
      std::ostringstream key{};
      key << variable_type << i;
      symbolic::Variable var{key.str()};
      row.emplace_back(var);
    }
    sym.emplace_back(row);
    symbolic_variables_.insert(std::make_pair(variable_type, sym));
  }

  // Populates the input and output ports with the full complement of system
  // inputs and outputs.
  void PopulateDefaultPorts() {
    input_port_ids_.resize(system_->get_num_input_ports());
    std::iota(std::begin(input_port_ids_), std::end(input_port_ids_), 0);
    output_port_ids_.resize(system_->get_num_output_ports());
    std::iota(std::begin(output_port_ids_), std::end(output_port_ids_), 0);
  }

  // An identifier for this mode.
  // TODO(jadecastro): Allow ModeId to take on a descriptor in place of the int.
  ModeId mode_id_;
  // The system model.
  shared_ptr<System<T>> system_;
  // Expression representing the invariant for this mode.
  std::vector<symbolic::Expression> invariant_;
  // Expression representing the initial conditions for this mode.
  std::vector<symbolic::Expression> initial_conditions_;
  // Index set of the input and output ports.
  std::vector<PortId> input_port_ids_;
  std::vector<PortId> output_port_ids_;
  // A vector of symbolic variables for each of the continuous states in the
  // system.
  std::map<std::string, std::vector<std::vector<symbolic::Variable>>>
      symbolic_variables_;
};

// TODO(jadecastro): Include the Hybrid Automaton specialization of Context.

}  // namespace systems
}  // namespace drake
