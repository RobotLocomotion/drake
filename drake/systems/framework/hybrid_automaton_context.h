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
 *  - Q = {q₁, q₂, …, qᵣ} is a set of discrete-valued states (modes);
 *  - Q₀ ⊆ Q is a set of initial modes;
 *  - X = Rⁿ is a set of continuous-valued states;
 *  - U = Rᵐ is a set of continuous-valued inputs;
 *  - Y = Rᵖ is a set of continuous-valued outputs;
 *  - f(·, ·, ·) : Q × X × U → Rⁿ is a vector field representing the system's
 *    dynamics;
 *  - g(·, ·, ·) : Q × X × U → Rᵖ is a mapping from states and inputs to system
 *    outputs;
 *  - Init ⊆ Q × X is a set of initial states;
 *  - Invar(·) : Q → P(X) is an invariant defining the domain over which
 *    f(·, ·, ·) holds;
 *  - E ⊆ Q × Q is a set of edges describing discrete transitions between modes;
 *  - Guard(·) : E → P(X) is a guard condition defining the conditions where it
 *    is possible to make a discrete mode transition;
 *  - Reset(·, ·, ·) : E × X × U → P(X) is a reset map taking, for each edge,
 *    the state and input and producing new states following the mode
 *    transition.
 *
 * Note that the sets Invar(q) ⊆ Rⁿ and Init are assigned to each discrete state
 * q ∈ Q.  We refer to q ∈ Q as the mode of the HA.
 *
 * In the context of Drake's System framework, ModalSubsystem captures, for each
 * mode, the System (f, g) for each mode, the Init and Invar for System, and the
 * list of inputs and outputs of the System.
 *
 * The invariants and initial conditions for this ModalSubsystem are of
 * symbolic::Expression type, with each defining a semialgebraic set: a state
 * assignment is within the set iff it evaluates to a non-negative value.  For
 * instance, assume we have a System with one state.  Given a
 * `ModalSubsystem<double>` declared as `mss`, then the following code snippet
 * defines the semialgebric set `x <= 1`:
 *
 * @code
 *  std::vector<symbolic::Expression>* invariant mss->get_mutable_invariant();
 *  std::vector<symbolic::Variable> xcs = mss->get_symbolic_continuous_states();
 *  const symbolic::Expression x0{xcs[0]};  // Symbol for the continuous state.
 *  (*invariant).push_back({-x0 + 1.})  // The unbounded set x <= 1 defines this
 *                                      // invariant.
 * @endcode
 */
template <typename T>
class ModalSubsystem {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ModalSubsystem)

  typedef int ModeId;
  typedef int PortId;

  /// ModalSubsystem constructor.
  /// - @p mode_id is an identifier for the mode.
  /// - @p system is a System, the memory of which is shared with the caller.
  /// - @p invariant is a symbolic::Expression containing Invar(·) in terms of
  ///   the continuous and discrete system states.
  /// - @p initial_conditions is a symbolic::Expression containing Init(·) in
  ///   terms of the continuous and discrete system states.
  /// - @p input_port_ids is a subset of the input ports exposed as inputs of
  ///   the HA.
  /// - @p output_port_ids is a subset of the output ports exposed as outputs of
  ///   the HA.
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
    ThrowIfAnyPortIsInvalid();
    CreateSymbolicStates();
  }

  ModalSubsystem(ModeId mode_id, shared_ptr<System<T>> system,
                 std::vector<PortId> input_port_ids,
                 std::vector<PortId> output_port_ids)
      : ModalSubsystem(mode_id, system, {}, {}, input_port_ids,
                       output_port_ids) {}

  ModalSubsystem(ModeId mode_id, shared_ptr<System<T>> system)
      : ModalSubsystem(mode_id, system, {}, {}, {}, {}) {
    // Populates the input and output ports with the full complement of system
    // inputs and outputs.
    input_port_ids_.resize(system_->get_num_input_ports());
    std::iota(std::begin(input_port_ids_), std::end(input_port_ids_), 0);
    output_port_ids_.resize(system_->get_num_output_ports());
    std::iota(std::begin(output_port_ids_), std::end(output_port_ids_), 0);
  }

  /// Immutable accessor to the underlying the ModeId for this ModalSubsystem.
  ModeId get_mode_id() const { return mode_id_; }

  /// Immutable accessor to the pointer of the underlying System.
  const System<T>* get_system() const { return system_.get(); }
  const std::vector<PortId> get_input_port_ids() const {
    return input_port_ids_;
  }

  /// Accessors to the PortIds and attributes of the ports to be exposed by this
  /// ModalSubsystem.
  std::vector<PortId>* get_mutable_input_port_ids() { return &input_port_ids_; }
  PortId get_input_port_id(const int index) const {
    DRAKE_DEMAND(index >= 0 && index < get_num_input_ports());
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
    DRAKE_DEMAND(index >= 0 && index < get_num_output_ports());
    return output_port_ids_[index];
  }
  int get_num_output_ports() const {
    return static_cast<int>(output_port_ids_.size());
  }

  /// Accessors to the symbolic::Expressions for the invariants and initial
  /// condition state sets for this ModalSubsystem.
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
  // Checks the sanity of the provided port ids.
  void ThrowIfAnyPortIsInvalid() {
    for (auto inport : input_port_ids_) {
      DRAKE_THROW_UNLESS(0 <= inport &&
                         inport < system_->get_num_input_ports());
    }
    for (auto outport : output_port_ids_) {
      DRAKE_THROW_UNLESS(0 <= outport &&
                         outport < system_->get_num_output_ports());
    }
  }

  // Creates symbolic variables based on the expected context for this
  // subsystem.
  void CreateSymbolicStates() {
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
  void CreateSymbolicVariables(const std::string& variable_type,
                               const int size) {
    std::vector<std::vector<symbolic::Variable>> sym{};
    if (symbolic_variables_.find(variable_type) != symbolic_variables_.end()) {
      sym = symbolic_variables_.at(variable_type);
    }
    std::vector<symbolic::Variable> row;
    for (int i = 0; i < size; ++i) {
      std::ostringstream key{};
      key << variable_type << i;
      row.emplace_back(key.str());
    }
    sym.emplace_back(row);
    symbolic_variables_.insert(std::make_pair(variable_type, sym));
  }

  // An identifier for this mode.
  // TODO(jadecastro): Allow ModeId to take on a descriptor in place of the int.
  ModeId mode_id_;

  // The system model.
  shared_ptr<System<T>> system_;

  // Vector of expressions representing the semialgebraic set defining the
  // invariant for this mode.  Note that a strict coupling between invariant_
  // and symbolic_variables_ will be enforced within HybridAutomatonContext.
  // (See the class docstring for details.)
  std::vector<symbolic::Expression> invariant_;

  // Vector of expressions representing the semialgebraic set defining
  // the initial conditions for this mode.  Note that a strict coupling between
  // initial_conditions_ and symbolic_variables_ will be enforced within
  // HybridAutomatonContext.
  // (See the class docstring for details.)
  std::vector<symbolic::Expression> initial_conditions_;

  // Index sets for the input and output ports for this mode.  Each @p PortId
  // that make up these vectors must correspond to a valid Port Number of the
  // installed System.  There is otherwise no restriction on repetition,
  // ordering, or completeness over the complement of System ports.
  std::vector<PortId> input_port_ids_;
  std::vector<PortId> output_port_ids_;

  // A data structure containing the symbolic variables corresponding to the
  // states of the System.  Continuous and discrete states correspond,
  // respectively, to the std::string keys "xc" and "xd".  For convenience,
  // these designations are also prefixes for the labeled keys for each of the
  // symbolic variables.
  std::map<std::string, std::vector<std::vector<symbolic::Variable>>>
      symbolic_variables_;
};

// TODO(jadecastro): Include the Hybrid Automaton specialization of Context.

}  // namespace systems
}  // namespace drake
