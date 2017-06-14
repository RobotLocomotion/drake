#include "drake/systems/framework/sparsity_matrix.h"

#include <sstream>

#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {

SparsityMatrix::SparsityMatrix(const System<symbolic::Expression>& system)
    : context_(system.CreateDefaultContext()),
      output_(system.AllocateOutput(*context_)),
      input_expressions_(system.get_num_input_ports()),
      output_port_types_(system.get_num_output_ports()),
      context_is_abstract_(IsAbstract(system, *context_)) {
  // Stop analysis if the Context is in any way abstract, because we have no way
  // to initialize the abstract elements.
  if (context_is_abstract_) return;

  // Time
  context_->set_time(symbolic::Variable("t"));

  // Input
  InitializeVectorInputs(system);

  // State
  InitializeContinuousState();
  InitializeDiscreteState();

  // Parameters
  // TODO(david-german-tri): Initialize parameters, once #5072 is resolved.

  // Outputs
  // -- Record the output ports and compute the outputs.
  for (int i = 0; i < system.get_num_output_ports(); ++i) {
    const OutputPort<symbolic::Expression>& port = system.get_output_port(i);
    output_port_types_[i] = port.get_data_type();
    port.Calc(*context_, output_->GetMutableData(i));
  }

  // TODO(david-german-tri): Other System computations, such as derivatives.
}

void SparsityMatrix::InitializeVectorInputs(
    const System<symbolic::Expression>& system) {
  // For each input vector i, set each element j to a symbolic expression whose
  // value is the variable "ui_j".
  for (int i = 0; i < system.get_num_input_ports(); ++i) {
    DRAKE_ASSERT(system.get_input_port(i).get_data_type() == kVectorValued);
    const int n = system.get_input_port(i).size();
    auto value = system.AllocateInputVector(system.get_input_port(i));
    for (int j = 0; j < n; ++j) {
      std::ostringstream name;
      name << "u" << i << "_" << j;
      value->SetAtIndex(j, symbolic::Variable(name.str()));
      // Save a copy of the input expression.
      input_expressions_[i].push_back(value->GetAtIndex(j));
    }
    context_->FixInputPort(i, std::move(value));
  }
}

void SparsityMatrix::InitializeContinuousState() {
  // Set each element i in the continuous state to a symbolic expression whose
  // value is the variable "xci".
  VectorBase<symbolic::Expression>& xc =
      *context_->get_mutable_continuous_state_vector();
  for (int i = 0; i < xc.size(); ++i) {
    std::ostringstream name;
    name << "xc" << i;
    xc[i] = symbolic::Variable(name.str());
  }
}

void SparsityMatrix::InitializeDiscreteState() {
  // For each discrete state vector i, set each element j to a symbolic
  // expression whose value is the variable "xdi_j".
  auto& xd = *context_->get_mutable_discrete_state();
  for (int i = 0; i < context_->get_num_discrete_state_groups(); ++i) {
    auto& xdi = *xd.get_mutable_vector(i);
    for (int j = 0; j < xdi.size(); ++j) {
      std::ostringstream name;
      name << "xd" << i << "_" << j;
      xdi[j] = symbolic::Variable(name.str());
    }
  }
}

bool SparsityMatrix::IsAbstract(const System<symbolic::Expression>& system,
                                const Context<symbolic::Expression>& context) {
  // If any of the input ports are abstract, we cannot do sparsity analysis of
  // this Context.
  for (int i = 0; i < system.get_num_input_ports(); ++i) {
    if (system.get_input_port(i).get_data_type() == kAbstractValued) {
      return true;
    }
  }

  // If there is any abstract state, we cannot do sparsity analysis of this
  // Context.
  if (context.get_num_abstract_state_groups() > 0) {
    return true;
  }

  // TODO(david-german-tri): Check parameters once #5072 is resolved.

  return false;
}

bool SparsityMatrix::IsConnectedInputToOutput(int input_port_index,
                                              int output_port_index) const {
  DRAKE_ASSERT(
      input_port_index >= 0 &&
      input_port_index < static_cast<int>(input_expressions_.size()));
  DRAKE_ASSERT(
      output_port_index >= 0 &&
      output_port_index < static_cast<int>(output_port_types_.size()));

  // If the Context contains any abstract values, any input might be connected
  // to any output.
  if (context_is_abstract_) {
    return true;
  }

  // If the given output port is abstract, we can't determine which inputs
  // influenced it.
  if (output_port_types_[output_port_index] == kAbstractValued) {
    return true;
  }

  // Extract all the variables that appear in any element of the given
  // output_port_index.
  symbolic::Variables output_variables;
  const BasicVector<symbolic::Expression>* output_exprs =
      output_->get_vector_data(output_port_index);
  for (int j = 0; j < output_exprs->size(); ++j) {
    output_variables.insert(output_exprs->GetAtIndex(j).GetVariables());
  }

  // Check whether any of the variables in any of the input_expressions_ are
  // among the output_variables.
  for (const auto& expr : input_expressions_[input_port_index]) {
    symbolic::Variables input_variables = expr.GetVariables();
    DRAKE_DEMAND(input_variables.size() == 1);
    for (const auto& var : input_variables) {
      if (output_variables.include(var)) {
        return true;
      }
    }
  }

  return false;
}

}  // namespace systems
}  // namespace drake
