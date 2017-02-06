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
  // Abort early if the Context is in any way abstract, because we have no way
  // to initialize the abstract elements.
  if (context_is_abstract_) return;

  // Time
  context_->set_time(symbolic::Expression("t"));

  // Input
  InitializeVectorInputs(system);

  // State
  InitializeContinuousState();
  InitializeDiscreteState();

  // Parameters
  // TODO(david-german-tri): Initialize parameters, once #5072 is resolved.

  // Outputs
  // -- Record the output port descriptors.
  for (int i = 0; i < system.get_num_output_ports(); ++i) {
    output_port_types_[i] = system.get_output_port(i).get_data_type();
  }

  // -- Compute the Outputs.
  system.CalcOutput(*context_, output_.get());
  // TODO(david-german-tri): Other System computations, such as derivatives.
}

SparsityMatrix::~SparsityMatrix() {}

void SparsityMatrix::InitializeVectorInputs(
    const System<symbolic::Expression>& system) {
  // For each input vector i, set each element j to a symbolic expression whose
  // value is the variable "ui_j".
  for (int i = 0; i < system.get_num_input_ports(); ++i) {
    if (system.get_input_port(i).get_data_type() == kVectorValued) {
      const int n = system.get_input_port(i).size();
      auto value = std::make_unique<BasicVector<symbolic::Expression>>(n);
      for (int j = 0; j < n; ++j) {
        std::ostringstream name;
        name << "u" << i << "_" << j;
        value->SetAtIndex(j, symbolic::Expression(name.str()));
        input_expressions_[i].push_back(value->GetAtIndex(j));
      }
      context_->FixInputPort(i, std::move(value));
    }
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
    xc[i] = symbolic::Expression(name.str());
  }
}

void SparsityMatrix::InitializeDiscreteState() {
  // For each discrete state vector i, set each element j to a symbolic
  // expression whose value is the variable "xdi_j".
  auto& xd = *context_->get_mutable_discrete_state();
  for (int i = 0; i < context_->get_num_discrete_state_groups(); ++i) {
    auto &xdi = *xd.get_mutable_discrete_state(i);
    for (int j = 0; j < xdi.size(); ++j) {
      std::ostringstream name;
      name << "xd" << i << "_" << j;
      xdi[j] = symbolic::Expression(name.str());
    }
  }
}

bool SparsityMatrix::IsAbstract(const System<symbolic::Expression>& system,
                                const Context<symbolic::Expression>& context) {
  // If any of the input ports are abstract, the context is abstract.
  for (int i = 0; i < system.get_num_input_ports(); ++i) {
    if (system.get_input_port(i).get_data_type() == kAbstractValued) {
      return true;
    }
  }

  // If there is any abstract state, the context is abstract.
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
  for (int i = 0; i < output_exprs->size(); ++i) {
    symbolic::Variables vars = output_exprs->GetAtIndex(i).GetVariables();
    output_variables.insert(vars.begin(), vars.end());
  }

  // Check whether any of the variables in any of the input_expressions_ are
  // among the output_variables.
  for (const auto& expr : input_expressions_[input_port_index]) {
    symbolic::Variables input_variables = expr.GetVariables();
    DRAKE_DEMAND(input_variables.size() == 1);
    for (const auto& var : input_variables) {
      if (output_variables.find(var) != output_variables.end()) {
        return true;
      }
    }
  }

  return false;
}

}  // namespace systems
}  // namespace drake
