#include "drake/systems/framework/system_symbolic_inspector.h"

#include <sstream>

#include "drake/common/drake_assert.h"
#include "drake/common/polynomial.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace systems {

using symbolic::Expression;
using symbolic::Formula;

SystemSymbolicInspector::SystemSymbolicInspector(
    const System<symbolic::Expression>& system)
    : context_(system.CreateDefaultContext()),
      input_variables_(system.num_input_ports()),
      continuous_state_variables_(context_->num_continuous_states()),
      discrete_state_variables_(context_->num_discrete_state_groups()),
      numeric_parameters_(context_->num_numeric_parameter_groups()),
      output_(system.AllocateOutput()),
      derivatives_(system.AllocateTimeDerivatives()),
      discrete_updates_(system.AllocateDiscreteVariables()),
      output_port_types_(system.num_output_ports()),
      context_is_abstract_(IsAbstract(system, *context_)) {
  // Stop analysis if the Context is in any way abstract, because we have no way
  // to initialize the abstract elements.
  if (context_is_abstract_) return;

  // Time.
  time_ = symbolic::Variable("t");
  context_->SetTime(time_);

  // Input.
  InitializeVectorInputs(system);

  // State.
  InitializeContinuousState();
  InitializeDiscreteState();

  // Parameters.
  InitializeParameters();

  // Outputs.
  for (int i = 0; i < system.num_output_ports(); ++i) {
    const OutputPort<symbolic::Expression>& port = system.get_output_port(i);
    output_port_types_[i] = port.get_data_type();
    port.Calc(*context_, output_->GetMutableData(i));
  }

  // Time derivatives.
  if (context_->num_continuous_states() > 0) {
    system.CalcTimeDerivatives(*context_, derivatives_.get());
  }

  // Discrete updates.
  if (context_->num_discrete_state_groups() > 0) {
    system.CalcDiscreteVariableUpdates(*context_, discrete_updates_.get());
  }

  // Constraints.
  // TODO(russt): Maintain constraint descriptions.
  for (int i = 0; i < system.num_constraints(); i++) {
    const SystemConstraint<Expression>& constraint =
        system.get_constraint(SystemConstraintIndex(i));
    const double tol = 0.0;
    constraints_.emplace(constraint.CheckSatisfied(*context_, tol));
  }
}

void SystemSymbolicInspector::InitializeVectorInputs(
    const System<symbolic::Expression>& system) {
  // For each input vector i, set each element j to a symbolic expression whose
  // value is the variable "ui_j".
  for (int i = 0; i < system.num_input_ports(); ++i) {
    DRAKE_ASSERT(system.get_input_port(i).get_data_type() == kVectorValued);
    const int n = system.get_input_port(i).size();
    input_variables_[i].resize(n);
    auto value = system.AllocateInputVector(system.get_input_port(i));
    for (int j = 0; j < n; ++j) {
      std::ostringstream name;
      name << "u" << i << "_" << j;
      input_variables_[i][j] = symbolic::Variable(name.str());
      value->SetAtIndex(j, input_variables_[i][j]);
    }
    system.get_input_port(i).FixValue(context_.get(), *value);
  }
}

void SystemSymbolicInspector::InitializeContinuousState() {
  // Set each element i in the continuous state to a symbolic expression whose
  // value is the variable "xci".
  VectorBase<symbolic::Expression>& xc =
      context_->get_mutable_continuous_state_vector();
  for (int i = 0; i < xc.size(); ++i) {
    std::ostringstream name;
    name << "xc" << i;
    continuous_state_variables_[i] = symbolic::Variable(name.str());
    xc[i] = continuous_state_variables_[i];
  }
}

void SystemSymbolicInspector::InitializeDiscreteState() {
  // For each discrete state vector i, set each element j to a symbolic
  // expression whose value is the variable "xdi_j".
  auto& xd = context_->get_mutable_discrete_state();
  for (int i = 0; i < context_->num_discrete_state_groups(); ++i) {
    auto& xdi = xd.get_mutable_vector(i);
    discrete_state_variables_[i].resize(xdi.size());
    for (int j = 0; j < xdi.size(); ++j) {
      std::ostringstream name;
      name << "xd" << i << "_" << j;
      discrete_state_variables_[i][j] = symbolic::Variable(name.str());
      xdi[j] = discrete_state_variables_[i][j];
    }
  }
}

void SystemSymbolicInspector::InitializeParameters() {
  // For each numeric parameter vector i, set each element j to a symbolic
  // expression whose value is the variable "pi_j".
  for (int i = 0; i < context_->num_numeric_parameter_groups(); ++i) {
    auto& pi = context_->get_mutable_numeric_parameter(i);
    numeric_parameters_[i].resize(pi.size());
    for (int j = 0; j < pi.size(); ++j) {
      std::ostringstream name;
      name << "p" << i << "_" << j;
      numeric_parameters_[i][j] = symbolic::Variable(name.str());
      pi[j] = numeric_parameters_[i][j];
    }
  }
}

bool SystemSymbolicInspector::IsAbstract(
    const System<symbolic::Expression>& system,
    const Context<symbolic::Expression>& context) {
  // If any of the input ports are abstract, we cannot do sparsity analysis of
  // this Context.
  for (int i = 0; i < system.num_input_ports(); ++i) {
    if (system.get_input_port(i).get_data_type() == kAbstractValued) {
      return true;
    }
  }

  // If there is any abstract state or parameters, we cannot do sparsity
  // analysis of this Context.
  if (context.num_abstract_states() > 0) {
    return true;
  }
  if (context.num_abstract_parameters() > 0) {
    return true;
  }

  return false;
}

bool SystemSymbolicInspector::IsConnectedInputToOutput(
    int input_port_index, int output_port_index) const {
  DRAKE_ASSERT(input_port_index >= 0 &&
               input_port_index < static_cast<int>(input_variables_.size()));
  DRAKE_ASSERT(output_port_index >= 0 &&
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

  // Check whether any of the variables in any of the input_variables_ are
  // among the output_variables.
  for (int j = 0; j < input_variables_[input_port_index].size(); ++j) {
    if (output_variables.include((input_variables_[input_port_index])(j))) {
      return true;
    }
  }

  return false;
}

namespace {

// helper method for IsTimeInvariant.
bool is_time_invariant(const VectorX<symbolic::Expression>& expressions,
                       const symbolic::Variable& t) {
  for (int i = 0; i < expressions.size(); ++i) {
    const Expression& e{expressions(i)};
    if (e.GetVariables().include(t)) {
      return false;
    }
  }
  return true;
}

}  // namespace

bool SystemSymbolicInspector::IsTimeInvariant() const {
  // Do not trust the parsing, so return the conservative answer.
  if (context_is_abstract_) {
    return false;
  }

  if (!is_time_invariant(derivatives_->CopyToVector(), time_)) {
    return false;
  }
  for (int i = 0; i < discrete_updates_->num_groups(); ++i) {
    if (!is_time_invariant(discrete_updates_->get_vector(i).get_value(),
                           time_)) {
      return false;
    }
  }
  for (int i = 0; i < output_->num_ports(); ++i) {
    if (output_port_types_[i] == kAbstractValued) {
      // Then I can't be sure.  Return the conservative answer.
      return false;
    }
    if (!is_time_invariant(output_->get_vector_data(i)->get_value(), time_)) {
      return false;
    }
  }

  return true;
}

bool SystemSymbolicInspector::HasAffineDynamics() const {
  // If the Context contains any abstract values, then I can't trust my parsing.
  if (context_is_abstract_) {
    return false;
  }

  symbolic::Variables vars(continuous_state_variables_);
  for (const auto& v : discrete_state_variables_) {
    vars.insert(symbolic::Variables(v));
  }
  for (const auto& v : input_variables_) {
    vars.insert(symbolic::Variables(v));
  }

  if (!IsAffine(derivatives_->CopyToVector(), vars)) {
    return false;
  }
  for (int i = 0; i < discrete_updates_->num_groups(); ++i) {
    if (!IsAffine(discrete_updates_->get_vector(i).get_value(), vars)) {
      return false;
    }
  }
  return true;
}

}  // namespace systems
}  // namespace drake
