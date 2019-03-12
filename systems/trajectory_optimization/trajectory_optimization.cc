#include "drake/systems/trajectory_optimization/trajectory_optimization.h"

#include <fmt/format.h>

#include "drake/common/text_logging.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {

using symbolic::Variable;

VectorX<Variable> TrajectoryOptimization::NewSequentialVariables(
    int rows, const std::string& name) {
  VectorX<Variable> placeholders{rows};
  MatrixX<Variable> sequential_variables =
      NewContinuousVariables(rows, SamplesPerSequentialVariable(),
                             name);
  for (int i = 0; i < rows; ++i) {
    placeholders(i) = Variable(fmt::format("{}{}", name, i));
    placeholder_to_sequential_variables_.insert(
        {placeholders(i), sequential_variables.row(i).eval()});
  }
  return placeholders;
}

Variable TrajectoryOptimization::GetSequentialVariableAtIndex(
    const Variable& placeholder, int index) {
  const auto it = placeholder_to_sequential_variables_.find(placeholder);
  DRAKE_DEMAND(it != placeholder_to_sequential_variables_.end());
  return it->second(index);
}

VectorX<Variable> TrajectoryOptimization::GetSequentialVariablesAtIndex(
    const VectorX<Variable>& placeholders, int index) {
  const int num_variables = placeholders.size();
  VectorX<Variable> sequential_variables(num_variables);
  for (int i = 0; i < num_variables; ++i) {
    sequential_variables(i) =
        GetSequentialVariableAtIndex(placeholders(i), index);
  }
  return sequential_variables;
}

void TrajectoryOptimization::AddPlaceholderVariableSubstitutionsForIndex(
    int index, symbolic::Substitution* substitution) const {
  DRAKE_ASSERT(substitution);
  for (const auto& pair : placeholder_to_sequential_variables_) {
    substitution->emplace(pair.first, pair.second(index));
  }
}

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
