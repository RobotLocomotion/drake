#include "drake/systems/trajectory_optimization/sequential_expression_manager.h"

#include <fmt/format.h>

namespace drake {
namespace systems {
namespace trajectory_optimization {
namespace internal {

using symbolic::Substitution;
using symbolic::Expression;
using symbolic::Variable;
using symbolic::Variables;

SequentialExpressionManager::SequentialExpressionManager(int num_samples)
    : num_samples_(num_samples) {
  DRAKE_THROW_UNLESS(num_samples_ > 0);
}

VectorX<Variable> SequentialExpressionManager::RegisterSequentialExpressions(
    const Eigen::Ref<const MatrixX<Expression>>& sequential_expressions,
    const std::string& name) {
  const int rows = sequential_expressions.rows();
  DRAKE_THROW_UNLESS(sequential_expressions.cols() == num_samples_);
  VectorX<Variable> placeholders{rows};
  for (int i = 0; i < rows; ++i) {
    placeholders(i) = Variable(fmt::format("{}{}", name, i));
  }
  const auto pair = name_to_placeholders_and_sequential_expressions_.insert(
      {name, {placeholders, sequential_expressions}});
  DRAKE_THROW_UNLESS(pair.second);
  return placeholders;
}

Substitution
SequentialExpressionManager::ConstructPlaceholderVariableSubstitution(
    int index) const {
  DRAKE_THROW_UNLESS(0 <= index && index < num_samples_);
  Substitution substitution;
  for (const auto& pair : name_to_placeholders_and_sequential_expressions_) {
    // pair.first is the name, which we don't need here.
    const VectorX<Variable>& placeholders = pair.second.first;
    const MatrixX<Expression>& sequential_expressions = pair.second.second;
    const int rows = placeholders.rows();
    for (int row = 0; row < rows; ++row) {
      substitution.emplace(placeholders(row),
                           sequential_expressions(row, index));
    }
  }
  return substitution;
}

VectorX<Expression>
SequentialExpressionManager::GetSequentialExpressionsByName(
    const std::string& name, int index) const {
  DRAKE_THROW_UNLESS(0 <= index && index < num_samples_);
  const auto it = name_to_placeholders_and_sequential_expressions_.find(name);
  DRAKE_THROW_UNLESS(it !=
                     name_to_placeholders_and_sequential_expressions_.end());
  const MatrixX<Expression>& sequential_expressions = it->second.second;
  return sequential_expressions.col(index);
}

}  // namespace internal
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
