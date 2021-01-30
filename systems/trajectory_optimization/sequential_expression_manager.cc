#include "drake/systems/trajectory_optimization/sequential_expression_manager.h"

#include <fmt/format.h>

namespace drake {
namespace systems {
namespace trajectory_optimization {
namespace internal {

using std::string;
using std::vector;

using symbolic::Expression;
using symbolic::Substitution;
using symbolic::Variable;
using symbolic::Variables;

SequentialExpressionManager::SequentialExpressionManager(int num_samples)
    : num_samples_(num_samples) {
  DRAKE_THROW_UNLESS(num_samples_ > 0);
}

VectorX<Variable> SequentialExpressionManager::RegisterSequentialExpressions(
    const Eigen::Ref<const MatrixX<Expression>>& sequential_expressions,
    const string& name) {
  const int rows = sequential_expressions.rows();
  DRAKE_THROW_UNLESS(sequential_expressions.cols() == num_samples_);
  const VectorX<Variable> placeholders{
      symbolic::MakeVectorContinuousVariable(rows, name)};
  RegisterSequentialExpressions(placeholders, sequential_expressions, name);
  return placeholders;
}

void SequentialExpressionManager::RegisterSequentialExpressions(
    const VectorX<Variable>& placeholders,
    const Eigen::Ref<const MatrixX<Expression>>& sequential_expressions,
    const string& name) {
  DRAKE_THROW_UNLESS(sequential_expressions.rows() == placeholders.size());
  DRAKE_THROW_UNLESS(sequential_expressions.cols() == num_samples_);
  const auto pair = name_to_placeholders_and_sequential_expressions_.insert(
      {name, {placeholders, sequential_expressions}});
  DRAKE_THROW_UNLESS(pair.second);
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

VectorX<Expression> SequentialExpressionManager::GetSequentialExpressionsByName(
    const string& name, int index) const {
  DRAKE_THROW_UNLESS(0 <= index && index < num_samples_);
  const auto it = name_to_placeholders_and_sequential_expressions_.find(name);
  DRAKE_THROW_UNLESS(it !=
                     name_to_placeholders_and_sequential_expressions_.end());
  const MatrixX<Expression>& sequential_expressions = it->second.second;
  return sequential_expressions.col(index);
}

int SequentialExpressionManager::num_rows(const string& name) const {
  const auto it = name_to_placeholders_and_sequential_expressions_.find(name);
  DRAKE_THROW_UNLESS(it !=
                     name_to_placeholders_and_sequential_expressions_.end());
  return it->second.first.size();
}

vector<string> SequentialExpressionManager::GetSequentialExpressionNames()
    const {
  vector<string> ret;
  for (const auto& item : name_to_placeholders_and_sequential_expressions_) {
    const MatrixX<symbolic::Expression>& sequential_expressions{
        item.second.second};
    for (int i = 0; i < static_cast<int>(sequential_expressions.rows()); ++i) {
      for (int j = 0; j < static_cast<int>(sequential_expressions.cols());
           ++j) {
        ret.emplace_back(sequential_expressions(i, j).to_string());
      }
    }
  }
  return ret;
}

}  // namespace internal
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
