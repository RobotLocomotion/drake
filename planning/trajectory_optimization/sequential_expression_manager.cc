#include "drake/planning/trajectory_optimization/sequential_expression_manager.h"

#include <fmt/format.h>

#include "drake/common/unused.h"

namespace drake {
namespace planning {
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
  name_to_placeholders_.emplace(std::make_pair(name, placeholders));
  for (int i = 0; i < placeholders.size(); ++i) {
    placeholders_to_expressions_.emplace(
        std::make_pair(placeholders[i], sequential_expressions.row(i)));
  }
}

Substitution
SequentialExpressionManager::ConstructPlaceholderVariableSubstitution(
    int index) const {
  DRAKE_THROW_UNLESS(0 <= index && index < num_samples_);
  Substitution substitution;
  for (const auto& [p, e] : placeholders_to_expressions_) {
    substitution.emplace(p, e(index));
  }
  return substitution;
}

VectorX<symbolic::Variable> SequentialExpressionManager::GetVariables(
    const Eigen::Ref<const VectorX<symbolic::Variable>>& vars,
    int index) const {
  DRAKE_THROW_UNLESS(0 <= index && index < num_samples_);
  VectorX<symbolic::Variable> vars_at_index(vars.size());
  for (int i = 0; i < vars.size(); ++i) {
    const auto it = placeholders_to_expressions_.find(vars[i]);
    if (it == placeholders_to_expressions_.end()) {
      throw std::runtime_error(
          vars[i].get_name() +
          " does not appear to be a placeholder variable in this program.");
    }
    if (!symbolic::is_variable(it->second[index])) {
      throw std::runtime_error(
          fmt::format("The placeholder variable {} is associated with {} which "
                      "is not a variable.",
                      vars[i].get_name(), it->second[index].to_string()));
    }
    vars_at_index[i] = symbolic::get_variable(it->second[index]);
  }
  return vars_at_index;
}

VectorX<Expression> SequentialExpressionManager::GetSequentialExpressionsByName(
    const string& name, int index) const {
  DRAKE_THROW_UNLESS(0 <= index && index < num_samples_);
  const auto it = name_to_placeholders_.find(name);
  DRAKE_THROW_UNLESS(it != name_to_placeholders_.end());
  VectorX<Expression> e(it->second.size());
  for (int i = 0; i < it->second.size(); ++i) {
    const auto e_it = placeholders_to_expressions_.find(it->second[i]);
    DRAKE_THROW_UNLESS(e_it != placeholders_to_expressions_.end());
    e[i] = e_it->second[index];
  }
  return e;
}

int SequentialExpressionManager::num_rows(const string& name) const {
  const auto it = name_to_placeholders_.find(name);
  DRAKE_THROW_UNLESS(it != name_to_placeholders_.end());
  return it->second.size();
}

vector<string> SequentialExpressionManager::GetSequentialExpressionNames()
    const {
  vector<string> ret;
  for (const auto& [p, e] : placeholders_to_expressions_) {
    unused(p);
    for (int i = 0; i < static_cast<int>(e.size()); ++i) {
      ret.emplace_back(e(i).to_string());
    }
  }
  return ret;
}

}  // namespace internal
}  // namespace trajectory_optimization
}  // namespace planning
}  // namespace drake
