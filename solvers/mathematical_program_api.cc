/* clang-format off to disable clang-format-includes */
#include "drake/solvers/mathematical_program.h"
/* clang-format on */

#include <sstream>
#include <stdexcept>

// This file contains the portions of mathematical_program.h's implementation
// that are called by MathematicalProgramSolverInterface implementations.

namespace drake {
namespace solvers {

using std::ostringstream;
using std::runtime_error;

using symbolic::Variable;

int MathematicalProgram::FindDecisionVariableIndex(const Variable& var) const {
  auto it = decision_variable_index_.find(var.get_id());
  if (it == decision_variable_index_.end()) {
    ostringstream oss;
    oss << var
        << " is not a decision variable in the mathematical program, "
           "when calling FindDecisionVariableIndex.\n";
    throw runtime_error(oss.str());
  }
  return it->second;
}

std::vector<int> MathematicalProgram::FindDecisionVariableIndices(
    const Eigen::Ref<const VectorXDecisionVariable>& vars) const {
  std::vector<int> x_indices(vars.rows());
  for (int i = 0; i < vars.rows(); ++i) {
    x_indices[i] = FindDecisionVariableIndex(vars(i));
  }
  return x_indices;
}

size_t MathematicalProgram::FindIndeterminateIndex(const Variable& var) const {
  auto it = indeterminates_index_.find(var.get_id());
  if (it == indeterminates_index_.end()) {
    ostringstream oss;
    oss << var
        << " is not an indeterminate in the mathematical program, "
           "when calling GetSolution.\n";
    throw runtime_error(oss.str());
  }
  return it->second;
}

void MathematicalProgram::SetSolverResult(const SolverResult& solver_result) {
  this->solver_id_ = solver_result.solver_id();
  if (solver_result.decision_variable_values().has_value()) {
    DRAKE_DEMAND(solver_result.decision_variable_values().value().rows() ==
                 num_vars());
    x_values_ = solver_result.decision_variable_values().value();
  }
  if (solver_result.optimal_cost().has_value()) {
    optimal_cost_ = solver_result.optimal_cost().value();
  }
  if (solver_result.optimal_cost_lower_bound().has_value()) {
    lower_bound_cost_ = solver_result.optimal_cost_lower_bound().value();
  }
}

}  // namespace solvers
}  // namespace drake
