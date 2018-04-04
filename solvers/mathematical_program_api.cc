/* clang-format off to disable clang-format-includes */
#include "drake/solvers/mathematical_program.h"
/* clang-format on */

#include <sstream>
#include <stdexcept>

// This file contains the portions of mathematical_program.h's implementation
// that are referred to by MathematicalProgramSolverInterface implementations.
//
// The MathematicalProgram destructor is listed here because the typeinfo for
// MathematicalProgram might be needed by ...SolverInterface implementations,
// not because they would usually destroy MathematicalProgram objects.

namespace drake {
namespace solvers {

using std::ostringstream;
using std::runtime_error;

using symbolic::Variable;

MathematicalProgram::~MathematicalProgram() = default;

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
  if (solver_result.decision_variable_values()) {
    DRAKE_DEMAND(solver_result.decision_variable_values()->rows() ==
                 num_vars());
    x_values_ = *(solver_result.decision_variable_values());
  } else {
    x_values_ = Eigen::VectorXd::Constant(
        num_vars(), std::numeric_limits<double>::quiet_NaN());
  }
  if (solver_result.optimal_cost()) {
    optimal_cost_ = *(solver_result.optimal_cost());
  } else {
    optimal_cost_ = std::numeric_limits<double>::quiet_NaN();
  }
  if (solver_result.optimal_cost_lower_bound()) {
    lower_bound_cost_ = *(solver_result.optimal_cost_lower_bound());
  } else {
    lower_bound_cost_ = optimal_cost_;
  }
}

}  // namespace solvers
}  // namespace drake
