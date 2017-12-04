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
           "when calling GetSolution.\n";
    throw runtime_error(oss.str());
  }
  return it->second;
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

void MathematicalProgram::SetDecisionVariableValues(
    const Eigen::Ref<const Eigen::VectorXd>& values) {
  SetDecisionVariableValues(decision_variables_, values);
}

void MathematicalProgram::SetDecisionVariableValues(
    const Eigen::Ref<const VectorXDecisionVariable>& variables,
    const Eigen::Ref<const Eigen::VectorXd>& values) {
  DRAKE_ASSERT(values.rows() == variables.rows());
  for (int i = 0; i < values.rows(); ++i) {
    x_values_[FindDecisionVariableIndex(variables(i))] = values(i);
  }
}

void MathematicalProgram::SetDecisionVariableValue(const Variable& var,
                                                   double value) {
  x_values_[FindDecisionVariableIndex(var)] = value;
}

}  // namespace solvers
}  // namespace drake
