/* clang-format off to disable clang-format-includes */
#include "drake/solvers/mathematical_program.h"
/* clang-format on */

#include "drake/solvers/choose_best_solver.h"

namespace drake {
namespace solvers {

using symbolic::Variable;

// This method is placed in this file to break a dependency cycle.  In the
// MathematicalProgram, ideally we should only deal with problem formulation,
// and then in a SolverInterface implementation we use the MathematicalProgram
// as input to search for a solution.  However, the contract of this method *on
// MathematicalProgram* is to solve the program, which means it needs to depend
// on specific SolverInterface implementations, but those implementations need
// to depend on this program, so we have a cycle.  To break the cycle, we can
// compile this method in a separate file so that this method can depend on
// both MathematicalProgram and all of the SolverInterface implementations.
//
// Since this method is deprecated (#9633), this code-organization oddity won't
// have to last very long.
SolutionResult MathematicalProgram::Solve() {
  const SolverId solver_id = ChooseBestSolver(*this);
  // Note that this implementation isn't threadsafe (we might race on the
  // solver_cache_... value), but that's OK because the contract of Solve() is
  // already known to be racy and single-threaded only (it writes back the
  // solution results using setters on this MathematicalProgram instance).
  if (!solver_cache_for_deprecated_solve_method_ ||
      (solver_cache_for_deprecated_solve_method_->solver_id() != solver_id)) {
    solver_cache_for_deprecated_solve_method_ = MakeSolver(solver_id);
  }
  return solver_cache_for_deprecated_solve_method_->Solve(*this);
}

optional<SolverId> MathematicalProgram::GetSolverId() const {
  return solver_id_;
}

double MathematicalProgram::GetOptimalCost() const {
  return optimal_cost_;
}

double MathematicalProgram::GetLowerBoundCost() const {
  return lower_bound_cost_;
}

double MathematicalProgram::GetSolution(const Variable& var) const {
  return x_values_[FindDecisionVariableIndex(var)];
}

void MathematicalProgram::PrintSolution() {
  for (int i = 0; i < num_vars(); ++i) {
    std::cout << decision_variables_(i).get_name() << " = "
              << x_values_(i) << std::endl;
  }
}

symbolic::Expression MathematicalProgram::SubstituteSolution(
    const symbolic::Expression& e) const {
  symbolic::Environment::map map_decision_vars;
  for (const auto& var : e.GetVariables()) {
    const auto it = decision_variable_index_.find(var.get_id());
    if (it != decision_variable_index_.end()) {
      map_decision_vars.emplace(var, x_values_[it->second]);
    } else if (indeterminates_index_.find(var.get_id()) ==
               indeterminates_index_.end()) {
      // var is not a decision variable or an indeterminate in the optimization
      // program.
      std::ostringstream oss;
      oss << var << " is not a decision variable or an indeterminate of the "
                    "optimization program.\n";
      throw std::runtime_error(oss.str());
    }
  }
  return e.EvaluatePartial(symbolic::Environment(map_decision_vars));
}

symbolic::Polynomial MathematicalProgram::SubstituteSolution(
    const symbolic::Polynomial& p) const {
  symbolic::Environment::map map_decision_vars;
  for (const auto& var : p.decision_variables()) {
    map_decision_vars.emplace(var, x_values_[FindDecisionVariableIndex(var)]);
  }
  return p.EvaluatePartial(symbolic::Environment(map_decision_vars));
}

}  // namespace solvers
}  // namespace drake
