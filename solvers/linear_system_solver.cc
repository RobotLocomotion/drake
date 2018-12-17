#include "drake/solvers/linear_system_solver.h"

#include <cstring>
#include <limits>
#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/never_destroyed.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

bool LinearSystemSolver::is_available() { return true; }

void LinearSystemSolver::Solve(const MathematicalProgram& prog,
                               const optional<Eigen::VectorXd>& initial_guess,
                               const optional<SolverOptions>& solver_options,
                               MathematicalProgramResult* result) const {
  *result = {};
  // The initial guess doesn't help us, and we don't offer any tuning options.
  unused(initial_guess, solver_options);
  size_t num_constraints = 0;
  for (auto const& binding : prog.linear_equality_constraints()) {
    num_constraints += binding.evaluator()->A().rows();
  }

  DRAKE_ASSERT(prog.generic_constraints().empty());
  DRAKE_ASSERT(prog.generic_costs().empty());
  DRAKE_ASSERT(prog.quadratic_costs().empty());
  DRAKE_ASSERT(prog.linear_constraints().empty());
  DRAKE_ASSERT(prog.bounding_box_constraints().empty());
  DRAKE_ASSERT(prog.linear_complementarity_constraints().empty());

  Eigen::MatrixXd Aeq = Eigen::MatrixXd::Zero(num_constraints, prog.num_vars());
  // TODO(naveenoid) : use a sparse matrix here?
  Eigen::VectorXd beq(num_constraints);

  size_t constraint_index = 0;
  for (auto const& binding : prog.linear_equality_constraints()) {
    auto const& c = binding.evaluator();
    size_t n = c->A().rows();
    for (int i = 0; i < static_cast<int>(binding.GetNumElements()); ++i) {
      size_t variable_index =
          prog.FindDecisionVariableIndex(binding.variables()(i));
      Aeq.block(constraint_index, variable_index, n, 1) = c->A().col(i);
    }
    beq.segment(constraint_index, n) =
        c->lower_bound();  // = c->upper_bound() since it's an equality
    // constraint
    constraint_index += n;
  }

  // least-squares solution
  const Eigen::VectorXd least_square_sol =
      Aeq.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(beq);

  result->set_solver_id(id());
  result->set_x_val(least_square_sol);
  if (beq.isApprox(Aeq * least_square_sol)) {
    result->set_optimal_cost(0.);
    result->set_solution_result(SolutionResult::kSolutionFound);
  } else {
    result->set_optimal_cost(MathematicalProgram::kGlobalInfeasibleCost);
    result->set_solution_result(SolutionResult::kInfeasibleConstraints);
  }
}

SolutionResult LinearSystemSolver::Solve(MathematicalProgram& prog) const {
  MathematicalProgramResult result;
  Solve(prog, {}, {}, &result);
  const SolverResult solver_result = result.ConvertToSolverResult();
  prog.SetSolverResult(solver_result);
  return result.get_solution_result();
}

SolverId LinearSystemSolver::solver_id() const { return id(); }

SolverId LinearSystemSolver::id() {
  static const never_destroyed<SolverId> singleton{"Linear system"};
  return singleton.access();
}

bool LinearSystemSolver::AreProgramAttributesSatisfied(
    const MathematicalProgram& prog) const {
  return ProgramAttributesSatisfied(prog);
}

bool LinearSystemSolver::ProgramAttributesSatisfied(
    const MathematicalProgram& prog) {
  static const never_destroyed<ProgramAttributes> solver_capability(
      std::initializer_list<ProgramAttribute>{
          ProgramAttribute::kLinearEqualityConstraint});
  return prog.required_capabilities() == solver_capability.access();
}

}  // namespace solvers
}  // namespace drake
