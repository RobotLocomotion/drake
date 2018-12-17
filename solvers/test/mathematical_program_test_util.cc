#include "drake/solvers/test/mathematical_program_test_util.h"

#include <stdexcept>

namespace drake {
namespace solvers {
namespace test {
void CheckSolver(const MathematicalProgram& prog, SolverId desired_solver_id) {
  const optional<SolverId> solver_id = prog.GetSolverId();
  EXPECT_TRUE(solver_id);
  if (!solver_id) { return; }

  EXPECT_EQ(*solver_id, desired_solver_id);
}

MathematicalProgramResult RunSolver(
    const MathematicalProgram& prog,
    const MathematicalProgramSolverInterface& solver,
    const optional<Eigen::VectorXd>& initial_guess) {
  if (!solver.available()) {
    throw std::runtime_error(
        "Solver " + solver.solver_id().name() + " is not available");
  }

  MathematicalProgramResult result;
  solver.Solve(prog, initial_guess, {}, &result);
  EXPECT_EQ(result.get_solution_result(), SolutionResult::kSolutionFound);
  if (result.get_solution_result() != SolutionResult::kSolutionFound) {
    throw std::runtime_error(
        "Solver " + solver.solver_id().name() + " fails to find the solution");
  }
  return result;
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
