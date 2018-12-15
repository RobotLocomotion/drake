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

void RunSolver(const MathematicalProgram& prog,
               const optional<Eigen::VectorXd>& initial_guess,
               const MathematicalProgramSolverInterface& solver,
               MathematicalProgramResult* result) {
  if (!solver.available()) {
    throw std::runtime_error(
        "Solver " + solver.solver_id().name() + " is not available");
  }

  solver.Solve(prog, initial_guess, {}, result);
  EXPECT_EQ(result->get_solution_result(), SolutionResult::kSolutionFound);
  if (result->get_solution_result() != SolutionResult::kSolutionFound) {
    throw std::runtime_error(
        "Solver " + solver.solver_id().name() + " fails to find the solution");
  }
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
