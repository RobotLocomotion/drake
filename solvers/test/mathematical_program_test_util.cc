#include "drake/solvers/test/mathematical_program_test_util.h"

#include <stdexcept>

namespace drake {
namespace solvers {
namespace test {
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
