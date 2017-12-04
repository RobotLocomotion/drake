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

void RunSolver(MathematicalProgram* prog,
               const MathematicalProgramSolverInterface& solver) {
  if (!solver.available()) {
    throw std::runtime_error(
        "Solver " + solver.solver_id().name() + " is not available");
  }

  SolutionResult result = solver.Solve(*prog);
  EXPECT_EQ(result, SolutionResult::kSolutionFound);
  if (result != SolutionResult::kSolutionFound) {
    throw std::runtime_error(
        "Solver " + solver.solver_id().name() + " fails to find the solution");
  }
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
