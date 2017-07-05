#include "drake/solvers/test/mathematical_program_test_util.h"

#include <stdexcept>

namespace drake {
namespace solvers {
namespace test {
void CheckSolver(const MathematicalProgram& prog,
                 SolverType desired_solver_type) {
  SolverType solver_type;
  int solver_result;
  prog.GetSolverResult(&solver_type, &solver_result);
  EXPECT_EQ(solver_type, desired_solver_type);
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
