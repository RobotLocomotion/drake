#include "drake/solvers/test/mathematical_program_test_util.h"

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
  if (solver.available()) {
    SolutionResult result = solver.Solve(*prog);
    SolverType solver_type;
    int solver_status;
    prog->GetSolverResult(&solver_type, &solver_status);
    EXPECT_EQ(result, SolutionResult::kSolutionFound)
        << "Solver " << solver.SolverName() << " fails to find the solution."
        << std::endl;
  }
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
