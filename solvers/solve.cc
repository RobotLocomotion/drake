#include "drake/solvers/solve.h"

#include "drake/solvers/choose_best_solver.h"

namespace drake {
namespace solvers {
MathematicalProgramResult Solve(const MathematicalProgram& prog,
                                const optional<Eigen::VectorXd>& initial_guess,
                                const optional<SolverOptions>& solver_options) {
  const SolverId solver_id = ChooseBestSolver(prog);
  std::unique_ptr<MathematicalProgramSolverInterface> solver =
      MakeSolver(solver_id);
  return solver->Solve(prog, initial_guess, solver_options);
}
}  // namespace solvers
}  // namespace drake
