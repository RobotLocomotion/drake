#include "drake/solvers/solve.h"

#include <memory>

#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/mathematical_program_solver_interface.h"

namespace drake {
namespace solvers {
MathematicalProgramResult Solve(const MathematicalProgram& prog,
                                const optional<Eigen::VectorXd>& initial_guess,
                                const optional<SolverOptions>& solver_options) {
  const SolverId solver_id = ChooseBestSolver(prog);
  std::unique_ptr<MathematicalProgramSolverInterface> solver =
      MakeSolver(solver_id);
  MathematicalProgramResult result{};
  solver->Solve(prog, initial_guess, solver_options, &result);
  return result;
}

MathematicalProgramResult Solve(
    const MathematicalProgram& prog,
    const Eigen::Ref<const Eigen::VectorXd>& initial_guess) {
  const Eigen::VectorXd initial_guess_xd = initial_guess;
  return Solve(prog, initial_guess_xd, {});
}

MathematicalProgramResult Solve(const MathematicalProgram& prog) {
  return Solve(prog, {}, {});
}
}  // namespace solvers
}  // namespace drake
