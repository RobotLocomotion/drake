#include "drake/solvers/solve.h"

#include <memory>

#include "drake/common/nice_type_name.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/solver_interface.h"

namespace drake {
namespace solvers {
MathematicalProgramResult Solve(
    const MathematicalProgram& prog,
    const std::optional<Eigen::VectorXd>& initial_guess,
    const std::optional<SolverOptions>& solver_options) {
  const SolverId solver_id = ChooseBestSolver(prog);
  std::unique_ptr<SolverInterface> solver = MakeSolver(solver_id);
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

MathematicalProgramResult SolveWithFirstAvailableSolver(
    const MathematicalProgram& prog,
    const std::vector<const SolverInterface*>& solvers,
    const std::optional<Eigen::VectorXd>& initial_guess,
    const std::optional<SolverOptions>& solver_options) {
  MathematicalProgramResult result;
  for (const auto solver : solvers) {
    if (solver->available() && solver->enabled()) {
      solver->Solve(prog, initial_guess, solver_options, &result);
      return result;
    }
  }
  std::string solver_names = "";
  for (const auto solver : solvers) {
    solver_names.append(solver->solver_id().name() + " ");
  }
  throw std::runtime_error(
      "SolveWithFirstAvailableSolver(): none of the solvers " + solver_names +
      "is available or enabled.");
}
}  // namespace solvers
}  // namespace drake
