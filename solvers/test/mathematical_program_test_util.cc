#include "drake/solvers/test/mathematical_program_test_util.h"

#include <stdexcept>

namespace drake {
namespace solvers {
namespace test {
MathematicalProgramResult RunSolver(
    const MathematicalProgram& prog,
    const SolverInterface& solver,
    const std::optional<Eigen::VectorXd>& initial_guess) {
  if (!solver.available()) {
    throw std::runtime_error(
        "Solver " + solver.solver_id().name() + " is not available");
  }

  MathematicalProgramResult result{};
  solver.Solve(prog, initial_guess, {}, &result);
  EXPECT_TRUE(result.is_success());
  if (!result.is_success()) {
    throw std::runtime_error(
        "Solver " + solver.solver_id().name() + " fails to find the solution");
  }
  return result;
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
