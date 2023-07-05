#include "drake/solvers/gurobi_solver.h"

using drake::solvers::GurobiSolver;

/* Acquire a license and report the overall use_count. */
int main() {
  std::shared_ptr<GurobiSolver::License> license =
      GurobiSolver::AcquireLicense();
  fmt::print("{}\n", license.use_count());
  return 0;
}
