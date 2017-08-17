/* clang-format off to disable clang-format-includes */
#include "drake/solvers/gurobi_solver.h"
/* clang-format on */

#include "drake/common/never_destroyed.h"

// This file contains implementations that are common to both the available and
// unavailable flavor of this class.

namespace drake {
namespace solvers {

SolverId GurobiSolver::solver_id() const {
  return id();
}

SolverId GurobiSolver::id() {
  static const never_destroyed<SolverId> singleton{"Gurobi"};
  return singleton.access();
}

}  // namespace solvers
}  // namespace drake
