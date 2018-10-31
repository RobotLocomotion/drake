#pragma once

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solver_id.h"

namespace drake {
namespace solvers {
/**
 * Choose the best solver given the formulation in the optimization program and
 * the availability of the solvers.
 * @throw invalid_argument if there is no available solver for @p prog.
 */
SolverId ChooseBestSolver(const MathematicalProgram& prog);
}  // namespace solvers
}  // namespace drake
