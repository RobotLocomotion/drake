#pragma once

#include <memory>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solver_id.h"
#include "drake/solvers/solver_interface.h"

namespace drake {
namespace solvers {
/**
 * Choose the best solver given the formulation in the optimization program and
 * the availability of the solvers.
 * @throw invalid_argument if there is no available solver for @p prog.
 */
SolverId ChooseBestSolver(const MathematicalProgram& prog);

/**
 * Given the solver ID, create the solver with the matching ID.
 * @throw invalid_argument if there is no matching solver.
 */
std::unique_ptr<SolverInterface> MakeSolver(const SolverId& id);
}  // namespace solvers
}  // namespace drake
