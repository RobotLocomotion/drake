#pragma once

#include <memory>
#include <set>
#include <vector>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solver_id.h"
#include "drake/solvers/solver_interface.h"

namespace drake {
namespace solvers {

/**
 * Choose the best solver given the formulation in the optimization program and
 * the availability of the solvers.
 * @throws std::exception if there is no available solver for @p prog.
 */
SolverId ChooseBestSolver(const MathematicalProgram& prog);

/**
 * Returns the set of solvers known to ChooseBestSolver.
 */
const std::set<SolverId>& GetKnownSolvers();

/**
 * Given the solver ID, create the solver with the matching ID.
 * @throws std::exception if there is no matching solver.
 */
std::unique_ptr<SolverInterface> MakeSolver(const SolverId& id);

/**
 * Makes the first available and enabled solver. If no solvers are available,
 * throws a std::exception.
 */
std::unique_ptr<SolverInterface> MakeFirstAvailableSolver(
    const std::vector<SolverId>& solver_ids);

}  // namespace solvers
}  // namespace drake
