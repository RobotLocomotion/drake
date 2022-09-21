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
[[nodiscard]] SolverId ChooseBestSolver(const MathematicalProgram& prog);

/**
 * Returns the set of solvers known to ChooseBestSolver.
 */
[[nodiscard]] const std::set<SolverId>& GetKnownSolvers();

/**
 * Given the solver ID, create the solver with the matching ID.
 * @throws std::exception if there is no matching solver.
 */
[[nodiscard]] std::unique_ptr<SolverInterface> MakeSolver(const SolverId& id);

/**
 * Makes the first available and enabled solver. If no solvers are available,
 * throws a std::exception.
 */
[[nodiscard]] std::unique_ptr<SolverInterface> MakeFirstAvailableSolver(
    const std::vector<SolverId>& solver_ids);

/**
 * Returns the list of available and enabled solvers that definitely accept all
 * programs of the given program type.
 * The order of the returned SolverIds reflects an approximate order of
 * preference, from most preferred (front) to least preferred (back). Because we
 * are analyzing only based on the program type rather than a specific program,
 * it's possible that solvers later in the list would perform better in certain
 * situations. To obtain the truly best solver, using ChooseBestSolver()
 * instead.
 * @note If a solver only accepts a subset of the program type, then that solver
 * is not included in the returned results. For example
 * EqualityConstrainedQPSolver doesn't accept programs with inequality linear
 * constraints, so it doesn't show up in the return of
 * GetAvailableSolvers(ProgramType::kQP).
 */
[[nodiscard]] std::vector<SolverId> GetAvailableSolvers(ProgramType prog_type);

}  // namespace solvers
}  // namespace drake
