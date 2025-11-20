#pragma once

#include <optional>
#include <string>
#include <vector>

#include "drake/common/parallelism.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/solvers/solver_base.h"

namespace drake {
namespace solvers {
/**
 * Solves an optimization program, with optional initial guess and solver
 * options. This function first chooses the best solver depending on the
 * availability of the solver and the program formulation; it then constructs
 * that solver and call the Solve function of that solver. The optimization
 * result is stored in the return argument.
 * @param prog Contains the formulation of the program, and possibly solver
 * options.
 * @param initial_guess The initial guess for the decision variables. If an
 * @p initial_guess is provided, then the solver uses @p initial_guess and
 * ignores the initial guess stored in @p prog.
 * @param solver_options The options in addition to those stored in @p prog.
 * For each option entry (like print out), there are 4 ways to set that option,
 * and the priority given to the solver options is as follows (from lowest /
 * least, to highest / most):
 * 1. common option set on the MathematicalProgram itself
 * 2. common option passed as an argument to Solve
 * 3. solver-specific option set on the MathematicalProgram itself
 * 4. solver-specific option passed as an argument to Solve
 * @return result The result of solving the program through the solver.
 */
MathematicalProgramResult Solve(
    const MathematicalProgram& prog,
    const std::optional<Eigen::VectorXd>& initial_guess,
    const std::optional<SolverOptions>& solver_options);

/**
 * Solves an optimization program with a given initial guess.
 */
MathematicalProgramResult Solve(
    const MathematicalProgram& prog,
    const Eigen::Ref<const Eigen::VectorXd>& initial_guess);

MathematicalProgramResult Solve(const MathematicalProgram& prog);

/**
 * Solves progs[i] into result[i], optionally using initial_guess[i] and
 * solver_options[i] if given, by invoking the solver at solver_ids[i] if
 * provided.  If solver_ids[i] is nullopt then the best available solver is
 * selected for progs[i] depending on the availability of
 * the solver and the problem formulation. If solver_ids == nullptr then this is
 * done for every progs[i].
 *
 * Uses at most parallelism cores, with static scheduling by default.
 *
 * @param dynamic_schedule If dynamic_schedule is false then static scheduling
 * is used and so each core will solve approximately 1/parallelism of the
 * programs. This is most efficient when all the programs take approximately the
 * same amount of time to solve. If dynamic_schedule is true, then dynamic
 * scheduling is used and all the programs are queued into a single pool and
 * each core will take the next program off the queue when it becomes available.
 * This is best when each program takes a dramatically different amount of time
 * to solve.
 *
 * @note When using a proprietary solver (e.g. Mosek) your organization may have
 * limited license seats. It is recommended that the number of parallel solves
 * does not exceed the total number of license seats.
 *
 * @note Only programs which are thread safe are solved concurrently. Programs
 * that are not thread safe will be solved sequentially in a thread safe manner.
 *
 * @throws std::exception if initial_guess and solver_options are provided and
 * not the same size as progs.
 *
 * @throws std::exception if any of the progs are nullptr.
 *
 * @throws std::exception if any of the programs cannot be solved.
 */
std::vector<MathematicalProgramResult> SolveInParallel(
    const std::vector<const MathematicalProgram*>& progs,
    const std::vector<const Eigen::VectorXd*>* initial_guesses,
    const std::vector<const SolverOptions*>* solver_options,
    const std::vector<std::optional<SolverId>>* solver_ids,
    Parallelism parallelism = Parallelism::Max(),
    bool dynamic_schedule = false);

/**
 * Provides the same functionality as SolveInParallel, but allows for specifying
 * a single solver id and solver option that is used when solving all programs.
 *
 * @throws std::exception if the provided solver cannot solve all of progs.
 *
 * @throws std::exception if initial_guesses are provided and not the same
 * size as progs.
 *
 * @throws std::exception if any of the progs are nullptr.
 */
std::vector<MathematicalProgramResult> SolveInParallel(
    const std::vector<const MathematicalProgram*>& progs,
    const std::vector<const Eigen::VectorXd*>* initial_guesses = nullptr,
    const SolverOptions* solver_options = nullptr,
    const std::optional<SolverId>& solver_id = std::nullopt,
    Parallelism parallelism = Parallelism::Max(),
    bool dynamic_schedule = false);

}  // namespace solvers
}  // namespace drake
