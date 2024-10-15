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
 * @param initial_guess The initial guess for the decision variables.
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
 * selected for each progs[i] individually depending on the availability of
 * the solver and the problem formulation. If solver_ids == nullptr then this is
 * done for every progs[i]. Uses at most parallelism cores, with dynamic
 * scheduling by default.
 *
 * @note only programs which are thread safe are solved concurrently. Programs
 * which are not thread safe will be solved sequentially in a thread safe
 * manner.
 *
 * @throws if initial guess and solver options are provided and not the same
 * size as progs.
 *
 * @throws if any of the progs are nullptr.
 *
 * @throws if the solver specified by solver_ids[i] cannot solve progs[i].
 */
std::vector<MathematicalProgramResult> SolveInParallel(
    const std::vector<const MathematicalProgram*>& progs,
    const std::vector<const Eigen::VectorXd*>* initial_guesses = nullptr,
    const std::vector<const SolverOptions*>* solver_options = nullptr,
    const std::vector<std::optional<SolverId>>* solver_ids = nullptr,
    const Parallelism parallelism = Parallelism::Max(),
    bool dynamic_schedule = true);

/**
 * Solves progs[i] into result[i], optionally using initial_guesses[i] and
 * solver_options if given, by invoking the solver if provided. If
 * solvers is not provided then the best available solver is constructed which
 * can solve all of progs. Note that the same solver options are used for all
 * the programs. Uses at most parallelism cores, with dynamic scheduling by
 * default.
 *
 * @note only programs which are thread safe are solved concurrently. Programs
 * which are not thread safe will be solved sequentially in a thread safe
 * manner.
 *
 * @throws if the provided solver cannot solve all of progs or if the solver_id
 * is not provided and there is not a single solver which can solve all of
 * progs.
 *
 * @throws if initial_guesses are provided and not the same
 * size as progs.
 *
 * @throws if any of the progs are nullptr.
 */
std::vector<MathematicalProgramResult> SolveInParallel(
    const std::vector<const MathematicalProgram*>& progs,
    const std::vector<const Eigen::VectorXd*>* initial_guesses = nullptr,
    const std::optional<SolverOptions>& solver_options = std::nullopt,
    const std::optional<SolverId>& solver_id = std::nullopt,
    const Parallelism parallelism = Parallelism::Max(),
    bool dynamic_schedule = true);

}  // namespace solvers
}  // namespace drake
