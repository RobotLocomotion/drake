#pragma once

#include <functional>
#include <memory>
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

namespace internal {
template <typename T>
concept MathematicalProgramPtr =
    std::is_same_v<T, std::unique_ptr<MathematicalProgram>> ||
    std::is_same_v<T, MathematicalProgram*> ||
    std::is_same_v<T, const MathematicalProgram*>;
}  // namespace internal

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
template <typename T>
  requires internal::MathematicalProgramPtr<T>
std::vector<MathematicalProgramResult> SolveInParallel(
    const std::vector<T>& progs,
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
template <typename T>
  requires internal::MathematicalProgramPtr<T>
std::vector<MathematicalProgramResult> SolveInParallel(
    const std::vector<T>& progs,
    const std::vector<const Eigen::VectorXd*>* initial_guesses = nullptr,
    const SolverOptions* solver_options = nullptr,
    const std::optional<SolverId>& solver_id = std::nullopt,
    Parallelism parallelism = Parallelism::Max(),
    bool dynamic_schedule = false);  // requires MathematicalProgramPtr;

/**
 * Provides the same functionality as SolveInParallel, but the programs, initial
 * guesses, solver options and solver ids are created using a generator.
 *
 * The input to the generator is the current thread number and an index i and
 * the output is the ith program, guess, option and solver_id respectively. The
 * index i is iterated from [range_start, range_end).
 *
 * If the output of prog_generator is nullptr, then the program is skipped. This
 * can be useful if e.g. we wish to exit the parallel solve early. The user is
 * responsible for ensuring that no pointer is not dangling.
 *
 * The output of the initial_guesses_generator, solver_options_generator and
 * solver_ids can be std::nullopt
 *
 * After the ith program is solved, the prog_teardown function is called with
 * the ith program and its result, the current thread number, and the index i to
 * potentially clean up after the solve or perform some other callback.
 *
 * @note This method may call the generator on non-threadsafe programs more than
 * once. If generating the programs is an expensive operation, consider solving
 * these programs sequentially without calling this method.
 *
 * @return A vector of size range_end - range_start where the 0th index is
 * populated with the result of solving prog_generator(range_start) and the last
 * index is populated with the result of solving prog_generator(range_end-1);
 */
template <typename T>
  requires internal::MathematicalProgramPtr<T>
std::vector<MathematicalProgramResult> SolveInParallel(
    const std::function<T(int64_t, int64_t)>& prog_generator,
    const int64_t range_start, const int64_t range_end,
    const std::function<std::optional<Eigen::VectorXd>(int64_t, int64_t)>&
        initial_guesses_generator,
    const std::function<std::optional<SolverOptions>(int64_t, int64_t)>&
        solver_options_generator =
            [](int64_t, int64_t) {
              return std::nullopt;
            },
    const std::function<std::optional<SolverId>(int64_t, int64_t)>&
        solver_ids_generator =
            [](int64_t, int64_t) {
              return std::nullopt;
            },
    Parallelism parallelism = Parallelism::Max(), bool dynamic_schedule = false,
    const std::function<void(T*, const MathematicalProgramResult&, int64_t,
                             int64_t)>* prog_teardown = nullptr);

/**
 * Provides the same functionality as SolveInParallel, but the programs and
 * initial guesses are created using a generator. This overloads allows for
 * specifying a single solver id and solver option that is used when solving all
 * programs.
 */
template <typename T>
  requires internal::MathematicalProgramPtr<T>
std::vector<MathematicalProgramResult> SolveInParallel(
    const std::function<T(int64_t, int64_t)>& prog_generator,
    const int64_t range_start, const int64_t range_end,
    const std::function<std::optional<Eigen::VectorXd>(int64_t, int64_t)>&
        initial_guesses_generator =
            [](int64_t, int64_t) {
              return std::nullopt;
            },
    const SolverOptions* solver_options = nullptr,
    const std::optional<SolverId>& solver_id = std::nullopt,
    Parallelism parallelism = Parallelism::Max(), bool dynamic_schedule = false,
    const std::function<void(T*, const MathematicalProgramResult&, int64_t,
                             int64_t)>* prog_teardown = nullptr);

}  // namespace solvers
}  // namespace drake
