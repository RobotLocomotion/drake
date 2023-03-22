#pragma once

#include <optional>
#include <string>
#include <vector>

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
 * Solves a list of optimization programs in parallel. Optionally, an initial
 * guess can be provided for each optimization program, and a common set of
 * solver options can be used across the calls. This function will choose the
 * best available solver for each program individually depending on the
 * availability of the solver and the program formulation and so the programs in
 * @p prog_list can in general be of different type. The return is a list of
 * MathematicalProgramResult where the result of the iᵗʰ program is stored in
 * the iᵗʰ position of the return. If @p terminate_at_first_infeasible is set to
 * true, then this function will exit when any program returns infeasible. In
 * this case, if the iᵗʰ program is not solved, the iᵗʰ position of the return
 * will be std::nullopt.
 * @param prog_list A vector containing the formulation of the programs each of
 * which may possibly contain solver options.
 * @param initial_guesses A vector containing the initial guesses for the
 * decision variables. In the case that, the user does not wish to provide a
 * guess for the iᵗʰ program, the iᵗʰ entry of initial_guesses is allowed to be
 * optional.
 * @param solver_options The options in addition to those stored in each program
 * of @p prog_list. For each option entry (like print out), there are 4 ways to
 * set that option, and the priority given to the solver options is as follows
 * (from lowest / least, to highest / most):
 * 1. common option set on each MathematicalProgram itself
 * 2. common option passed as an argument to each Solve
 * 3. solver-specific option set on each MathematicalProgram itself
 * 4. solver-specific option passed as an argument to each Solve
 * @param num_threads The number of threads used to solve the programs. If this
 * is less than or equal to 0, the number of threads is set to the hardware
 * concurrency limit.
 * @param terminate_at_first_infeasible Whether to exit this program early if
 * any of the programs in @p prog_list is infeasible.
 * @return result A vector of MathematicalProgramResult where the iᵗʰ position
 * contains the result of solving the iᵗʰ program in @p prog_list through the
 * solver. The iᵗʰ may be std::nullopt if the iᵗʰ program is not solved when @p
 * terminate_at_first_infeasible is true.
 */
std::vector<std::optional<MathematicalProgramResult>> SolveInParallel(
    const std::vector<MathematicalProgram>& prog_list,
    const std::optional<std::vector<std::optional<Eigen::VectorXd>>>&
        initial_guesses,
    const std::optional<SolverOptions>& solver_options, int num_threads = -1,
    bool terminate_at_first_infeasible = false);

std::vector<std::optional<MathematicalProgramResult>> SolveInParallel(
    const std::vector<MathematicalProgram>& prog_list,
    const std::optional<std::vector<std::optional<Eigen::VectorXd>>>&
        initial_guesses,
    int num_threads = -1, bool terminate_at_first_infeasible = false);

std::vector<std::optional<MathematicalProgramResult>> SolveInParallel(
    const std::vector<MathematicalProgram>& prog_list, int num_threads = -1,
    bool terminate_at_first_infeasible = false);
}  // namespace solvers
}  // namespace drake
