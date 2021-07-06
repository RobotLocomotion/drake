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
}  // namespace solvers
}  // namespace drake
