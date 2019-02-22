#pragma once

#include "drake/solvers/mathematical_program.h"

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
 * @return result The result of solving the program through the solver.
 */
MathematicalProgramResult Solve(const MathematicalProgram& prog,
                                const optional<Eigen::VectorXd>& initial_guess,
                                const optional<SolverOptions>& solver_options);

/**
 * Solves an optimization program with a given initial guess, and the options
 * stored inside @p prog.
 */
MathematicalProgramResult Solve(
    const MathematicalProgram& prog,
    const Eigen::Ref<const Eigen::VectorXd>& initial_guess);

/**
 * Solves an optimization program with the initial guess and options stored in
 * @p prog. The initial guess can be accessed through prog.initial_guess(), and
 * the options can be accessed through prog.GetSolverOptionsDouble/Int/Str().
 */
MathematicalProgramResult Solve(const MathematicalProgram& prog);
}  // namespace solvers
}  // namespace drake
