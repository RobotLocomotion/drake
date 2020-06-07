#pragma once

#include <optional>
#include <string>
#include <vector>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"

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

/** Some solvers (e.g. SNOPT) provide a "best-effort solution" even when they
 * determine that a problem is infeasible.  This method will return the
 * descriptions corresponding to the constraints for which `CheckSatisfied`
 * evaluates to false given the reported solution.  This can be very useful
 * for debugging.
 *
 * @param prog A MathematicalProgram
 * @param result A MathematicalProgramResult obtained by solving @p prog.
 * @param tolerance A positive tolerance to check the constraint violation.
 * If no tolerance is provided, this method will attempt to obtain the
 * constraint tolerance from the solver, or insert a conservative default
 * tolerance.
 *
 * Note: Currently most constraints have the empty string as the
 * description, so the NiceTypeName of the Constraint is used instead.  Use
 * e.g.
 * `prog.AddConstraint(x == 1).evaluator().set_description(str)`
 * to make this method more specific/useful. */
std::vector<std::string> GetInfeasibleConstraints(
    const MathematicalProgram& prog, const MathematicalProgramResult& result,
    std::optional<double> tolerance = std::nullopt);

}  // namespace solvers
}  // namespace drake
