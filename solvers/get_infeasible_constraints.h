#pragma once

#include <optional>
#include <string>
#include <vector>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"

namespace drake {
namespace solvers {
/** @anchor get_infeasible_constraints
 * @name Get infeasible constraints
 * Some solvers (e.g. SNOPT) provide a "best-effort solution" even when they
 * determine that a problem is infeasible.  This method will return the
 * descriptions corresponding to the constraints for which `CheckSatisfied`
 * evaluates to false given the reported solution.  This can be very useful
 * for debugging. Note that this feature is available only when the optimization
 * problem is solved through certain solvers (like SNOPT, IPOPT) which provide a
 * "best-effort solution". Some solvers (like Gurobi) don't return the
 * "best-effor solution" when the problem is infeasible, and this feature is
 * hence unavailable.
 */
//@{

/**
 * See @ref get_infeasible_constraints for more information.
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

/**
 * See @ref get_infeasible_constraints for more information.
 * @param prog A MathematicalProgram
 * @param result A MathematicalProgramResult obtained by solving @p prog.
 * @param tolerance A positive tolerance to check the constraint violation.
 * If no tolerance is provided, this method will attempt to obtain the
 * constraint tolerance from the solver, or insert a conservative default
 * tolerance.
 * @return infeasible_bindings A vector of all infeasible bindings (constraints
 * together with the associated variables) at the best-effort solution.
 */
std::vector<Binding<Constraint>> GetInfeasibleConstraintBindings(
    const MathematicalProgram& prog, const MathematicalProgramResult& result,
    std::optional<double> tolerance = std::nullopt);
// @}
}  // namespace solvers
}  // namespace drake
