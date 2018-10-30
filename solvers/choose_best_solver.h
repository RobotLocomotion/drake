#pragma once

#include <memory>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_solver_interface.h"

namespace drake {
namespace solvers {
/**
 * Choose the best solver given the formulation in the optimization program and
 * the availability of the solvers.
 * @throw invalid_argument if there is no available solver for @p prog.
 */
std::unique_ptr<MathematicalProgramSolverInterface> ChooseBestSolver(
    const MathematicalProgram& prog);
}  // namespace solvers
}  // namespace drake
