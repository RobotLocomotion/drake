#pragma once

#include <utility>

#include <gtest/gtest.h>

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
namespace test {
/// Test that @p prog was solved by @p desired_solver_id.
void CheckSolver(const MathematicalProgram& prog, SolverId desired_solver_id);

/// Run solver.Solve() on the given @p prog.  If the solver is absent or does
/// not find a solution, stop immediately via an exception.  (Were we to
/// continue, testing statements that examine the results would be likely to
/// fail with confusing messages, so best to avoid them entirely.)
void RunSolver(MathematicalProgram* prog,
               const MathematicalProgramSolverInterface& solver);
}  // namespace test
}  // namespace solvers
}  // namespace drake
