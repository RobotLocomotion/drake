#pragma once

#include <utility>

#include <gtest/gtest.h>

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
namespace test {
void CheckSolver(const MathematicalProgram& prog,
                 SolverType desired_solver_type);

void RunSolver(MathematicalProgram* prog,
               const MathematicalProgramSolverInterface& solver);
}  // namespace test
}  // namespace solvers
}  // namespace drake
