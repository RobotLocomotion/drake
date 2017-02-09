#pragma once

#include <list>
#include <memory>
#include <string>
#include <utility>

#include "gtest/gtest.h"

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
namespace test {
void CheckSolver(const MathematicalProgram& prog,
                 SolverType desired_solver_type);

void RunSolver(MathematicalProgram* prog,
               const MathematicalProgramSolverInterface& solver);

// TODO (hongkai.dai) : delete this function when mixed_integer_optimization_test.cc and convex_optimization_test.cc are refactored.
void AddSolverIfAvailable(
    SolverType solver_type,
    std::list<std::unique_ptr<MathematicalProgramSolverInterface>>*
        solver_list);

}  // namespace test
}  // namespace solvers
}  // namespace drake
