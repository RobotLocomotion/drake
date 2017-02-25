// TODO(hongkai.dai) : delete this file when
// mixed_integer_optimization_test.cc and convex_optimization_test.cc are
// refactored.
#pragma once

#include <list>
#include <memory>

#include "drake/solvers/mathematical_program_solver_interface.h"

namespace drake {
namespace solvers {
namespace test {
void AddSolverIfAvailable(
    SolverType solver_type,
    std::list<std::unique_ptr<MathematicalProgramSolverInterface>>*
    solver_list);
}  // namespace test
}  // namespace solvers
}  // namespace drake
