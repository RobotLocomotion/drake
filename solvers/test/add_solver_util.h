// TODO(hongkai.dai) : delete this file when
// mixed_integer_optimization_test.cc and convex_optimization_test.cc are
// refactored.
#pragma once

#include <list>
#include <memory>
#include <utility>

#include "drake/solvers/solver_interface.h"

namespace drake {
namespace solvers {
namespace test {

template <typename Solver>
void AddSolverIfAvailable(
    std::list<std::unique_ptr<SolverInterface>>*
    solver_list) {
  auto solver = std::make_unique<Solver>();
  if (solver->available()) {
    solver_list->push_back(std::move(solver));
  }
}

}  // namespace test
}  // namespace solvers
}  // namespace drake
