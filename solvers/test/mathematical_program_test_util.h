#pragma once

#include <optional>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solver_interface.h"

namespace drake {
namespace solvers {
namespace test {
/// Run solver.Solve() on the given @p prog.  If the solver is absent or does
/// not find a solution, stop immediately via an exception.  (Were we to
/// continue, testing statements that examine the results would be likely to
/// fail with confusing messages, so best to avoid them entirely.)
MathematicalProgramResult RunSolver(
    const MathematicalProgram& prog,
    const SolverInterface& solver,
    const std::optional<Eigen::VectorXd>& initial_guess = {});

/// Determine if two bindings are the same. Two bindings are the same if
///
/// 1. They contain the same constraint pointer.
/// 2. Their bound variables are the same.
template <typename Constraint>
::testing::AssertionResult IsBindingEqual(const Binding<Constraint>& binding1,
                                          const Binding<Constraint>& binding2) {
  if (binding1.evaluator() != binding2.evaluator()) {
    return ::testing::AssertionFailure()
           << "Constraint pointers are not the same.";
  }
  if (binding1.variables().rows() != binding2.variables().rows()) {
    return ::testing::AssertionFailure()
           << "Constraint variable sizes are not the same.";
  }
  for (int i = 0; i < binding1.variables().rows(); ++i) {
    if (!binding1.variables()(i).equal_to(binding2.variables()(i))) {
      return ::testing::AssertionFailure()
             << "Constraint variable mismatch:(" << binding1.variables()(i)
             << " vs. " << binding2.variables()(i) << ")";
    }
  }
  return ::testing::AssertionSuccess() << "Same binding.";
}

/// Determines if two vectors of bindings are the same.
template <typename Constraint>
::testing::AssertionResult IsVectorOfBindingEqual(
    const std::vector<Binding<Constraint>>& bindings1,
    const std::vector<Binding<Constraint>>& bindings2) {
  if (bindings1.size() != bindings2.size()) {
    return ::testing::AssertionFailure()
           << "Size mismatches: (" << bindings1.size() << " vs. "
           << bindings2.size() << ").";
  }
  for (int i = 0; i < static_cast<int>(bindings1.size()); ++i) {
    auto result = IsBindingEqual(bindings1[i], bindings2[i]);
    if (!result) return result;
  }
  return ::testing::AssertionSuccess() << " Same bindings.";
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
