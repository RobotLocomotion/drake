#include "drake/solvers/mathematical_program.h"

#include <gtest/gtest.h>

namespace drake {
namespace solvers {
namespace {

GTEST_TEST(SosConstraintTest, Univariate1) {
  MathematicalProgram prog;
  auto x = prog.NewIndeterminates<1>();
  // p = x₀² + 2x₀ + 1
  prog.AddSosConstraint(2 * pow(x(0), 2) + 2 * x(0) + 1);
  auto result = prog.Solve();
  EXPECT_EQ(result, SolutionResult::kSolutionFound);
}

GTEST_TEST(SosConstraintTest, Multivariate1) {
  MathematicalProgram prog;
  auto x = prog.NewIndeterminates<2>();
  // p = 2x₀⁴ + 2x₀³x₁ - x₀²x₁² + 5x₁⁴
  prog.AddSosConstraint(2 * pow(x(0), 4) + 2 * pow(x(0), 3) * x(1) -
                        pow(x(0), 2) * pow(x(1), 2) + 5 * pow(x(1), 4));
  auto result = prog.Solve();
  EXPECT_EQ(result, SolutionResult::kSolutionFound);
}

}  // namespace
}  // namespace solvers
}  // namespace drake
