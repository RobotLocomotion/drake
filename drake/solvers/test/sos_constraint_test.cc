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

GTEST_TEST(SosConstraintTest, Univariate2_Test) {
  // Find the global optimal of the polynomial
  // f(x) = (x-2)⁶ + 2 * (x-2)⁴ + 4(x-2)² + 3
  // The global optimal is obtained at x = 2, with optimal cost 3
  // We solve this by
  // max c
  // s.t f(x) - c is sum-of-squares.
  MathematicalProgram prog;
  auto x = prog.NewIndeterminates<1>();
  auto c = prog.NewContinuousVariables<1>();
  prog.AddCost(-c(0));
  symbolic::Polynomial p(pow(x(0) - 2, 6) + 2 * pow(x(0) - 2, 4) + 4 * pow(x(0) - 2, 2) + 3 - c(0), symbolic::Variables({x(0)}));
  prog.AddSosConstraint(p);

  auto result = prog.Solve();
  EXPECT_EQ(result, SolutionResult::kSolutionFound);
  EXPECT_NEAR(prog.GetSolution(c(0)), 3, 1E-4);
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
