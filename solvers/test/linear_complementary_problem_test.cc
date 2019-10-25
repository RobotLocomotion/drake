#include "drake/common/test_utilities/expect_no_throw.h"
/* clang-format off to disable clang-format-includes */
#include "drake/solvers/mathematical_program.h"
/* clang-format on */

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/solvers/solve.h"

using Eigen::Vector2d;

namespace drake {
namespace solvers {
namespace test {
// Simple linear complementarity problem example.
// @brief a hand-created LCP easily solved.
//
// Note: This test is meant to test that MathematicalProgram.Solve() works in
// this case; tests of the correctness of the Moby LCP solver itself live in
// testMobyLCP.
GTEST_TEST(testMathematicalProgram, simpleLCP) {
  MathematicalProgram prog;
  Eigen::Matrix<double, 2, 2> M;

  // clang-format off
  M << 1, 4,
      3, 1;
  // clang-format on

  Eigen::Vector2d q(-16, -15);

  auto x = prog.NewContinuousVariables<2>();

  prog.AddLinearComplementarityConstraint(M, q, x);
  MathematicalProgramResult result;
  DRAKE_EXPECT_NO_THROW(result = Solve(prog));
  const auto& x_value = result.GetSolution(x);
  EXPECT_TRUE(CompareMatrices(x_value, Vector2d(16, 0), 1e-4,
                              MatrixCompareType::absolute));
}

// Multiple LC constraints in a single optimization problem
// @brief Just two copies of the simpleLCP example, to make sure that the
// write-through of LCP results to the solution vector works correctly.
GTEST_TEST(testMathematicalProgram, multiLCP) {
  MathematicalProgram prog;
  Eigen::Matrix<double, 2, 2> M;

  // clang-format off
  M << 1, 4,
      3, 1;
  // clang-format on

  Eigen::Vector2d q(-16, -15);

  auto x = prog.NewContinuousVariables<2>();
  auto y = prog.NewContinuousVariables<2>();

  prog.AddLinearComplementarityConstraint(M, q, x);
  prog.AddLinearComplementarityConstraint(M, q, y);
  MathematicalProgramResult result;
  DRAKE_EXPECT_NO_THROW(result = Solve(prog));
  const auto& x_value = result.GetSolution(x);
  const auto& y_value = result.GetSolution(y);
  EXPECT_TRUE(CompareMatrices(x_value, Vector2d(16, 0), 1e-4,
                              MatrixCompareType::absolute));

  EXPECT_TRUE(CompareMatrices(y_value, Vector2d(16, 0), 1e-4,
                              MatrixCompareType::absolute));
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
