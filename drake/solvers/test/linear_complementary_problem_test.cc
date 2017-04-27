/* clang-format off */
#include "drake/solvers/mathematical_program.h"
/* clang-format on */

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"

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
  EXPECT_NO_THROW(prog.Solve());
  const auto& x_value = prog.GetSolution(x);
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
  EXPECT_NO_THROW(prog.Solve());
  const auto& x_value = prog.GetSolution(x);
  const auto& y_value = prog.GetSolution(y);
  EXPECT_TRUE(CompareMatrices(x_value, Vector2d(16, 0), 1e-4,
                              MatrixCompareType::absolute));

  EXPECT_TRUE(CompareMatrices(y_value, Vector2d(16, 0), 1e-4,
                              MatrixCompareType::absolute));
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
