#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
GTEST_TEST(DiagonallyDominantMatrixConstraint, FeasibilityCheck) {
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables<2>();
  auto Y = prog.AddDiagonallyDominantMatrixConstraint(
      X.cast<symbolic::Expression>());

  auto X_constraint = prog.AddBoundingBoxConstraint(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      VectorDecisionVariable<3>(X(0, 0), X(0, 1), X(1, 1)));

  auto set_X_value = [&X_constraint](const Eigen::Vector3d& x_upper_triangle) {
    X_constraint.evaluator()->UpdateLowerBound(x_upper_triangle);
    X_constraint.evaluator()->UpdateUpperBound(x_upper_triangle);
  };

  // [1 0.9;0.9 2] is diagonally dominant
  set_X_value(Eigen::Vector3d(1, 0.9, 2));
  EXPECT_EQ(prog.Solve(), SolutionResult::kSolutionFound);

  // [1 -0.9; -0.9 2] is diagonally dominant
  set_X_value(Eigen::Vector3d(1, -0.9, 2));
  EXPECT_EQ(prog.Solve(), SolutionResult::kSolutionFound);

  // [1 1.1; 1.1 2] is not diagonally dominant
  set_X_value(Eigen::Vector3d(1, 1.1, 2));
  auto result = prog.Solve();
  EXPECT_TRUE(result == SolutionResult::kInfeasibleConstraints ||
              result == SolutionResult::kInfeasible_Or_Unbounded);
  // [1 -1.1; -1.1 2] is not diagonally dominant
  set_X_value(Eigen::Vector3d(1, -1.1, 2));
  result = prog.Solve();
  EXPECT_TRUE(result == SolutionResult::kInfeasibleConstraints ||
              result == SolutionResult::kInfeasible_Or_Unbounded);
}

GTEST_TEST(DiagonallyDominantMatrixConstraint, three_by_three_vertices) {
  // I can manually compute the polytope of (a, b, c) to make
  //     [1 a b]
  // A = [a 2 c]
  //     [b c 3]
  // to be a diagonally dominant matrix. The vertices of the polytope is
  // (0, ±1, 0), (±1, 0, 0), (0, ±1, ±2), (±1, 0, ±1), (0, 0, ±2)
  // By optimizing the LP
  // min nᵀ* (a, b, c)
  // s.t   A is diagonally dominant
  // with different vector n, we can recover the vertices of the polytope as
  // the optimal solution to this LP.
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables<3>();
  prog.AddBoundingBoxConstraint(Eigen::Vector3d(1, 2, 3),
                                Eigen::Vector3d(1, 2, 3), X.diagonal());
  prog.AddDiagonallyDominantMatrixConstraint(X.cast<symbolic::Expression>());

  auto cost =
      prog.AddLinearCost(Eigen::Vector3d::Zero(), 0,
                         VectorDecisionVariable<3>(X(0, 1), X(0, 2), X(1, 2)));

  auto solve_and_check = [&prog, &X](const Eigen::Vector3d& sol_expected,
                                     double tol) {
    const auto result = prog.Solve();
    EXPECT_EQ(result, SolutionResult::kSolutionFound);
    EXPECT_TRUE(CompareMatrices(
        prog.GetSolution(VectorDecisionVariable<3>(X(0, 1), X(0, 2), X(1, 2))),
        sol_expected, tol));
  };

  const double tol{1E-6};

  cost.evaluator()->UpdateCoefficients(Eigen::Vector3d(1, 0, 0));
  solve_and_check(Eigen::Vector3d(-1, 0, 0), tol);

  cost.evaluator()->UpdateCoefficients(Eigen::Vector3d(-1, 0, 0));
  solve_and_check(Eigen::Vector3d(1, 0, 0), tol);

  cost.evaluator()->UpdateCoefficients(Eigen::Vector3d(0, 1, 0));
  solve_and_check(Eigen::Vector3d(0, -1, 0), tol);

  cost.evaluator()->UpdateCoefficients(Eigen::Vector3d(0, -1, 0));
  solve_and_check(Eigen::Vector3d(0, 1, 0), tol);

  cost.evaluator()->UpdateCoefficients(Eigen::Vector3d(0, 0, 1));
  solve_and_check(Eigen::Vector3d(0, 0, -2), tol);

  cost.evaluator()->UpdateCoefficients(Eigen::Vector3d(0, 0, -1));
  solve_and_check(Eigen::Vector3d(0, 0, 2), tol);

  cost.evaluator()->UpdateCoefficients(Eigen::Vector3d(1, 1, 1));
  solve_and_check(Eigen::Vector3d(0, -1, -2), tol);

  cost.evaluator()->UpdateCoefficients(Eigen::Vector3d(2, 0, 1));
  solve_and_check(Eigen::Vector3d(-1, 0, -1), tol);
}

}  // namespace solvers
}  // namespace drake
