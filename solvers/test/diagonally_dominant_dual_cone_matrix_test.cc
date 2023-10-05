#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace solvers {
namespace {

// Returns the first index of the extreme ray v for which vᵀXv < 0. If no such
// index exists, returns -1 and X is in DD*.
int TestIn2by2DiagonallyDominantDualCone(const Eigen::Matrix2d& X) {
  // These are the extreme rays of the diagonally dominant matrices of size 2
  // (minus the extreme points with the same parity).
  Eigen::MatrixXd extreme_rays(2, 4);
  extreme_rays << Eigen::Matrix2d::Identity(), Eigen::Vector2d::Ones(),
      Eigen::Vector2d(1, -1);

  for (int c = 0; c < extreme_rays.size(); ++c) {
    if (extreme_rays.col(c).transpose() * X * extreme_rays.col(c) < 0) {
      return c;
    }
  }
  return -1;
}

// Returns the first index of the extreme ray v for which vᵀXv < 0. If no such
// index exists, returns -1 and X is in DD*.
int TestIn3by3DiagonallyDominantDualCone(const Eigen::Matrix3d& X) {
  // These are the extreme rays of the diagonally dominant matrices of size 2
  // (minus the extreme points with the same parity).
  Eigen::MatrixXd extreme_rays(3, 9);
  extreme_rays << Eigen::Matrix3d::Identity(), Eigen::Vector3d(1, 1, 0),
      Eigen::Vector3d(1, -1, 0), Eigen::Vector3d(0, 1, 1),
      Eigen::Vector3d(0, 1, -1), Eigen::Vector3d(1, 0, 1),
      Eigen::Vector3d(1, 0, -1);
  for (int c = 0; c < extreme_rays.cols(); ++c) {
    if (extreme_rays.col(c).transpose() * X * extreme_rays.col(c) < 0) {
      return c;
    }
  }
  return -1;
}
}  // namespace

GTEST_TEST(DiagonallyDominantMatrixDualConeConstraint, SizeOfReturnTest) {
  // Test the number of constraints added to the program. This should be n * n
  // for any choice of matrix X of size n.
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables<5>();
  auto dual_cone_constraints =
      prog.AddPositiveDiagonallyDominantDualConeMatrixConstraint(
          X.cast<symbolic::Expression>());
  EXPECT_EQ(dual_cone_constraints.size(), 5 * 5);
}

GTEST_TEST(DiagonallyDominantMatrixDualConeConstraint, FeasibilityCheck2by2) {
  // Test that DD* matrices are feasible.
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables<2>();
  auto dual_cone_constraints =
      prog.AddPositiveDiagonallyDominantDualConeMatrixConstraint(
          X.cast<symbolic::Expression>());

  auto X_constraint = prog.AddBoundingBoxConstraint(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      VectorDecisionVariable<3>(X(0, 0), X(0, 1), X(1, 1)));

  auto set_X_value = [&X_constraint](const Eigen::Vector3d& x_upper_triangle) {
    X_constraint.evaluator()->UpdateLowerBound(x_upper_triangle);
    X_constraint.evaluator()->UpdateUpperBound(x_upper_triangle);
    Eigen::Matrix2d ret;
    ret << x_upper_triangle(0), x_upper_triangle(1), x_upper_triangle(1),
        x_upper_triangle(2);
    return ret;
  };

  // [1, 0.9; 0.9 2] is in DD* and DD.
  set_X_value(Eigen::Vector3d(1, 0.9, 2));
  MathematicalProgramResult result = Solve(prog);
  EXPECT_TRUE(result.is_success());
  EXPECT_EQ(TestIn2by2DiagonallyDominantDualCone(result.GetSolution(X)), -1);

  // [1, -1.2; -1.2 2] is in DD* but not DD.
  set_X_value(Eigen::Vector3d(1, -1.2, 2));
  result = Solve(prog);
  EXPECT_TRUE(result.is_success());
  EXPECT_EQ(TestIn2by2DiagonallyDominantDualCone(result.GetSolution(X)), -1);

  // X_num = [2, -4; -4, 3] is not in DD*. The matrix Y = [1, 0.75; 0.75, 0.9]
  // is in DD, but 〈 X, Y 〉< 0
  const Eigen::Matrix2d X_bad = set_X_value(Eigen::Vector3d(2, -4, 3));
  result = Solve(prog);
  EXPECT_FALSE(result.is_success());
  EXPECT_GE(TestIn2by2DiagonallyDominantDualCone(X_bad), 0);
}

GTEST_TEST(DiagonallyDominantMatrixDualConeConstraint,
           three_by_three_vertices) {
  // For a matrix to be in DD*, we have to have that Xₖₖ + Xⱼⱼ ≥ 2|Xₖⱼ+ Xⱼₖ|
  // I can manually compute the vertices of the polytope of (a, b, c)
  // to make
  //     [1 a+b b+c]
  // A = [a+b 2 a+c]
  //     [b+c a+c 3]
  // be in DD*. These vertices are
  // ±(0.5, 1, 3), ±(1,0.5,1.5), ±(1.5,-3,1), ±(1.5, -3, 0.5)
  // By optimizing the LP
  // min nᵀ* (a, b, c)
  // s.t   A is in DD*
  // with different vector n, we can recover the vertices of the polytope as
  // the optimal solution to this LP.
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables<3>();
  auto a = prog.NewContinuousVariables<1>("a")(0);
  auto b = prog.NewContinuousVariables<1>("b")(0);
  auto c = prog.NewContinuousVariables<1>("c")(0);
  prog.AddLinearEqualityConstraint(X(0, 1) - (a + b), 0);
  prog.AddLinearEqualityConstraint(X(0, 2) - (b + c), 0);
  prog.AddLinearEqualityConstraint(X(1, 2) - (a + c), 0);

  prog.AddBoundingBoxConstraint(Eigen::Vector3d(1, 2, 3),
                                Eigen::Vector3d(1, 2, 3), X.diagonal());
  prog.AddPositiveDiagonallyDominantDualConeMatrixConstraint(
      X.cast<symbolic::Expression>());

  auto cost = prog.AddLinearCost(Eigen::Vector3d::Zero(), 0,
                                 VectorDecisionVariable<3>(a, b, c));
  auto solve_and_check = [&prog, &X, &a, &b, &c](
                             const Eigen::Vector3d& sol_expected, double tol) {
    const auto result = Solve(prog);
    EXPECT_TRUE(result.is_success());
    EXPECT_TRUE(
        CompareMatrices(result.GetSolution(VectorDecisionVariable<3>(a, b, c)),
                        sol_expected, tol));
    // The matrix should be positive in DD*.
    const Eigen::Matrix3d X_sol = result.GetSolution(X);
    EXPECT_EQ(TestIn3by3DiagonallyDominantDualCone(X_sol), -1);
  };

  const double tol{1E-6};

  // The costs are chosen to make the optimal solution visit each vertex.
  cost.evaluator()->UpdateCoefficients(Eigen::Vector3d(1, 1, -0.5));
  solve_and_check(Eigen::Vector3d(-0.5, -1, 3), tol);
  cost.evaluator()->UpdateCoefficients(Eigen::Vector3d(0.33, 0.33, 1.33));
  solve_and_check(Eigen::Vector3d(0.5, 1, -3), tol);

  cost.evaluator()->UpdateCoefficients(Eigen::Vector3d(-0.53, -0.58, -0.45));
  solve_and_check(Eigen::Vector3d(1, 0.5, 1.5), tol);
  cost.evaluator()->UpdateCoefficients(Eigen::Vector3d(0.82, 0.82, 0.82));
  solve_and_check(Eigen::Vector3d(-1, -0.5, -1.5), tol);

  cost.evaluator()->UpdateCoefficients(Eigen::Vector3d(0.44, 1.27, 0.44));
  solve_and_check(Eigen::Vector3d(1.5, -3, 1), tol);
  cost.evaluator()->UpdateCoefficients(Eigen::Vector3d(0.87, -0.58, 0.95));
  solve_and_check(Eigen::Vector3d(-1.5, 3, -1), tol);

  cost.evaluator()->UpdateCoefficients(Eigen::Vector3d(-0.53, 0.86, 1.0));
  solve_and_check(Eigen::Vector3d(3, -1.5, -0.5), tol);
  cost.evaluator()->UpdateCoefficients(Eigen::Vector3d(1.29, 0.40, 0.40));
  solve_and_check(Eigen::Vector3d(-3, 1.5, 0.5), tol);
}

}  // namespace solvers
}  // namespace drake
