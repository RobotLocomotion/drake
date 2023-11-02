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
  extreme_rays << Eigen::Matrix2d::Identity(2, 2), Eigen::Vector2d::Ones(),
      Eigen::Vector2d(1, -1);

  for (int c = 0; c < extreme_rays.cols(); ++c) {
    if (extreme_rays.col(c).transpose() * X * extreme_rays.col(c) < -1e-10) {
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
    if (extreme_rays.col(c).transpose() * X * extreme_rays.col(c) < -1e-10) {
      return c;
    }
  }
  return -1;
}

GTEST_TEST(DiagonallyDominantMatrixDualConeConstraint,
           FeasibilityVariableCheck2by2) {
  // Test that DD* matrices are feasible.
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables<2>();
  auto dual_cone_constraint =
      prog.AddPositiveDiagonallyDominantDualConeMatrixConstraint(X);
  // The dual cone constraint should be a single linear constraint with n²
  // inequalities where n is the number of rows in X.
  EXPECT_EQ(dual_cone_constraint.evaluator()->get_sparse_A().rows(),
            X.rows() * X.rows());
  // There are n rows with one non-zero entry in the row, and 2 * (n choose 2)
  // rows with 4 non-zero entries in the row. This requires 4*n*n-3*n non-zero
  // entries.
  EXPECT_EQ(dual_cone_constraint.evaluator()->get_sparse_A().nonZeros(),
            4 * X.rows() * X.rows() - 3 * X.rows());

  // Ensure that a sparse constraint is constructed.
  EXPECT_FALSE(dual_cone_constraint.evaluator()->is_dense_A_constructed());

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
           FeasibilityExpressionCheck2by2) {
  // Test that DD* matrices are feasible.
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables<2>();
  auto Y = prog.NewSymmetricContinuousVariables<2>();
  MatrixX<symbolic::Expression> Z(2, 2);
  Z = 2. * X + 3. * Y;

  auto GetZResult =
      [&Z](const MathematicalProgramResult& result) -> Eigen::MatrixXd {
    return result.GetSolution(Z).unaryExpr([](const symbolic::Expression& x) {
      return x.Evaluate();
    });
  };

  auto dual_cone_constraint =
      prog.AddPositiveDiagonallyDominantDualConeMatrixConstraint(Z);
  // The dual cone constraint should be a single linear constraint with n²
  // inequalities where n is the number of rows in Z.
  EXPECT_EQ(dual_cone_constraint.evaluator()->get_sparse_A().rows(),
            Z.rows() * Z.rows());
  // Ensure that a sparse constraint is constructed.
  EXPECT_FALSE(dual_cone_constraint.evaluator()->is_dense_A_constructed());

  VectorX<symbolic::Expression> z_upper(3);
  z_upper << Z(0, 0), Z(0, 1), Z(1, 1);
  auto Z_constraint =
      prog.AddLinearEqualityConstraint(z_upper, Eigen::Vector3d::Zero());

  auto set_Z_value = [&Z_constraint](const Eigen::Vector3d& z_upper_triangle) {
    Z_constraint.evaluator()->UpdateLowerBound(z_upper_triangle);
    Z_constraint.evaluator()->UpdateUpperBound(z_upper_triangle);
    Eigen::Matrix2d ret;
    ret << z_upper_triangle(0), z_upper_triangle(1), z_upper_triangle(1),
        z_upper_triangle(2);
    return ret;
  };

  // [1, 0.9; 0.9 2] is in DD* and DD.
  set_Z_value(Eigen::Vector3d(1, 0.9, 2));
  MathematicalProgramResult result = Solve(prog);
  EXPECT_TRUE(result.is_success());
  EXPECT_EQ(TestIn2by2DiagonallyDominantDualCone(GetZResult(result)), -1);

  // [1, -1.2; -1.2 2] is in DD* but not DD.
  set_Z_value(Eigen::Vector3d(1, -1.2, 2));
  result = Solve(prog);
  EXPECT_TRUE(result.is_success());
  EXPECT_EQ(TestIn2by2DiagonallyDominantDualCone(GetZResult(result)), -1);

  // Z_num = [2, -4; -4, 3] is not in DD*. The matrix Y = [1, 0.75; 0.75, 0.9]
  // is in DD, but 〈 X, Y 〉< 0
  const Eigen::Matrix2d Z_bad = set_Z_value(Eigen::Vector3d(2, -4, 3));
  result = Solve(prog);
  EXPECT_FALSE(result.is_success());
  EXPECT_GE(TestIn2by2DiagonallyDominantDualCone(Z_bad), 0);
}

GTEST_TEST(DiagonallyDominantMatrixDualConeConstraint,
           ThreeByThreeVerticesVariable) {
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
  auto dual_cone_constraint =
      prog.AddPositiveDiagonallyDominantDualConeMatrixConstraint(X);
  // The dual cone constraint should be a single linear constraint with n²
  // inequalities where n is the number of rows in X.
  EXPECT_EQ(dual_cone_constraint.evaluator()->get_sparse_A().rows(),
            X.rows() * X.rows());
  // There are n rows with one non-zero entry in the row, and 2 * (n choose 2)
  // rows with 4 non-zero entries in the row. This requires 4*n*n-3*n non-zero
  // entries.
  EXPECT_EQ(dual_cone_constraint.evaluator()->get_sparse_A().nonZeros(),
            4 * X.rows() * X.rows() - 3 * X.rows());

  // Ensure that a sparse constraint is constructed.
  EXPECT_FALSE(dual_cone_constraint.evaluator()->is_dense_A_constructed());

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

GTEST_TEST(DiagonallyDominantMatrixDualConeConstraint,
           ThreeByThreeVerticesExpression) {
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
  auto Y = prog.NewSymmetricContinuousVariables<3>();
  MatrixX<symbolic::Expression> Z(3, 3);
  Z = 2. * X + 3. * Y;

  auto GetZResult =
      [&Z](const MathematicalProgramResult& result) -> Eigen::MatrixXd {
    return result.GetSolution(Z).unaryExpr([](const symbolic::Expression& x) {
      return x.Evaluate();
    });
  };

  auto a = prog.NewContinuousVariables<1>("a")(0);
  auto b = prog.NewContinuousVariables<1>("b")(0);
  auto c = prog.NewContinuousVariables<1>("c")(0);
  prog.AddLinearEqualityConstraint(Z(0, 1) - (a + b), 0);
  prog.AddLinearEqualityConstraint(Z(0, 2) - (b + c), 0);
  prog.AddLinearEqualityConstraint(Z(1, 2) - (a + c), 0);

  prog.AddLinearEqualityConstraint(Z.diagonal(), Eigen::Vector3d(1, 2, 3));
  auto dual_cone_constraint =
      prog.AddPositiveDiagonallyDominantDualConeMatrixConstraint(Z);
  // The dual cone constraint should be a single linear constraint with n²
  // inequalities where n is the number of rows in X.
  EXPECT_EQ(dual_cone_constraint.evaluator()->get_sparse_A().rows(),
            Z.rows() * Z.rows());
  // Ensure that a sparse constraint is constructed.
  EXPECT_FALSE(dual_cone_constraint.evaluator()->is_dense_A_constructed());

  auto cost = prog.AddLinearCost(Eigen::Vector3d::Zero(), 0,
                                 VectorDecisionVariable<3>(a, b, c));
  auto solve_and_check = [&prog, &GetZResult, &a, &b, &c](
                             const Eigen::Vector3d& sol_expected, double tol) {
    const auto result = Solve(prog);
    EXPECT_TRUE(result.is_success());
    EXPECT_TRUE(
        CompareMatrices(result.GetSolution(VectorDecisionVariable<3>(a, b, c)),
                        sol_expected, tol));
    // The matrix should be positive in DD*.
    const Eigen::Matrix3d Z_sol = GetZResult(result);
    EXPECT_EQ(TestIn3by3DiagonallyDominantDualCone(Z_sol), -1);
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

}  // namespace
}  // namespace solvers
}  // namespace drake
