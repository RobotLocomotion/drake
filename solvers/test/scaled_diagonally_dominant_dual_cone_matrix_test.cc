#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace solvers {
namespace {
// Make all the generator of the SDD dual cone, namely the n x 2 matrices V
// where each column has exactly one non-zero element equal to 1.
std::vector<Eigen::MatrixX2d> MakeSDDDualConeGenerator(int n) {
  std::vector<Eigen::MatrixX2d> generators(n * n, Eigen::MatrixX2d::Zero(n, 2));
  int ctr = 0;
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      generators.at(ctr)(i, 0) = 1;
      generators.at(ctr)(j, 1) = 1;
      ++ctr;
    }
  }
  return generators;
}

// Returns the first index of the extreme ray V for which VᵀXV is not positive
// semidefinite. If no such index exists, returns -1 and X is in SDD*.
int TestInDualConeByGenerators(const Eigen::MatrixXd& X) {
  DRAKE_DEMAND(X.rows() == X.cols());
  const std::vector<Eigen::MatrixX2d> generators =
      MakeSDDDualConeGenerator(X.rows());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver;

  for (int i = 0; i < ssize(generators); ++i) {
    const Eigen::Matrix2d product =
        generators.at(i).transpose() * X * generators.at(i);
    solver.compute(product);
    if (!(solver.eigenvalues().array() >= -1e-12).all()) {
      return i;
    }
  }
  return -1;
}

GTEST_TEST(ScaledDiagonallyDominantMatrixDualConeConstraint, SizeOfReturnTest) {
  // Test the number of constraints added to the program. This should be n
  // choose 2 for any choice of matrix X of size n.
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables<5>();
  auto dual_cone_constraints =
      prog.AddScaledDiagonallyDominantDualConeMatrixConstraint(
          X.cast<symbolic::Expression>());
  EXPECT_EQ(dual_cone_constraints.size(), 10);

  auto X2 = prog.NewSymmetricContinuousVariables<7>();
  auto dual_cone_constraints2 =
      prog.AddScaledDiagonallyDominantDualConeMatrixConstraint(
          X2.cast<symbolic::Expression>());
  EXPECT_EQ(dual_cone_constraints2.size(), 21);
}

GTEST_TEST(ScaledDiagonallyDominantMatrixDualConeConstraint,
           FeasibilityCheck3by3Variable) {
  // SDD = PD = PD* = SDD* when n = 2, so the smallest meaningful test is 3.
  // Test that SDD* matrices are feasible.
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables<3>();
  auto dual_cone_constraints =
      prog.AddScaledDiagonallyDominantDualConeMatrixConstraint(X);
  VectorXDecisionVariable x_flat(6);
  x_flat << X(0, 0), X(0, 1), X(0, 2), X(1, 1), X(1, 2), X(2, 2);
  auto X_constraint = prog.AddBoundingBoxConstraint(
      Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6), x_flat);
  auto set_X_value = [&X_constraint](const Eigen::Matrix3d& X_val) {
    Eigen::VectorXd x_upper_triangle(6);
    x_upper_triangle << X_val(0, 0), X_val(0, 1), X_val(0, 2), X_val(1, 1),
        X_val(1, 2), X_val(2, 2);
    X_constraint.evaluator()->UpdateLowerBound(x_upper_triangle);
    X_constraint.evaluator()->UpdateUpperBound(x_upper_triangle);
  };

  // This matrix is in both PSD and SDD*
  Eigen::Matrix3d test1;
  // clang-format off
  test1 << 2., 1., 1.,
           1., 3., 1.,
           1., 1., 4.;

  // clang-format on
  set_X_value(test1);
  MathematicalProgramResult result = Solve(prog);
  EXPECT_TRUE(result.is_success());
  EXPECT_EQ(TestInDualConeByGenerators(result.GetSolution(X)), -1);

  // This matrix is in both SDD* but not is not PSD.
  Eigen::Matrix3d test2;
  // clang-format off
  test2 << 32., 8., 0.,
           8., 2., 3.,
           0., 3., 8.;
  // clang-format on
  set_X_value(test2);
  result = Solve(prog);
  EXPECT_TRUE(result.is_success());
  EXPECT_EQ(TestInDualConeByGenerators(result.GetSolution(X)), -1);

  // This matrix is in DD* but not SDD*.
  Eigen::Matrix3d test3;
  // clang-format off
  test3 << 32., 9., 0.,
            9., 2., 3.,
            0., 3., 8.;
  // clang-format on
  set_X_value(test3);
  result = Solve(prog);
  EXPECT_FALSE(result.is_success());
  EXPECT_GE(TestInDualConeByGenerators(result.GetSolution(X)), -1);

  // Verifies that test3 matrix is in DD*.
  MathematicalProgram prog_dd = MathematicalProgram();
  auto X_dd = prog_dd.NewSymmetricContinuousVariables<3>();
  auto dd_dual_cone_constraints =
      prog_dd.AddPositiveDiagonallyDominantDualConeMatrixConstraint(X_dd);
  Eigen::VectorXd test3_flat(6);
  test3_flat << test3(0, 0), test3(0, 1), test3(0, 2), test3(1, 1), test3(1, 2),
      test3(2, 2);

  VectorXDecisionVariable x_dd_flat(6);
  x_dd_flat << X_dd(0, 0), X_dd(0, 1), X_dd(0, 2), X_dd(1, 1), X_dd(1, 2),
      X_dd(2, 2);
  auto X_dd_constraint =
      prog_dd.AddBoundingBoxConstraint(test3_flat, test3_flat, x_dd_flat);
  auto result_dd = Solve(prog_dd);
  EXPECT_TRUE(result_dd.is_success());
}

GTEST_TEST(ScaledDiagonallyDominantMatrixDualConeConstraint,
           FeasibilityCheck3by3Expression) {
  // SDD = PD = PD* = SDD* when n = 2, so the smallest meaningful test is 3.
  // Test that SDD* matrices are feasible.
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
  VectorX<symbolic::Expression> z_upper(6);
  z_upper << Z(0, 0), Z(0, 1), Z(0, 2), Z(1, 1), Z(1, 2), Z(2, 2);

  auto dual_cone_constraints =
      prog.AddScaledDiagonallyDominantDualConeMatrixConstraint(Z);

  auto Z_constraint =
      prog.AddLinearEqualityConstraint(z_upper, Eigen::VectorXd::Zero(6));

  auto set_Z_value = [&Z_constraint](const Eigen::Matrix3d& Z_val) {
    Eigen::VectorXd z_upper_triangle(6);
    z_upper_triangle << Z_val(0, 0), Z_val(0, 1), Z_val(0, 2), Z_val(1, 1),
        Z_val(1, 2), Z_val(2, 2);
    Z_constraint.evaluator()->UpdateLowerBound(z_upper_triangle);
    Z_constraint.evaluator()->UpdateUpperBound(z_upper_triangle);
  };

  // This matrix is in both PSD and SDD*
  Eigen::Matrix3d test1;
  // clang-format off
  test1 << 2., 1., 1.,
           1., 3., 1.,
           1., 1., 4.;
  // clang-format on
  set_Z_value(test1);
  MathematicalProgramResult result = Solve(prog);
  EXPECT_TRUE(result.is_success());
  EXPECT_EQ(TestInDualConeByGenerators(GetZResult(result)), -1);

  // This matrix is in both SDD* but not is not PSD.
  Eigen::Matrix3d test2;

  // clang-format off
  test2 << 32., 8., 0.,
           8., 2., 3.,
           0., 3., 8.;
  // clang-format on
  set_Z_value(test2);
  result = Solve(prog);
  EXPECT_TRUE(result.is_success());
  EXPECT_EQ(TestInDualConeByGenerators(GetZResult(result)), -1);

  // This matrix is in DD* but not SDD*.
  Eigen::Matrix3d test3;
  // clang-format off
  test3 << 32., 9., 0.,
           9., 2., 3.,
           0., 3., 8.;
  // clang-format on
  set_Z_value(test3);
  result = Solve(prog);
  EXPECT_FALSE(result.is_success());
  EXPECT_GE(TestInDualConeByGenerators(GetZResult(result)), -1);

  // Verifies that test3 matrix is in DD*.
  MathematicalProgram prog_dd = MathematicalProgram();
  auto X_dd = prog_dd.NewSymmetricContinuousVariables<3>();
  auto dd_dual_cone_constraints =
      prog_dd.AddPositiveDiagonallyDominantDualConeMatrixConstraint(
          X_dd.cast<symbolic::Expression>());
  Eigen::VectorXd test3_flat(6);
  test3_flat << test3(0, 0), test3(0, 1), test3(0, 2), test3(1, 1), test3(1, 2),
      test3(2, 2);
  VectorXDecisionVariable x_dd_flat(6);
  x_dd_flat << X_dd(0, 0), X_dd(0, 1), X_dd(0, 2), X_dd(1, 1), X_dd(1, 2),
      X_dd(2, 2);
  auto X_dd_constraint =
      prog_dd.AddBoundingBoxConstraint(test3_flat, test3_flat, x_dd_flat);
  auto result_dd = Solve(prog_dd);
  EXPECT_TRUE(result_dd.is_success());
}

GTEST_TEST(ScaledDiagonallyDominantMatrixDualConeConstraint,
           MaximizeOffDiagonalTest) {
  // For a matrix to be in SDD*, we have to have that XₖₖXⱼⱼ ≥ Xₖⱼ * Xⱼₖ
  // We consider the matrix
  //     [1 a b c]
  // A = [a 2 d e]
  //     [b d 3 f]
  //     [c e f 4]
  // In this test we maximize x ∈ {a,b,c,d,e,f} while constraining all other
  // variables to be equal to 0 to obtain optimal solutions (√2, 0, 0, 0, 0, 0),
  // (0, √3, 0, 0, 0, 0), (0, 0, 2, 0, 0, 0), (0, 0, 0, √6, 0, 0), (0, 0, 0, 0,
  // √8, 0), (0, 0, 0, 0, 0, √12)

  // Tests
  // max A(i,j) subject to
  // A ∈ SDD*
  // A(r,c) = 0 if r > c and  (r,c) ≠ (i,j)
  // The optimal solution should be A(i,j) = √(A(i,i)*A(j,j))
  auto TestIdx = [](int i, int j) {
    MathematicalProgram prog;
    auto X = prog.NewSymmetricContinuousVariables<4>("X");
    auto Y = prog.NewSymmetricContinuousVariables<4>("Y");
    MatrixX<symbolic::Expression> Z(4, 4);
    Z = 2. * X + 3. * Y;

    prog.AddBoundingBoxConstraint(Eigen::Vector4d(1, 2, 3, 4),
                                  Eigen::Vector4d(1, 2, 3, 4), X.diagonal());
    // Y is constrained to be 0 so we can control the value of Z.
    prog.AddLinearEqualityConstraint(Y == Eigen::Matrix4d::Zero());

    auto dual_cone_constraints =
        prog.AddScaledDiagonallyDominantDualConeMatrixConstraint(X);
    auto dual_cone_constraints_Z =
        prog.AddScaledDiagonallyDominantDualConeMatrixConstraint(Z);
    for (int r = 0; r < X.rows(); ++r) {
      for (int c = r + 1; c < X.cols(); ++c) {
        if (r != i || c != j) {
          prog.AddLinearEqualityConstraint(X(r, c) == 0);
        }
      }
    }
    prog.AddLinearCost(-X(i, j));
    MathematicalProgramResult result = Solve(prog);
    EXPECT_TRUE(result.is_success());
    Eigen::Matrix4d X_res = result.GetSolution(X);
    Eigen::Matrix4d Z_res =
        result.GetSolution(Z).unaryExpr([](const symbolic::Expression& x) {
          return x.Evaluate();
        });
    Eigen::Matrix4d X_expected = Eigen::Matrix4d::Zero();
    X_expected.diagonal() = Eigen::Vector4d(1, 2, 3, 4);
    X_expected(i, j) = sqrt(X_expected(i, i) * X_expected(j, j));
    X_expected(j, i) = X_expected(i, j);
    EXPECT_TRUE(CompareMatrices(X_res, X_expected, 1e-8));
    // Y is constrained to be 0 so we expect the value of Z = 2*X.
    EXPECT_TRUE(CompareMatrices(Z_res, 2 * X_expected, 1e-8));
    EXPECT_EQ(TestInDualConeByGenerators(X_res), -1);
    EXPECT_EQ(TestInDualConeByGenerators(Z_res), -1);
  };

  for (int i = 0; i < 4; ++i) {
    for (int j = i + 1; j < 4; ++j) {
      TestIdx(i, j);
    }
  }
}

}  // namespace
}  // namespace solvers
}  // namespace drake
