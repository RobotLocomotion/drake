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
    if (!(solver.eigenvalues().array() >= 0).all()) {
      return i;
    }
  }
  return -1;
}
}  // namespace

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
           FeasibilityCheck3by3) {
  // SDD = PD = PD* = SDD* when n = 2, so the smallest meaningful test is 3.
  // Test that SDD* matrices are feasible.
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables<3>();
  auto dual_cone_constraints =
      prog.AddScaledDiagonallyDominantDualConeMatrixConstraint(
          X.cast<symbolic::Expression>());

  auto X_constraint = prog.AddBoundingBoxConstraint(
      Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6),
      VectorDecisionVariable<6>(X(0, 0), X(0, 1), X(0, 2), X(1, 1), X(1, 2),
                                X(2, 2)));

  auto set_X_value = [&X_constraint](const Eigen::Matrix3d& X_val) {
    Eigen::VectorXd x_upper_triangle(6);
    x_upper_triangle << X_val(0, 0), X_val(0, 1), X_val(0, 2), X_val(1, 1),
        X_val(1, 2), X_val(2, 2);
    X_constraint.evaluator()->UpdateLowerBound(x_upper_triangle);
    X_constraint.evaluator()->UpdateUpperBound(x_upper_triangle);
  };

  // This matrix is in both PSD and SDD*
  // clang-format off
  const Eigen::Matrix3d test1{{2, 1, 1},
                              {1, 3, 1},
                              {1, 1, 4}};
  // clang-format on
  set_X_value(test1);
  MathematicalProgramResult result = Solve(prog);
  EXPECT_TRUE(result.is_success());
  EXPECT_EQ(TestInDualConeByGenerators(result.GetSolution(X)), -1);

  // This matrix is in both SDD* but not is not PSD.
  // clang-format off
  const Eigen::Matrix3d test2{{32, 8, 0},
                              {8, 2, 3},
                              {0, 3, 8}};
  // clang-format on
  set_X_value(test2);
  result = Solve(prog);
  EXPECT_TRUE(result.is_success());
  EXPECT_EQ(TestInDualConeByGenerators(result.GetSolution(X)), -1);

  // This matrix is in DD* but not SDD*.
  // clang-format off
  const Eigen::Matrix3d test3{{32, 9, 0},
                              {9, 2, 3},
                              {0, 3, 8}};
  // clang-format on
  set_X_value(test3);
  result = Solve(prog);
  EXPECT_FALSE(result.is_success());
  EXPECT_GE(TestInDualConeByGenerators(result.GetSolution(X)), -1);

  // Verifies that test3 matrix is in DD*.
  MathematicalProgram prog_dd = MathematicalProgram();
  auto X_dd = prog_dd.NewSymmetricContinuousVariables<3>();
  auto dd_dual_cone_constraints =
      prog_dd.AddPositiveDiagonallyDominantDualConeMatrixConstraint(
          X_dd.cast<symbolic::Expression>());
  Eigen::VectorXd test3_flat(6);
  test3_flat << test3(0, 0), test3(0, 1), test3(0, 2), test3(1, 1), test3(1, 2),
      test3(2, 2);

  auto X_dd_constraint = prog_dd.AddBoundingBoxConstraint(
      test3_flat, test3_flat,
      VectorDecisionVariable<6>(X_dd(0, 0), X_dd(0, 1), X_dd(0, 2), X_dd(1, 1),
                                X_dd(1, 2), X_dd(2, 2)));
  auto result_dd = Solve(prog_dd);
  EXPECT_TRUE(result_dd.is_success());
}

GTEST_TEST(ScaledDiagonallyDominantMatrixDualConeConstraint,
           MaximizeOffDiagonalTest) {
  // For a matrix to be in sDD*, we have to have that XₖₖXⱼⱼ ≥ Xₖⱼ * Xⱼₖ
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
    auto X = prog.NewSymmetricContinuousVariables<4>();
    prog.AddBoundingBoxConstraint(Eigen::Vector4d(1, 2, 3, 4),
                                  Eigen::Vector4d(1, 2, 3, 4), X.diagonal());

    auto dual_cone_constraints =
        prog.AddScaledDiagonallyDominantDualConeMatrixConstraint(
            X.cast<symbolic::Expression>());
    for (int r = 0; r < X.rows(); ++r) {
      for (int c = r + 1; c < X.cols(); ++c) {
        if (r != i && c != j) {
          prog.AddLinearEqualityConstraint(X(r, c) == 0);
        }
      }
    }
    prog.AddLinearCost(-X(i, j));
    MathematicalProgramResult result = Solve(prog);
    EXPECT_TRUE(result.is_success());

    Eigen::Matrix4d X_res = result.GetSolution(X);
    Eigen::Matrix4d X_expected = Eigen::Matrix4d::Zero();
    X_expected.diagonal() = Eigen::Vector4d(1, 2, 3, 4);
    X_expected(i, j) = sqrt(X_expected(i, i) * X_expected(j, j));
    EXPECT_TRUE(CompareMatrices(X_res, X_expected, 1e-8));
    EXPECT_EQ(TestInDualConeByGenerators(X_res), -1);
  };

  for (int i = 0; i < 4; ++i) {
    for (int j = j + 1; j < 4; ++j) {
      TestIdx(i, j);
    }
  }
}

}  // namespace solvers
}  // namespace drake
