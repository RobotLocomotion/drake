#include <limits>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/solvers/mathematical_program.h"

using drake::symbolic::test::ExprEqual;

namespace drake {
namespace solvers {
GTEST_TEST(ScaledDiagonallyDominantMatrixTest, AddConstraint) {
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables<4>();

  auto M = prog.AddScaledDiagonallyDominantMatrixConstraint(
      X.cast<symbolic::Expression>());
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      if (i >= j) {
        for (int m = 0; m < 2; ++m) {
          for (int n = 0; n < 2; ++n) {
            EXPECT_EQ(M[i][j](m, n), 0);
          }
        }
      } else {
        EXPECT_PRED2(ExprEqual, M[i][j](0, 1).Expand(),
                     symbolic::Expression(X(i, j)));
      }
    }
  }
}

void CheckSDDMatrix(const Eigen::Ref<const Eigen::MatrixXd>& X_val,
                    bool is_sdd) {
  // Test if a sdd matrix satisfies the constraint.
  const int nx = X_val.rows();
  DRAKE_DEMAND(X_val.cols() == nx);
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables(nx);
  auto M = prog.AddScaledDiagonallyDominantMatrixConstraint(
      X.cast<symbolic::Expression>());

  for (int i = 0; i < nx; ++i) {
    prog.AddBoundingBoxConstraint(X_val.col(i), X_val.col(i), X.col(i));
  }

  const auto result = prog.Solve();
  if (is_sdd) {
    EXPECT_EQ(result, SolutionResult::kSolutionFound);
  } else {
    EXPECT_TRUE(result == SolutionResult::kInfeasibleConstraints ||
                result == SolutionResult::kInfeasible_Or_Unbounded);
  }
}

bool IsMatrixSDD(const Eigen::Ref<Eigen::MatrixXd>& X) {
  // A matrix X is scaled diagonally dominant, if there exists a positive vector
  // d, such that the matrix A defined as A(i, j) = d(j) * X(i, j) is diagonally
  // dominant with positive diagonals.
  const int nx = X.rows();
  MathematicalProgram prog;
  auto d = prog.NewContinuousVariables(nx);
  MatrixX<symbolic::Expression> A(nx, nx);
  for (int i = 0; i < nx; ++i) {
    for (int j = 0; j < nx; ++j) {
      A(i, j) = d(j) * X(i, j);
    }
  }
  prog.AddPositiveDiagonallyDominantMatrixConstraint(A);
  prog.AddBoundingBoxConstraint(1, std::numeric_limits<double>::infinity(), d);

  const auto result = prog.Solve();
  return result == solvers::SolutionResult::kSolutionFound;
}

GTEST_TEST(ScaledDiagonallyDominantMatrixTest, TestSDDMatrix) {
  Eigen::Matrix4d dd_X;
  // A diagonally dominant matrix.
  // clang-format off
  dd_X << 1, -0.2, 0.3, -0.45,
           -0.2, 2, 0.5, 1,
           0.3, 0.5, 3, 2,
           -0.45, 1, 2, 4;
  // clang-format on
  CheckSDDMatrix(dd_X, true);

  Eigen::Matrix4d D = Eigen::Vector4d(1, 2, 3, 4).asDiagonal();
  Eigen::Matrix4d sdd_X = D * dd_X * D;
  CheckSDDMatrix(sdd_X, true);

  D = Eigen::Vector4d(0.2, -1, -0.5, 1.2).asDiagonal();
  sdd_X = D * dd_X * D;
  CheckSDDMatrix(sdd_X, true);

  // not_dd_X is not diagonally dominant (dd), but is scaled diagonally
  // dominant.
  Eigen::Matrix4d not_dd_X;
  not_dd_X << 1, -0.2, 0.3, -0.55, -0.2, 2, 0.5, 1, 0.3, 0.5, 3, 2, -0.55, 1, 2,
      4;
  DRAKE_DEMAND(IsMatrixSDD(not_dd_X));
  CheckSDDMatrix(not_dd_X, true);
}

GTEST_TEST(ScaledDiagonallyDominantMatrixTest, TestNotSDDMatrix) {
  Eigen::Matrix4d not_sdd_X;
  // Not a diagonally dominant matrix.
  // clang-format off
  not_sdd_X << 1, -0.2, 0.3, -1.55,
               -0.2, 2, 0.5, 1,
               0.3, 0.5, 3, 2,
               -1.55, 1, 2, 4;
  // clang-format on
  DRAKE_DEMAND(!IsMatrixSDD(not_sdd_X));
  CheckSDDMatrix(not_sdd_X, false);

  Eigen::Matrix4d D = Eigen::Vector4d(1, 2, 3, 4).asDiagonal();
  not_sdd_X = D * not_sdd_X * D;
  CheckSDDMatrix(not_sdd_X, false);

  D = Eigen::Vector4d(0.2, -1, -0.5, 1.2).asDiagonal();
  not_sdd_X = D * not_sdd_X * D;
  CheckSDDMatrix(not_sdd_X, false);
}
}  // namespace solvers
}  // namespace drake
