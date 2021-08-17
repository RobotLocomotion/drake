#include "drake/multibody/fixed_fem/dev/matrix_utilities.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/fixed_fem/dev/test/test_utilities.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace {

using Eigen::MatrixXd;

/* Returns an arbitrary matrix for testing purpose. */
MatrixXd MakeMatrix(int rows, int cols) {
  MatrixXd A(rows, cols);
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      A(i, j) = cols * i + j;
    }
  }
  return A;
}

MatrixX<AutoDiffXd> MakeAutoDiffMatrix(int rows, int cols) {
  const MatrixXd A = MakeMatrix(rows, cols);
  MatrixX<AutoDiffXd> A_ad(rows, cols);
  math::initializeAutoDiff(A, A_ad);
  return A_ad;
}

const double kTol = 1e-14;

/* Calculates the absolute tolerance kTol scaled by the condition number of the
 input matrix A. */
double CalcTolerance(const Matrix3<double>& A) {
  return test::CalcConditionNumber<double>(A) * kTol;
}

GTEST_TEST(MatrixUtilitiesTest, PolarDecompose) {
  const Matrix3<double> A = MakeMatrix(3, 3);
  Matrix3<double> R, S;
  PolarDecompose<double>(A, &R, &S);
  /* Tests reconstruction. */
  EXPECT_TRUE(CompareMatrices(A, R * S, CalcTolerance(A)));
  /* Tests symmetry of S. */
  EXPECT_TRUE(CompareMatrices(S, S.transpose(), kTol));
  /* Tests R is a rotation matrix. */
  EXPECT_TRUE(math::RotationMatrix<double>::IsValid(R, kTol));
}

GTEST_TEST(MatrixUtilitiesTest, AddScaledRotationalDerivative) {
  const Matrix3<AutoDiffXd> F = MakeAutoDiffMatrix(3, 3);
  Matrix3<AutoDiffXd> R, S;
  PolarDecompose<AutoDiffXd>(F, &R, &S);
  Eigen::Matrix<AutoDiffXd, 9, 9> scaled_dRdF =
      Eigen::Matrix<AutoDiffXd, 9, 9>::Zero();
  AutoDiffXd scale = 1.23;
  AddScaledRotationalDerivative<AutoDiffXd>(R, S, scale, &scaled_dRdF);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      Matrix3<double> scaled_dRijdF;
      for (int k = 0; k < kSpaceDimension; ++k) {
        for (int l = 0; l < kSpaceDimension; ++l) {
          scaled_dRijdF(k, l) = scaled_dRdF(3 * j + i, 3 * l + k).value();
        }
      }
      EXPECT_TRUE(
          CompareMatrices(scale * Eigen::Map<const Matrix3<double>>(
                                      R(i, j).derivatives().data(), 3, 3),
                          scaled_dRijdF, CalcTolerance(scaled_dRijdF)));
    }
  }
}

GTEST_TEST(MatrixUtilitiesTest, CalcCofactorMatrix) {
  const Matrix3<double> A = MakeMatrix(3, 3);
  Matrix3<double> C;
  CalcCofactorMatrix<double>(A, &C);
  EXPECT_TRUE(CompareMatrices(
      A.inverse(), 1.0 / A.determinant() * C.transpose(), CalcTolerance(A)));
}

GTEST_TEST(MatrixUtilitiesTest, AddScaledCofactorMatrixDerivative) {
  const Matrix3<AutoDiffXd> A = MakeAutoDiffMatrix(3, 3);
  Matrix3<AutoDiffXd> C;
  CalcCofactorMatrix<AutoDiffXd>(A, &C);
  Eigen::Matrix<AutoDiffXd, 9, 9> scaled_dCdA =
      Eigen::Matrix<AutoDiffXd, 9, 9>::Zero();
  AutoDiffXd scale = 1.23;
  AddScaledCofactorMatrixDerivative<AutoDiffXd>(A, scale, &scaled_dCdA);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      Matrix3<double> scaled_dCijdA;
      for (int k = 0; k < kSpaceDimension; ++k) {
        for (int l = 0; l < kSpaceDimension; ++l) {
          scaled_dCijdA(k, l) = scaled_dCdA(3 * j + i, 3 * l + k).value();
        }
      }
      EXPECT_TRUE(
          CompareMatrices(scale * Eigen::Map<const Matrix3<double>>(
                                      C(i, j).derivatives().data(), 3, 3),
                          scaled_dCijdA, CalcTolerance(scaled_dCijdA)));
    }
  }
}

GTEST_TEST(MatrixUtilitiesTest, PermuteBlockVector) {
  constexpr int kNumBlocks = 3;
  VectorX<double> v(3 * kNumBlocks);
  v << 0, 1, 2, 3, 4, 5, 6, 7, 8;
  const std::vector<int> block_permutation = {1, 2, 0};
  const VectorX<double> permuted_v =
      PermuteBlockVector<double>(v, block_permutation);
  VectorX<double> expected_result(3 * kNumBlocks);
  expected_result << 6, 7, 8, 0, 1, 2, 3, 4, 5;
  EXPECT_TRUE(CompareMatrices(expected_result, permuted_v));
}

/* Verify that PermuteBlockSparseMatrix provides the expected answer as
hand-calculated solution on a small problem. The problem consists of 2 blocks
permuted such that 0->1, 1->0. */
GTEST_TEST(PermuteBlockSparseMatrix, AnalyticTest) {
  constexpr int kNumBlocks = 2;
  constexpr int kSize = kNumBlocks * 3;
  const MatrixXd A = MakeMatrix(kSize, kSize);
  const Eigen::SparseMatrix<double> A_sparse = A.sparseView();
  const std::vector<int> block_permutation = {1, 0};
  const Eigen::SparseMatrix<double> permuted_A =
      PermuteBlockSparseMatrix(A_sparse, block_permutation);
  const MatrixXd permuted_A_dense = permuted_A;

  MatrixXd expected_result(kSize, kSize);
  /* The new 0-0 block is the same as the old 1-1 block. */
  expected_result.topLeftCorner<3, 3>() = A.bottomRightCorner<3, 3>();
  /* The new 0-1 block is the same as the old 1-0 block. */
  expected_result.topRightCorner<3, 3>() = A.bottomLeftCorner<3, 3>();
  /* The new 1-0 block is the same as the old 0-1 block. */
  expected_result.bottomLeftCorner<3, 3>() = A.topRightCorner<3, 3>();
  /* The new 1-1 block is the same as the old 0-0 block. */
  expected_result.bottomRightCorner<3, 3>() = A.topLeftCorner<3, 3>();
  EXPECT_TRUE(CompareMatrices(expected_result, permuted_A_dense));
}

}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
