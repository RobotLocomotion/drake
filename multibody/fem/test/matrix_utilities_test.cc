#include "drake/multibody/fem/matrix_utilities.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rotation_matrix.h"

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
      A(i, j) = cols * i + j + 1.0;
    }
  }
  return A;
}

MatrixX<AutoDiffXd> MakeAutoDiffMatrix(int rows, int cols) {
  const MatrixXd A = MakeMatrix(rows, cols);
  MatrixX<AutoDiffXd> A_ad(rows, cols);
  math::InitializeAutoDiff(A, &A_ad);
  return A_ad;
}

const double kTol = 1e-14;

/* Calculates the absolute tolerance kTol scaled by the condition number of the
 input matrix A. */
template <typename T>
double CalcTolerance(const Matrix3<T>& A) {
  return CalcConditionNumberOfInvertibleMatrix<T>(A) * kTol;
}

GTEST_TEST(MatrixUtilitiesTest, CalcConditionNumber) {
  const Matrix3<double> A = Vector3<double>(1, 2, 3).asDiagonal();
  EXPECT_DOUBLE_EQ(CalcConditionNumberOfInvertibleMatrix<double>(A), 3);

  /* The input must be square. */
  const MatrixX<double> B(1, 2);
  EXPECT_THROW(CalcConditionNumberOfInvertibleMatrix<double>(B),
               std::exception);
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

GTEST_TEST(MatrixUtilitiesTest, RotationSvd) {
  /* Lambda to exercise one F ⇒ (U, σ, V) decomposition and check all
   properties. */
  const auto check_svd = [&](const Matrix3<double>& F) {
    Matrix3<double> U, V;
    Vector3<double> sigma;
    RotationSvd<double>(F, &U, &V, &sigma);

    /* 1) Reconstruction: F ≈ U S Vᵀ */
    EXPECT_TRUE(
        CompareMatrices(F, U * sigma.asDiagonal() * V.transpose(), kTol));

    /* 2) Orthonormality: UᵀU = I, VᵀV = I */
    EXPECT_TRUE(
        CompareMatrices(Matrix3<double>::Identity(), U.transpose() * U, kTol));
    EXPECT_TRUE(
        CompareMatrices(Matrix3<double>::Identity(), V.transpose() * V, kTol));

    /* 3) Proper rotations: det > 0 */
    EXPECT_GT(U.determinant(), 0.0);
    EXPECT_GT(V.determinant(), 0.0);
  };

  {
    check_svd(Matrix3<double>::Identity());
  }

  /* Pure scaling */
  {
    Matrix3<double> F = Matrix3<double>::Zero();
    F.diagonal() << 3.0, 2.0, -1.0;
    check_svd(F);
  }

  /* General rotation–scale–rotation */
  {
    math::RotationMatrix<double> R1(math::RollPitchYaw<double>(1, 2, 3));
    math::RotationMatrix<double> R2(math::RollPitchYaw<double>(4, 5, 6));
    Matrix3<double> S = Matrix3<double>::Zero();
    /* We choose F so that a standard SVD (with all non-negative singular
     values) produce a non-rotation U and V. */
    S.diagonal() << 5.0, -4.0, -2.0;

    Matrix3<double> F = R1.matrix() * S * R2.matrix().transpose();
    check_svd(F);

    Eigen::JacobiSVD<Matrix3<double>> svd(
        F, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Matrix3<double> U = svd.matrixU();
    const Matrix3<double> V = svd.matrixV();
    EXPECT_TRUE(U.determinant() < 0 || V.determinant() < 0);
  }
}

GTEST_TEST(MatrixUtilitiesTest, AddScaledRotationalDerivative) {
  const Matrix3<AutoDiffXd> F = MakeAutoDiffMatrix(3, 3);
  Matrix3<AutoDiffXd> R, S;
  PolarDecompose<AutoDiffXd>(F, &R, &S);
  math::internal::FourthOrderTensor<AutoDiffXd> scaled_dRdF;
  AutoDiffXd scale = 1.23;
  AddScaledRotationalDerivative<AutoDiffXd>(R, S, scale, &scaled_dRdF);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      Matrix3<double> scaled_dRijdF;
      for (int k = 0; k < 3; ++k) {
        for (int l = 0; l < 3; ++l) {
          scaled_dRijdF(k, l) = scaled_dRdF(i, j, k, l).value();
        }
      }
      EXPECT_TRUE(
          CompareMatrices(scale * Eigen::Map<const Matrix3<double>>(
                                      R(i, j).derivatives().data(), 3, 3),
                          scaled_dRijdF, CalcTolerance(R)));
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
  math::internal::FourthOrderTensor<AutoDiffXd> scaled_dCdA;
  AutoDiffXd scale = 1.23;
  AddScaledCofactorMatrixDerivative<AutoDiffXd>(A, scale, &scaled_dCdA);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      Matrix3<double> scaled_dCijdA;
      for (int k = 0; k < 3; ++k) {
        for (int l = 0; l < 3; ++l) {
          scaled_dCijdA(k, l) = scaled_dCdA(i, j, k, l).value();
        }
      }
      EXPECT_TRUE(
          CompareMatrices(scale * Eigen::Map<const Matrix3<double>>(
                                      C(i, j).derivatives().data(), 3, 3),
                          scaled_dCijdA, CalcTolerance(A)));
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

}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
