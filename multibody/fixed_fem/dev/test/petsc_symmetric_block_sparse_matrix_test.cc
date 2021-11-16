#include "drake/multibody/fixed_fem/dev/petsc_symmetric_block_sparse_matrix.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace {

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::unique_ptr;
using std::vector;

constexpr double kEps = 1e-13;
// clang-format off
const Matrix3d A00 =
    (Eigen::Matrix3d() << -1, 2, 3,
                          2, -5, 6,
                          3, 6, -19).finished();
const Matrix3d A11 =
    (Eigen::Matrix3d() << 11, 12, 13,
                          12, 15, 16,
                          13, 16, 19).finished();
const Matrix3d A12 =
    (Eigen::Matrix3d() << 11, 12, 13,
                          14, 15, 16,
                          17, 18, 19).finished();
const Matrix3d A22 =
    (Eigen::Matrix3d() << 21, 22, 23,
                          22, 25, 26,
                          23, 26, 29).finished();
// clang-format on

/* Makes a dense matrix
   A =   A00 |  0  |  0
        -----------------
          0  | A11 | A12
        -----------------
          0  | A21 | A22
where A21 = A12.transpose(). */
MatrixXd MakeEigenDenseMatrix() {
  MatrixXd A = MatrixXd::Zero(9, 9);
  A.block<3, 3>(0, 0) = A00;
  A.block<3, 3>(3, 3) = A11;
  A.block<3, 3>(3, 6) = A12;
  A.block<3, 3>(6, 3) = A12.transpose();
  A.block<3, 3>(6, 6) = A22;
  return A;
}

/* Makes a PETSc block sparse matrix
   A =   A00 |  0  |  0
        -----------------
          0  | A11 | A12
        -----------------
          0  | A21 | A22
where A21 = A12.transpose(). */
unique_ptr<PetscSymmetricBlockSparseMatrix> MakeBlockSparseMatrix() {
  /* Number of nonzero blocks (on upper triangular portion + diagonal) per block
   * row. */
  const vector<int> sparsity_pattern = {1, 2, 1};
  auto A =
      std::make_unique<PetscSymmetricBlockSparseMatrix>(9, 3, sparsity_pattern);
  VectorX<int> block_indices;
  MatrixXd block;

  block_indices.resize(1);
  block_indices(0) = 0;
  block = A00;
  A->AddToBlock(block_indices, block);

  block_indices.resize(2);
  block_indices(0) = 1;
  block_indices(1) = 2;
  block.resize(6, 6);
  block.topLeftCorner<3, 3>() = A11;
  block.topRightCorner<3, 3>() = A12;
  block.bottomLeftCorner<3, 3>() = A12.transpose();
  block.bottomRightCorner<3, 3>() = A22;
  A->AddToBlock(block_indices, block);
  return A;
}

VectorXd MakeVector() {
  return (Eigen::VectorXd(9) << 1, 1, 2, 3, 5, 8, 13, 21, 34).finished();
}

/* Tests AddToBlock() and SetZero(). */
GTEST_TEST(PetscSymmetricBlockSparseMatrixTest, Construction) {
  unique_ptr<PetscSymmetricBlockSparseMatrix> A = MakeBlockSparseMatrix();
  MatrixXd A_dense = A->MakeDenseMatrix();
  const MatrixXd A_expected = MakeEigenDenseMatrix();
  EXPECT_TRUE(CompareMatrices(A_dense, A_expected, kEps));
  A->SetZero();
  A_dense = A->MakeDenseMatrix();
  EXPECT_TRUE(CompareMatrices(A_dense, MatrixXd::Zero(9, 9), kEps));
}

/* Verifies that SetZero() can be called without any calls to AddToBlock(). */
GTEST_TEST(PetscSymmetricBlockSparseMatrixTest, SetZero) {
  PetscSymmetricBlockSparseMatrix A(3, 3, {0});
  A.SetZero();
  MatrixXd A_dense = A.MakeDenseMatrix();
  EXPECT_EQ(A_dense, Matrix3d::Zero());
}

GTEST_TEST(PetscSymmetricBlockSparseMatrixTest, Solve) {
  unique_ptr<PetscSymmetricBlockSparseMatrix> A = MakeBlockSparseMatrix();
  const MatrixXd A_eigen = MakeEigenDenseMatrix();
  const VectorXd b = MakeVector();
  const VectorXd x_expected = A_eigen.lu().solve(b);
  const VectorXd x = A->Solve(
      PetscSymmetricBlockSparseMatrix::SolverType::kMINRES,
      PetscSymmetricBlockSparseMatrix::PreconditionerType::kBlockJacobi, b);
  EXPECT_TRUE(CompareMatrices(x, x_expected, kEps));
}

GTEST_TEST(PetscSymmetricBlockSparseMatrixTest, ZeroRowsAndColumns) {
  unique_ptr<PetscSymmetricBlockSparseMatrix> A = MakeBlockSparseMatrix();
  const vector<int> indexes = {1, 2, 5};
  constexpr double kDiagnoalValue = 3;
  A->ZeroRowsAndColumns(indexes, kDiagnoalValue);

  MatrixXd A_eigen = MakeEigenDenseMatrix();
  for (int i : indexes) {
    A_eigen.row(i).setZero();
    A_eigen.col(i).setZero();
    A_eigen(i, i) = kDiagnoalValue;
  }
  EXPECT_TRUE(CompareMatrices(A->MakeDenseMatrix(), A_eigen, kEps));
}

GTEST_TEST(PetscSymmetricBlockSparseMatrixTest, Clone) {
  unique_ptr<PetscSymmetricBlockSparseMatrix> A = MakeBlockSparseMatrix();
  unique_ptr<PetscSymmetricBlockSparseMatrix> A_clone = A->Clone();
  EXPECT_EQ(A->MakeDenseMatrix(), A_clone->MakeDenseMatrix());
  const VectorXd b = MakeVector();
  const VectorXd x = A->Solve(
      PetscSymmetricBlockSparseMatrix::SolverType::kMINRES,
      PetscSymmetricBlockSparseMatrix::PreconditionerType::kBlockJacobi, b);
  const VectorXd x_clone = A_clone->Solve(
      PetscSymmetricBlockSparseMatrix::SolverType::kMINRES,
      PetscSymmetricBlockSparseMatrix::PreconditionerType::kBlockJacobi, b);
  EXPECT_TRUE(CompareMatrices(x, x_clone, kEps));
}

}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
