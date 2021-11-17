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

constexpr double kEps = 4.0 * std::numeric_limits<double>::epsilon();
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
MatrixXd MakeDenseMatrix() {
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
  const vector<int> sparsity_pattern = {1, 2, 2};
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

GTEST_TEST(PetscSymmetricBlockSparseMatrixTest, Construction) {
  unique_ptr<PetscSymmetricBlockSparseMatrix> A = MakeBlockSparseMatrix();
  MatrixXd A_dense = A->MakeDenseMatrix();
  const MatrixXd A_expected = MakeDenseMatrix();
  EXPECT_TRUE(CompareMatrices(A_dense, A_expected, kEps));
  A->SetZero();
  A_dense = A->MakeDenseMatrix();
  EXPECT_TRUE(CompareMatrices(A_dense, MatrixXd::Zero(9, 9), kEps));
}

GTEST_TEST(PetscSymmetricBlockSparseMatrixTest, Solve) {
  unique_ptr<PetscSymmetricBlockSparseMatrix> A = MakeBlockSparseMatrix();
  const MatrixXd A_eigen = MakeDenseMatrix();
  const VectorXd b = MakeVector();
  const VectorXd x_expected = A_eigen.lu().solve(b);
  const VectorXd x = A->Solve(
      PetscSymmetricBlockSparseMatrix::SolverType::MINRES,
      PetscSymmetricBlockSparseMatrix::PreconditionerType::BlockJacobi, b);
  EXPECT_TRUE(CompareMatrices(x, x_expected, 1e-13));
}

}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake