#include "drake/multibody/fixed_fem/dev/petsc_symmetric_block_sparse_matrix.h"

#include <memory>
#include <thread>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"

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

constexpr double kEps = 2e-13;
// Size of the matrix in this test.
constexpr int kDofs = 9;

// clang-format off
const Matrix3d A00 =
    (Eigen::Matrix3d() << -1, 2, 3,
                          2, -5, 6,
                          3, 6, -19).finished();
const Matrix3d A11 =
    (Eigen::Matrix3d() << 11, 12, 13,
                          12, 15, 16,
                          13, 16, 19).finished();
const Matrix3d A02 =
    (Eigen::Matrix3d() << 11, 12, 13,
                          14, 15, 16,
                          17, 18, 19).finished();
const Matrix3d A22 =
    (Eigen::Matrix3d() << 21, 22, 23,
                          22, 25, 26,
                          23, 26, 29).finished();
// clang-format on

/* Makes a dense matrix
   A =   A00 |  0  | A02
        -----------------
          0  | A11 |  0
        -----------------
         A20 |  0  | A22
where A21 = A02.transpose(). */
MatrixXd MakeEigenDenseMatrix() {
  MatrixXd A = MatrixXd::Zero(9, 9);
  A.block<3, 3>(0, 0) = A00;
  A.block<3, 3>(3, 3) = A11;
  A.block<3, 3>(0, 6) = A02;
  A.block<3, 3>(6, 0) = A02.transpose();
  A.block<3, 3>(6, 6) = A22;
  return A;
}

/* Makes a PETSc block sparse matrix
   A =   A00 |  0  | A02
        -----------------
          0  | A11 |  0
        -----------------
         A02 |  0  | A22
where A21 = A02.transpose(). */
unique_ptr<PetscSymmetricBlockSparseMatrix> MakeBlockSparseMatrix() {
  /* Number of nonzero blocks (on upper triangular portion + diagonal) per block
   row. */
  const vector<int> num_upper_triangular_blocks_per_row = {2, 1, 1};
  auto A = std::make_unique<PetscSymmetricBlockSparseMatrix>(
      9, 3, num_upper_triangular_blocks_per_row);
  VectorX<int> block_indices;
  MatrixXd block;

  block_indices.resize(1);
  block_indices(0) = 1;
  block = A11;
  A->AddToBlock(block_indices, block);

  block_indices.resize(2);
  block_indices(0) = 0;
  block_indices(1) = 2;
  block.resize(6, 6);
  block.topLeftCorner<3, 3>() = A00;
  block.topRightCorner<3, 3>() = A02;
  block.bottomLeftCorner<3, 3>() = A02.transpose();
  block.bottomRightCorner<3, 3>() = A22;
  A->AddToBlock(block_indices, block);
  return A;
}

VectorXd MakeVector9d() {
  return (Eigen::VectorXd(9) << 1, 1, 2, 3, 5, 8, 13, 21, 34).finished();
}

/* Tests AddToBlock() and SetZero(). */
GTEST_TEST(PetscSymmetricBlockSparseMatrixTest, Construction) {
  unique_ptr<PetscSymmetricBlockSparseMatrix> A = MakeBlockSparseMatrix();
  A->AssembleIfNecessary();
  MatrixXd A_dense = A->MakeDenseMatrix();
  const MatrixXd A_expected = MakeEigenDenseMatrix();
  EXPECT_TRUE(CompareMatrices(A_dense, A_expected, kEps));
  A->SetZero();
  A_dense = A->MakeDenseMatrix();
  EXPECT_EQ(A_dense, MatrixXd::Zero(9, 9));
}

/* Verifies that SetZero() can be called without any calls to AddToBlock(). */
GTEST_TEST(PetscSymmetricBlockSparseMatrixTest, SetZero) {
  PetscSymmetricBlockSparseMatrix A(3, 3, {1});
  A.SetZero();
  MatrixXd A_dense = A.MakeDenseMatrix();
  EXPECT_EQ(A_dense, Matrix3d::Zero());
}

GTEST_TEST(PetscSymmetricBlockSparseMatrixTest, Solve) {
  unique_ptr<PetscSymmetricBlockSparseMatrix> A = MakeBlockSparseMatrix();
  const MatrixXd A_eigen = MakeEigenDenseMatrix();
  const VectorXd b = MakeVector9d();
  const VectorXd x_expected = A_eigen.lu().solve(b);
  DRAKE_EXPECT_THROWS_MESSAGE(
      A->Solve(PetscSymmetricBlockSparseMatrix::SolverType::kMINRES,
               PetscSymmetricBlockSparseMatrix::PreconditionerType::kJacobi, b),
      std::exception,
      "PetscSymmetricBlockSparseMatrix::Solve.*: matrix is not yet "
      "assembled.*");
  A->AssembleIfNecessary();
  const VectorXd x =
      A->Solve(PetscSymmetricBlockSparseMatrix::SolverType::kMINRES,
               PetscSymmetricBlockSparseMatrix::PreconditionerType::kJacobi, b);
  VectorXd x_in_place = b;
  A->SolveInPlace(PetscSymmetricBlockSparseMatrix::SolverType::kMINRES,
                  PetscSymmetricBlockSparseMatrix::PreconditionerType::kJacobi,
                  &x_in_place);
  EXPECT_EQ(x, x_in_place);
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
  EXPECT_EQ(A->MakeDenseMatrix(), A_eigen);
}

GTEST_TEST(PetscSymmetricBlockSparseMatrixTest, Clone) {
  unique_ptr<PetscSymmetricBlockSparseMatrix> A = MakeBlockSparseMatrix();
  DRAKE_EXPECT_THROWS_MESSAGE(A->Clone(),
                              "PetscSymmetricBlockSparseMatrix::Clone.*: "
                              "matrix is not yet assembled.*");
  A->AssembleIfNecessary();
  unique_ptr<PetscSymmetricBlockSparseMatrix> A_clone = A->Clone();
  EXPECT_EQ(A->MakeDenseMatrix(), A_clone->MakeDenseMatrix());
  const VectorXd b = MakeVector9d();
  const VectorXd x =
      A->Solve(PetscSymmetricBlockSparseMatrix::SolverType::kMINRES,
               PetscSymmetricBlockSparseMatrix::PreconditionerType::kJacobi, b);
  const VectorXd x_clone = A_clone->Solve(
      PetscSymmetricBlockSparseMatrix::SolverType::kMINRES,
      PetscSymmetricBlockSparseMatrix::PreconditionerType::kJacobi, b);
  EXPECT_TRUE(CompareMatrices(x, x_clone, kEps));
}
/* Test if we can run several PETSc solves simultaneously on multiple threads.
 We solve the same linear system with multiple right hand sides. The solution
 from using threads should be the same as those from serial. */
GTEST_TEST(PetscSymmetricBlockSparseMatrixTest, MultiThreadTest) {
  /* We first will solve each linear system one at a time, and then solve them
   all in parallel. The two sets of results should match. */
  constexpr int kNumThreads = 100;

  std::vector<VectorXd> single_threaded_results(kNumThreads);
  std::vector<VectorXd> multi_threaded_results(kNumThreads);

  /* Create a functor that solves A*x = b. */
  auto run_solver = [](const VectorXd& b, VectorXd* x) {
    unique_ptr<PetscSymmetricBlockSparseMatrix> A = MakeBlockSparseMatrix();
    A->AssembleIfNecessary();
    *x = A->Solve(PetscSymmetricBlockSparseMatrix::SolverType::kMINRES,
                  PetscSymmetricBlockSparseMatrix::PreconditionerType::kJacobi,
                  b);
  };

  /* Solve without using threads. */
  for (int i = 0; i < kNumThreads; ++i) {
    VectorXd b = VectorXd::Zero(kDofs);
    // Set b to a nonzero vector for a nontrivial solve.
    const int nonzero_entry = i % kDofs;
    b(nonzero_entry) = 1.0;
    run_solver(b, &single_threaded_results[i]);
  }

  /* Solve using threads. */
  std::vector<std::thread> test_threads;
  for (int i = 0; i < kNumThreads; ++i) {
    VectorXd b = VectorXd::Zero(kDofs);
    // Set b to a nonzero vector for a nontrivial solve.
    const int nonzero_entry = i % kDofs;
    b(nonzero_entry) = 1.0;
    test_threads.emplace_back(run_solver, b, &multi_threaded_results[i]);
  }
  for (int i = 0; i < kNumThreads; ++i) {
    test_threads[i].join();
  }

  /* All solutions should be the same. */
  for (int i = 0; i < kNumThreads; ++i) {
    EXPECT_EQ(single_threaded_results[i], multi_threaded_results[i]);
  }
}

GTEST_TEST(PetscSymmetricBlockSparseMatrixTest, CalcSchurComplement) {
  unique_ptr<PetscSymmetricBlockSparseMatrix> M = MakeBlockSparseMatrix();
  const vector<int> D_block_indexes = {1};
  const vector<int> A_block_indexes = {0, 2};
  const MatrixXd M_eigen = MakeEigenDenseMatrix();

  MatrixXd A = MatrixXd::Zero(6, 6);
  A.topLeftCorner<3, 3>() = M_eigen.topLeftCorner<3, 3>();
  A.topRightCorner<3, 3>() = M_eigen.topRightCorner<3, 3>();
  A.bottomLeftCorner<3, 3>() = M_eigen.bottomLeftCorner<3, 3>();
  A.bottomRightCorner<3, 3>() = M_eigen.bottomRightCorner<3, 3>();

  MatrixXd D = M_eigen.block<3, 3>(3, 3);

  MatrixXd B = MatrixXd::Zero(6, 3);
  B.topLeftCorner<3, 3>() = M_eigen.block<3, 3>(3, 0);
  B.bottomLeftCorner<3, 3>() = M_eigen.block<3, 3>(3, 6);

  const SchurComplement schur_complement =
      M->CalcSchurComplement(D_block_indexes, A_block_indexes);
  const MatrixXd schur_complement_matrix = schur_complement.get_D_complement();

  const MatrixXd expected_schur_complement =
      A - B * D.lu().solve(B.transpose());
  EXPECT_TRUE(CompareMatrices(schur_complement_matrix,
                              expected_schur_complement, kEps));

  /* Set arbitrary solution for x in the system
    Ax + By  =  a
    Bᵀx + Dy =  0
   Verify that y is solved to be -D⁻¹Bᵀx. */
  VectorXd x(6);
  x << 1, 2, 3, 4, 5, 6;
  const VectorXd y = schur_complement.SolveForY(x);
  VectorXd expected_y = -1.0 * D.lu().solve(B.transpose()) * x;
  EXPECT_TRUE(CompareMatrices(y, expected_y, kEps));
}

}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
