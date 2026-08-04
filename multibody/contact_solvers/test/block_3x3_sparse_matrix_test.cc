#include "drake/multibody/contact_solvers/block_3x3_sparse_matrix.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace {

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/* Returns an arbitrary non-zero matrix of size m-by-n.*/
MatrixXd MakeArbitraryMatrix(int m, int n) {
  MatrixXd A(m, n);
  for (int i = 0; i < m; ++i) {
    for (int j = 0; j < n; ++j) {
      A(i, j) = 3 * i + 4 * j;
    }
  }
  return A;
}

/* Returns a dummy 3x3 matrix with all entries in the matrix equal to the given
 value. */
Matrix3d MakeMatrix(double value) {
  return value * Matrix3d::Ones();
}

/* Returns an arbitrary Block3x3SparseMatrix with size 12-by-9. */
Block3x3SparseMatrix<double> MakeBlockSparseMatrix() {
  Block3x3SparseMatrix<double> sparse_matrix(4, 3);
  std::vector<Block3x3SparseMatrix<double>::Triplet> triplets;
  triplets.emplace_back(0, 0, MakeMatrix(1.0));
  triplets.emplace_back(0, 0, MakeMatrix(2.0));
  triplets.emplace_back(0, 1, MakeMatrix(3.0));
  triplets.emplace_back(0, 1, MakeMatrix(4.0));
  triplets.emplace_back(2, 1, MakeMatrix(5.0));
  triplets.emplace_back(3, 2, MakeMatrix(6.0));
  sparse_matrix.SetFromTriplets(triplets);
  EXPECT_EQ(sparse_matrix.num_blocks(), 4);
  return sparse_matrix;
}

GTEST_TEST(Block3x3SparseMatrixTest, Size) {
  const Block3x3SparseMatrix<double> sparse_matrix(4, 3);
  EXPECT_EQ(sparse_matrix.rows(), 12);
  EXPECT_EQ(sparse_matrix.cols(), 9);
  EXPECT_EQ(sparse_matrix.block_rows(), 4);
  EXPECT_EQ(sparse_matrix.block_cols(), 3);
}

GTEST_TEST(Block3x3SparseMatrixTest, SetFromTriplets) {
  Block3x3SparseMatrix<double> sparse_matrix = MakeBlockSparseMatrix();
  MatrixXd expected_matrix = MatrixXd::Zero(12, 9);
  expected_matrix.topLeftCorner<3, 3>() = MakeMatrix(3.0);
  expected_matrix.block<3, 3>(0, 3) = MakeMatrix(7.0);
  expected_matrix.block<3, 3>(6, 3) = MakeMatrix(5.0);
  expected_matrix.bottomRightCorner<3, 3>() = MakeMatrix(6.0);
  EXPECT_EQ(sparse_matrix.MakeDenseMatrix(), expected_matrix);

  /* Verify that setting the sparse matrix with a new vector of triplets resets
   the matrix. */
  std::vector<Block3x3SparseMatrix<double>::Triplet> triplets;
  triplets.emplace_back(2, 2, MakeMatrix(4.0));
  sparse_matrix.SetFromTriplets(triplets);
  expected_matrix.setZero();
  expected_matrix.block<3, 3>(6, 6) = MakeMatrix(4.0);
  EXPECT_EQ(sparse_matrix.MakeDenseMatrix(), expected_matrix);
  EXPECT_EQ(sparse_matrix.num_blocks(), 1);
}

GTEST_TEST(Block3x3SparseMatrixTest, MultiplyAndAddTo) {
  const MatrixXd A = MakeArbitraryMatrix(9, 7);
  const Block3x3SparseMatrix<double> sparse_matrix = MakeBlockSparseMatrix();
  const MatrixXd dense_matrix = sparse_matrix.MakeDenseMatrix();

  /* Set the destinations to compatible-sized non-zero matrices. */
  MatrixXd y1 = MakeArbitraryMatrix(12, 7);
  MatrixXd y2 = y1;

  sparse_matrix.MultiplyAndAddTo(A, &y1);
  y2 += dense_matrix * A;
  EXPECT_TRUE(CompareMatrices(y1, y2));
}

GTEST_TEST(Block3x3SparseMatrixTest, LeftMultiplyAndAddTo) {
  const Block3x3SparseMatrix<double> sparse_matrix = MakeBlockSparseMatrix();
  const MatrixXd dense_matrix = sparse_matrix.MakeDenseMatrix();

  const MatrixXd A = MakeArbitraryMatrix(7, 12);

  /* Set the destinations to compatible-sized non-zero matrices. */
  MatrixXd y1 = MakeArbitraryMatrix(7, 9);
  MatrixXd y2 = y1;

  sparse_matrix.LeftMultiplyAndAddTo(A, &y1);
  y2 += A * dense_matrix;
  EXPECT_TRUE(CompareMatrices(y1, y2));
}

GTEST_TEST(Block3x3SparseMatrixTest, TransposeAndMultiplyAndAddTo) {
  const Block3x3SparseMatrix<double> sparse_matrix = MakeBlockSparseMatrix();
  const MatrixXd dense_matrix = sparse_matrix.MakeDenseMatrix();

  const MatrixXd A = MakeArbitraryMatrix(12, 6);

  /* Test the sparse-dense multiplication. */
  /* Set the destinations to compatible-sized non-zero matrices. */
  MatrixXd y1 = MakeArbitraryMatrix(9, 6);
  MatrixXd y2 = y1;

  sparse_matrix.TransposeAndMultiplyAndAddTo(A, &y1);
  y2 += dense_matrix.transpose() * A;
  EXPECT_TRUE(CompareMatrices(y1, y2));

  /* Test the sparse-sparse multiplication. */
  Block3x3SparseMatrix<double> A_sparse(4, 2);
  std::vector<Block3x3SparseMatrix<double>::Triplet> triplets;
  triplets.emplace_back(0, 1, MakeMatrix(4.0));
  triplets.emplace_back(2, 1, MakeMatrix(5.0));
  A_sparse.SetFromTriplets(triplets);

  sparse_matrix.TransposeAndMultiplyAndAddTo(A_sparse, &y1);
  y2 += dense_matrix.transpose() * A_sparse.MakeDenseMatrix();
  EXPECT_TRUE(CompareMatrices(y1, y2));
}

GTEST_TEST(Block3x3SparseMatrixTest, MultiplyWithScaledTransposeAndAddTo) {
  const Block3x3SparseMatrix<double> sparse_matrix = MakeBlockSparseMatrix();
  const MatrixXd dense_matrix = sparse_matrix.MakeDenseMatrix();
  const VectorXd scale = VectorXd::LinSpaced(9, 0.0, 1.0);

  /* Set the destinations to compatible-sized non-zero matrices. */
  MatrixXd y1 = MakeArbitraryMatrix(12, 12);
  MatrixXd y2 = y1;

  sparse_matrix.MultiplyWithScaledTransposeAndAddTo(scale, &y1);
  y2 += dense_matrix * scale.asDiagonal() * dense_matrix.transpose();
  EXPECT_TRUE(CompareMatrices(y1, y2));
}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
