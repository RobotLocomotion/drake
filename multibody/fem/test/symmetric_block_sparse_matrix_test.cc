#include "drake/multibody/fem/symmetric_block_sparse_matrix.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace {

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using std::vector;

constexpr double kEps = 4.0 * std::numeric_limits<double>::epsilon();
// clang-format off
const Matrix2d A00 = (Eigen::Matrix2d() << 11, 12,
                                           12, 15).finished();
const Matrix3d A11 = (Eigen::Matrix3d() << 1, 2, 3,
                                           2, 5, 6,
                                           3, 6, 9).finished();
const Matrix4d A22 = (Eigen::Matrix4d() << 11, 12, 13, 13,
                                           14, 15, 16, 16,
                                           17, 18, 19, 19,
                                           20, 21, 22, 23).finished();
const Eigen::Matrix<double, 4, 3> A21 = (Eigen::Matrix<double, 4, 3>() << 11, 22, 33,
                                                                          33, 44, 55,
                                                                          66, 11, 88,
                                                                          22, 56, 78).finished();
// clang-format on

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

/* Makes a dense matrix
   A =   A00 |  0  |  0
        -----------------
          0  | A11 | A12
        -----------------
          0  | A21 | A22
where A12 = A21.transpose(). */
MatrixXd MakeDenseMatrix() {
  MatrixXd A(9, 9);
  A.block<2, 2>(0, 0) = A00;
  A.block<3, 3>(2, 2) = A11;
  A.block<4, 3>(5, 2) = A21;
  A.block<3, 4>(2, 5) = A21.transpose();
  A.block<4, 4>(5, 5) = A22;
  return A;
}

/* Makes a block sparse matrix
   A =   A00 |  0  |  0
        -----------------
          0  | A11 | A12
        -----------------
          0  | A21 | A22
where A21 = A12.transpose(). */
SymmetricBlockSparseMatrix<double> MakeBlockSparseMatrix() {
  vector<int> diag{2, 3, 4};
  vector<vector<int>> sparsity;
  sparsity.push_back({{0}});
  sparsity.push_back({{1, 2}});
  sparsity.push_back({{2}});
  SymmetricBlockSparseMatrix<double> A_blocks({diag, sparsity});
  A_blocks.SetBlock(0, 0, A00);
  A_blocks.SetBlock(1, 1, A11);
  A_blocks.SetBlock(2, 1, A21);
  A_blocks.SetBlock(2, 2, A22);
  return A_blocks;
}

GTEST_TEST(SymmetricBlockSparseMatrixTest, Construction) {
  const MatrixXd A = MakeDenseMatrix();
  const SymmetricBlockSparseMatrix<double> A_blocks = MakeBlockSparseMatrix();
  EXPECT_TRUE(CompareMatrices(A_blocks.MakeDenseMatrix(), A));
}

GTEST_TEST(SymmetricBlockSparseMatrixTest, SetZero) {
  SymmetricBlockSparseMatrix<double> A_blocks = MakeBlockSparseMatrix();
  A_blocks.SetZero();
  EXPECT_TRUE(
      CompareMatrices(A_blocks.MakeDenseMatrix(), MatrixXd::Zero(9, 9)));
}

GTEST_TEST(SymmetricBlockSparseMatrixTest, SubtractProductFromBlock) {
  MatrixXd A = MakeDenseMatrix();
  SymmetricBlockSparseMatrix<double> A_blocks = MakeBlockSparseMatrix();
  const MatrixXd M1 = MakeArbitraryMatrix(4, 5);
  const MatrixXd M2 = MakeArbitraryMatrix(3, 5);
  A_blocks.SubtractProductFromBlock(2, 1, M1, M2);
  A.block<4, 3>(5, 2) -= M1 * M2.transpose();
  A.block<3, 4>(2, 5) -= M2 * M1.transpose();
  EXPECT_TRUE(CompareMatrices(A_blocks.MakeDenseMatrix(), A));
}

// GTEST_TEST(SymmetricBlockSparseMatrixTest, CalcAdjacencyGrpah) {
//   const SymmetricBlockSparseMatrix<double> A_blocks =
//   MakeBlockSparseMatrix(); const std::vector<std::set<int>> adj =
//   A_blocks.CalcAdjacencyGraph(); ASSERT_EQ(adj.size(), 3);

//   EXPECT_EQ(adj[0].size(), 1);
//   EXPECT_EQ(adj[0].count(0), 1);

//   EXPECT_EQ(adj[1].size(), 2);
//   EXPECT_EQ(adj[1].count(1), 1);
//   EXPECT_EQ(adj[1].count(2), 1);

//   EXPECT_EQ(adj[2].size(), 1);
//   EXPECT_EQ(adj[2].count(2), 1);
// }

}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
