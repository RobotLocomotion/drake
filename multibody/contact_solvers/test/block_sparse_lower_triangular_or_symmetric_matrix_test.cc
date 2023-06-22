#include "drake/multibody/contact_solvers/block_sparse_lower_triangular_or_symmetric_matrix.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace {

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;

// clang-format off
const Matrix2d A00 = (Eigen::Matrix2d() << 11, 12,
                                           12, 15).finished();
const Matrix3d A11 = (Eigen::Matrix3d() << 1, 2, 3,
                                           2, 5, 6,
                                           3, 6, 9).finished();
const Matrix4d A22 = (Eigen::Matrix4d() << 11, 12, 13, 23,
                                           12, 15, 16, 26,
                                           13, 16, 19, 19,
                                           23, 26, 19, 23).finished();
const Eigen::Matrix<double, 4, 3> A21 =
    (Eigen::Matrix<double, 4, 3>() << 11, 22, 33,
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
 where A12 = A21.transpose() or 0 depending on whether the matrix is symmetric.
*/
MatrixXd MakeDenseMatrix(bool is_symmetric) {
  MatrixXd A = MatrixXd::Zero(9, 9);
  A.block<2, 2>(0, 0) = A00;
  A.block<3, 3>(2, 2) = A11;
  A.block<4, 3>(5, 2) = A21;
  if (is_symmetric) {
    A.block<3, 4>(2, 5) = A21.transpose();
  } else {
    A.block<3, 4>(2, 5) = MatrixXd::Zero(3, 4);
  }
  A.block<4, 4>(5, 5) = A22;
  return A;
}

/* Makes a block sparse lower triangular matrix
   A =   A00 |  0  |  0
        -----------------
          0  | A11 |  0
        -----------------
          0  | A21 | A22
 */
BlockSparseLowerTriangularMatrix MakeLowerTriangularMatrix() {
  std::vector<int> diag{2, 3, 4};
  std::vector<std::vector<int>> sparsity;
  sparsity.push_back(std::vector<int>{0});
  sparsity.push_back(std::vector<int>{1, 2});
  sparsity.push_back(std::vector<int>{2});
  BlockSparsityPattern pattern(diag, sparsity);
  BlockSparseLowerTriangularMatrix A_blocks(pattern);
  A_blocks.SetBlock(0, 0, A00);
  A_blocks.SetBlock(1, 1, A11);
  A_blocks.SetBlock(2, 1, A21);
  A_blocks.SetBlock(2, 2, A22);
  return A_blocks;
}

/* Makes a block sparse symmetric matrix
   A =   A00 |  0  |  0
        -----------------
          0  | A11 | A12
        -----------------
          0  | A21 | A22
 where A21 = A12.transpose(). */
BlockSparseSymmetricMatrix MakeSymmetricMatrix() {
  std::vector<int> diag{2, 3, 4};
  std::vector<std::vector<int>> sparsity;
  sparsity.push_back(std::vector<int>{0});
  sparsity.push_back(std::vector<int>{1, 2});
  sparsity.push_back(std::vector<int>{2});
  BlockSparsityPattern pattern(diag, sparsity);
  BlockSparseSymmetricMatrix A_blocks(pattern);
  A_blocks.SetBlock(0, 0, A00);
  A_blocks.SetBlock(1, 1, A11);
  A_blocks.SetBlock(2, 1, A21);
  A_blocks.SetBlock(2, 2, A22);
  return A_blocks;
}

GTEST_TEST(TriangularBlockSparseMatrixTest, Construction) {
  const BlockSparseLowerTriangularMatrix A_triangular =
      MakeLowerTriangularMatrix();
  const BlockSparseSymmetricMatrix A_symmetric = MakeSymmetricMatrix();
  EXPECT_EQ(A_triangular.MakeDenseMatrix(), MakeDenseMatrix(false));
  EXPECT_EQ(A_symmetric.MakeDenseMatrix(), MakeDenseMatrix(true));
}

GTEST_TEST(TriangularBlockSparseMatrixTest, SetZero) {
  BlockSparseLowerTriangularMatrix A_triangular = MakeLowerTriangularMatrix();
  A_triangular.SetZero();
  EXPECT_EQ(A_triangular.MakeDenseMatrix(), MatrixXd::Zero(9, 9));

  BlockSparseSymmetricMatrix A_symmetric = MakeSymmetricMatrix();
  A_symmetric.SetZero();
  EXPECT_EQ(A_symmetric.MakeDenseMatrix(), MatrixXd::Zero(9, 9));
}

GTEST_TEST(TriangularBlockSparseMatrixTest, Getter) {
  BlockSparseLowerTriangularMatrix A_triangular = MakeLowerTriangularMatrix();
  BlockSparseSymmetricMatrix A_symmetric = MakeSymmetricMatrix();
  /* Existing diagonal blocks. */
  ASSERT_TRUE(A_triangular.HasBlock(0, 0));
  ASSERT_TRUE(A_symmetric.HasBlock(0, 0));
  /* Existing lower triangular blocks. */
  ASSERT_TRUE(A_triangular.HasBlock(2, 1));
  ASSERT_TRUE(A_symmetric.HasBlock(2, 1));
  /* Upper triangular block that only exist in the symmetric matrix. */
  ASSERT_FALSE(A_triangular.HasBlock(1, 2));
  ASSERT_TRUE(A_symmetric.HasBlock(1, 2));
  /* Out of range block. */
  ASSERT_FALSE(A_symmetric.HasBlock(100, 200));
  ASSERT_FALSE(A_triangular.HasBlock(100, 200));

  EXPECT_EQ(A_triangular.block(2, 1), A21);
  EXPECT_EQ(A_symmetric.block(2, 1), A21);
  EXPECT_EQ(A_triangular.diagonal_block(0), A00);
  EXPECT_EQ(A_symmetric.diagonal_block(0), A00);
  EXPECT_EQ(A_triangular.block_flat(1, 1), A21);
  EXPECT_EQ(A_symmetric.block_flat(1, 1), A21);

  EXPECT_EQ(A_triangular.block_row_indices(1), std::vector<int>({1, 2}));
  EXPECT_EQ(A_symmetric.block_row_indices(1), std::vector<int>({1, 2}));

  EXPECT_EQ(A_triangular.starting_cols(), std::vector<int>({0, 2, 5}));
  EXPECT_EQ(A_symmetric.starting_cols(), std::vector<int>({0, 2, 5}));
}

GTEST_TEST(TriangularBlockSparseMatrixTest, SetBlock) {
  BlockSparseLowerTriangularMatrix A = MakeLowerTriangularMatrix();
  MatrixXd m = MakeArbitraryMatrix(4, 3);
  A.SetBlock(2, 1, m);
  EXPECT_EQ(A.block(2, 1), m);
  A.AddToBlock(2, 1, m);
  EXPECT_EQ(A.block(2, 1), 2 * m);
  A.SetBlockFlat(1, 1, m);
  EXPECT_EQ(A.block(2, 1), m);
}

GTEST_TEST(TriangularBlockSparseMatrixTest, ZeroRowsAndColumns) {
  /* Throws for lower triangular matrix. */
  BlockSparseLowerTriangularMatrix A_triangular = MakeLowerTriangularMatrix();
  DRAKE_EXPECT_THROWS_MESSAGE(A_triangular.ZeroRowsAndColumns({0, 1}),
                              ".*is_symmetric.*");

  /* Keeps the diagonal entries for the diagonal blocks 0 and 1. Keeps the
   diagonal block 2 untouched. All off-diagonal blocks are zeroed out. */
  BlockSparseSymmetricMatrix A_symmetric = MakeSymmetricMatrix();
  A_symmetric.ZeroRowsAndColumns({0, 1});

  MatrixXd A_symmetric_dense = MatrixXd::Zero(9, 9);
  A_symmetric_dense.topLeftCorner<2, 2>() = A00.diagonal().asDiagonal();
  A_symmetric_dense.block<3, 3>(2, 2) = A11.diagonal().asDiagonal();
  A_symmetric_dense.bottomRightCorner<4, 4>() = A22;

  EXPECT_EQ(A_symmetric.MakeDenseMatrix(), A_symmetric_dense);

  /* Throws if input is out of range. */
  DRAKE_EXPECT_THROWS_MESSAGE(A_symmetric.ZeroRowsAndColumns({42}),
                              ".* out of range.*42 is given.*");
}

GTEST_TEST(TriangularBlockSparseMatrixTest, MakeDenseBottomRightCorner) {
  BlockSparseLowerTriangularMatrix A_triangular = MakeLowerTriangularMatrix();
  MatrixXd expected = MakeDenseMatrix(false).bottomRightCorner(7, 7);
  EXPECT_EQ(expected, A_triangular.MakeDenseBottomRightCorner(2));
  EXPECT_EQ(MatrixXd::Zero(0, 0), A_triangular.MakeDenseBottomRightCorner(0));

  BlockSparseSymmetricMatrix A_symmetric = MakeSymmetricMatrix();
  expected = MakeDenseMatrix(true).bottomRightCorner(7, 7);
  EXPECT_EQ(expected, A_symmetric.MakeDenseBottomRightCorner(2));
  EXPECT_EQ(MatrixXd::Zero(0, 0), A_symmetric.MakeDenseBottomRightCorner(0));
}

GTEST_TEST(TriangularBlockSparseMatrixTest, InvalidOperations) {
  if (kDrakeAssertIsArmed) {
    BlockSparseLowerTriangularMatrix A_triangular = MakeLowerTriangularMatrix();
    BlockSparseSymmetricMatrix A_symmetric = MakeSymmetricMatrix();
    /* i <= block_rows() fails. */
    DRAKE_ASSERT_THROWS_MESSAGE_IF_ARMED(A_triangular.AddToBlock(200, 100, A00),
                                         ".*out of bound.*");
    DRAKE_ASSERT_THROWS_MESSAGE_IF_ARMED(A_symmetric.AddToBlock(200, 100, A00),
                                         ".*out of bound.*");
    /* j <= i fails. */
    DRAKE_ASSERT_THROWS_MESSAGE_IF_ARMED(
        A_triangular.AddToBlock(0, 1, MakeArbitraryMatrix(2, 3)),
        ".*out of bound.*");
    DRAKE_ASSERT_THROWS_MESSAGE_IF_ARMED(
        A_symmetric.AddToBlock(0, 1, MakeArbitraryMatrix(2, 3)),
        ".*out of bound.*");
    /* 0 <= j fails. */
    DRAKE_ASSERT_THROWS_MESSAGE_IF_ARMED(
        A_triangular.AddToBlock(0, -1, MakeArbitraryMatrix(2, 3)),
        ".*out of bound.*");
    DRAKE_ASSERT_THROWS_MESSAGE_IF_ARMED(
        A_symmetric.AddToBlock(0, -1, MakeArbitraryMatrix(2, 3)),
        ".*out of bound.*");

    /* Non-existing block. */
    DRAKE_ASSERT_THROWS_MESSAGE_IF_ARMED(
        A_triangular.AddToBlock(2, 0, MakeArbitraryMatrix(2, 4)),
        ".*doesn't exist.*");
    DRAKE_ASSERT_THROWS_MESSAGE_IF_ARMED(
        A_symmetric.AddToBlock(2, 0, MakeArbitraryMatrix(2, 4)),
        ".*doesn't exist.*");

    MatrixXd non_symmetric_matrix = MatrixXd::Zero(4, 4);
    non_symmetric_matrix(1, 0) = 0.1;
    /* Non-symmetric diagonal block. */
    DRAKE_ASSERT_THROWS_MESSAGE_IF_ARMED(
        A_symmetric.AddToBlock(2, 2, non_symmetric_matrix),
        ".*must be symmetric[^]*");
    /* No false error when zero matrix is added to the diagonal block. */
    EXPECT_NO_THROW(A_symmetric.AddToBlock(2, 2, MatrixXd::Zero(4, 4)));

    /* Evidence that other functions do the same checks. */
    ASSERT_THROW(A_triangular.block(0, 1), std::exception);
    ASSERT_THROW(A_triangular.SetBlock(0, 1, MakeArbitraryMatrix(2, 3)),
                 std::exception);
    ASSERT_THROW(A_symmetric.SetBlockFlat(0, 2, non_symmetric_matrix),
                 std::exception);
  }
}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
