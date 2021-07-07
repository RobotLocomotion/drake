#include "drake/multibody/contact_solvers/block_sparse_matrix.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace {

class BlockSparseMatrixTest : public ::testing::Test {
 protected:
  void SetUp() override {
    J_.resize(rows_, cols_);
    // clang-format off
    J_ <<  1, 2,   0,   2, 4,   0, 0, 0, 0,
           0, 1,   0,   1, 3,   0, 0, 0, 0,
           1, 3,   0,   2, 4,   0, 0, 0, 0,

           0, 0,   0,   1, 2,   0, 0, 0, 0,
           0, 0,   0,   2, 1,   0, 0, 0, 0,
           0, 0,   0,   2, 3,   0, 0, 0, 0,
           0, 0,   0,   1, 2,   0, 0, 0, 0,

           0, 0,   1,   0, 0,   1, 2, 3, 4,
           0, 0,   2,   0, 0,   1, 2, 3, 4,
           0, 0,   3,   0, 0,   1, 2, 3, 4;
    // clang-format on

    BlockSparseMatrixBuilder<double> builder(block_rows_, block_cols_, 5);
    builder.PushBlock(0, 0, J_.block(0, 0, 3, 2));
    builder.PushBlock(0, 2, J_.block(0, 3, 3, 2));
    builder.PushBlock(1, 2, J_.block(3, 3, 4, 2));
    builder.PushBlock(2, 1, J_.block(7, 2, 3, 1));
    builder.PushBlock(2, 3, J_.block(7, 5, 3, 4));
    Jblk_ = builder.Build();
  }
  // A dense matrix for testing. Many of its entries will be zero.
  const int rows_{10};
  const int cols_{9};
  Eigen::MatrixXd J_;
  // Block sparse representation of J.
  const int block_rows_{3};
  const int block_cols_{4};
  BlockSparseMatrix<double> Jblk_;
};

TEST_F(BlockSparseMatrixTest, Sizes) {
  EXPECT_EQ(Jblk_.rows(), rows_);
  EXPECT_EQ(Jblk_.cols(), cols_);
  EXPECT_EQ(Jblk_.block_rows(), block_rows_);
  EXPECT_EQ(Jblk_.block_cols(), block_cols_);
  EXPECT_EQ(Jblk_.num_blocks(), 5);
  // Verify blocks's size.
  EXPECT_EQ(Jblk_.block_row_size(0), 3);
  EXPECT_EQ(Jblk_.block_row_size(1), 4);
  EXPECT_EQ(Jblk_.block_row_size(2), 3);
  EXPECT_EQ(Jblk_.block_col_size(0), 2);
  EXPECT_EQ(Jblk_.block_col_size(1), 1);
  EXPECT_EQ(Jblk_.block_col_size(2), 2);
  EXPECT_EQ(Jblk_.block_col_size(3), 4);
  // Verify block structure.
  EXPECT_EQ(Jblk_.row_start(0), 0);
  EXPECT_EQ(Jblk_.row_start(1), 3);
  EXPECT_EQ(Jblk_.row_start(2), 7);
  EXPECT_EQ(Jblk_.col_start(0), 0);
  EXPECT_EQ(Jblk_.col_start(1), 2);
  EXPECT_EQ(Jblk_.col_start(2), 3);
  EXPECT_EQ(Jblk_.col_start(3), 5);
}

TEST_F(BlockSparseMatrixTest, AccessBlocks) {
  // Here we simply verify that get_block() returns the same matrices at each
  // entry of get_blocks().
  EXPECT_EQ(Jblk_.get_blocks().size(), 5u);
  for (int b = 0; b < Jblk_.num_blocks(); ++b) {
    EXPECT_EQ(Jblk_.get_block(b), std::get<2>(Jblk_.get_blocks()[b]));
  }
}

// This test verifies that the BlockSparseMatrixBuilder makes the expected
// block-sparse representation of the matrix J in this test. We also unit test
// BlockSparseMatrix::MakeDenseMatrix().
TEST_F(BlockSparseMatrixTest, VerifyMatrix) {
  const MatrixXd J_reconstructed = Jblk_.MakeDenseMatrix();
  EXPECT_TRUE(CompareMatrices(J_, J_reconstructed, 0));
}

// Verifies the builder throws an exception when a row is left empty.
TEST_F(BlockSparseMatrixTest, EmptyRow) {
  BlockSparseMatrixBuilder<double> builder(3, 4, 5);
  builder.PushBlock(0, 0, J_.block(0, 0, 3, 2));
  builder.PushBlock(0, 2, J_.block(0, 3, 3, 2));
  // builder.PushBlock(1, 2, J_.block(3, 3, 4, 2));  // 3rd block omitted.
  builder.PushBlock(2, 1, J_.block(7, 2, 3, 1));
  builder.PushBlock(2, 3, J_.block(7, 5, 3, 4));
  DRAKE_EXPECT_THROWS_MESSAGE(builder.Build(),
                              "No block was specified for row 1.");
}

// Verifies the builder throws an excpetion when a column is left empty.
TEST_F(BlockSparseMatrixTest, EmptyColumn) {
  BlockSparseMatrixBuilder<double> builder(3, 4, 5);
  builder.PushBlock(0, 0, J_.block(0, 0, 3, 2));
  builder.PushBlock(0, 2, J_.block(0, 3, 3, 2));
  builder.PushBlock(1, 2, J_.block(3, 3, 4, 2));
  // builder.PushBlock(2, 1, J_.block(7, 2, 3, 1));  // 4th block omitted.
  builder.PushBlock(2, 3, J_.block(7, 5, 3, 4));
  DRAKE_EXPECT_THROWS_MESSAGE(builder.Build(),
                              "No block was specified for column 1.");
}

TEST_F(BlockSparseMatrixTest, Multiply) {
  VectorXd x(cols_);
  VectorXd y(rows_);

  // Test matrix multiplication on a couple of arbitrary vectors.
  x << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  Jblk_.Multiply(x, &y);
  EXPECT_TRUE(
      CompareMatrices(J_ * x, y, std::numeric_limits<double>::epsilon()));

  x << -1, 2, -3, 4, 5, -6, 7, 8, 9;
  Jblk_.Multiply(x, &y);
  EXPECT_TRUE(
      CompareMatrices(J_ * x, y, std::numeric_limits<double>::epsilon()));

  // Test multiplication by transpose on a couple of arbitrary vectors.
  y << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10;
  Jblk_.MultiplyByTranspose(y, &x);
  EXPECT_TRUE(CompareMatrices(J_.transpose() * y, x,
                              std::numeric_limits<double>::epsilon()));
  y << -1, 2, -3, 4, -5, 6, -7, 8, -9, 10;
  Jblk_.MultiplyByTranspose(y, &x);
  EXPECT_TRUE(CompareMatrices(J_.transpose() * y, x,
                              std::numeric_limits<double>::epsilon()));
}

// Tests that we allow to overestimate the capacity of non-zero blocks.
TEST_F(BlockSparseMatrixTest, CapacityOverestimated) {
  // An arbitrary capacity of non-zero blocks estimated to be larger than the
  // actual number, which is 5.
  const int kCapacityOverestimated = 6;
  BlockSparseMatrixBuilder<double> builder(3, 4, kCapacityOverestimated);
  builder.PushBlock(0, 0, J_.block(0, 0, 3, 2));
  builder.PushBlock(0, 2, J_.block(0, 3, 3, 2));
  builder.PushBlock(1, 2, J_.block(3, 3, 4, 2));
  builder.PushBlock(2, 1, J_.block(7, 2, 3, 1));
  builder.PushBlock(2, 3, J_.block(7, 5, 3, 4));
  EXPECT_NO_THROW(builder.Build());
  const MatrixXd J_reconstructed = Jblk_.MakeDenseMatrix();
  EXPECT_TRUE(CompareMatrices(J_, J_reconstructed, 0));
}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
