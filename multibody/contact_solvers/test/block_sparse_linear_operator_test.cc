#include "drake/multibody/contact_solvers/block_sparse_linear_operator.h"

#include <memory>

#include <Eigen/SparseCore>
#include <gtest/gtest.h>

#include "drake/multibody/contact_solvers/block_sparse_matrix.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace {

using SparseMatrixd = Eigen::SparseMatrix<double>;
using SparseVectord = Eigen::SparseVector<double>;
using Eigen::VectorXd;

class BlockSparseLinearOperatorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Build dense A.
    A_.resize(kNumRows, kNumCols);
    // clang-format off
    A_ <<  1, 2,   0,   2, 4,   0, 0, 0, 0,
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

    // Build block sparse A.
    BlockSparseMatrixBuilder<double> builder(3, 4, 5);
    builder.PushBlock(0, 0, A_.block(0, 0, 3, 2));
    builder.PushBlock(0, 2, A_.block(0, 3, 3, 2));
    builder.PushBlock(1, 2, A_.block(3, 3, 4, 2));
    builder.PushBlock(2, 1, A_.block(7, 2, 3, 1));
    builder.PushBlock(2, 3, A_.block(7, 5, 3, 4));
    Ablock_ = builder.Build();

    // Build operator for A.
    A_operator_ =
        std::make_unique<BlockSparseLinearOperator<double>>("A", &Ablock_);
  }

  const int kNumRows{10};
  const int kNumCols{9};
  MatrixX<double> A_;
  BlockSparseMatrix<double> Ablock_;
  std::unique_ptr<BlockSparseLinearOperator<double>> A_operator_;
};

TEST_F(BlockSparseLinearOperatorTest, Construction) {
  EXPECT_EQ(A_operator_->name(), "A");
  EXPECT_EQ(A_operator_->rows(), kNumRows);
  EXPECT_EQ(A_operator_->cols(), kNumCols);
}

TEST_F(BlockSparseLinearOperatorTest, MultiplyDense) {
  VectorXd y(kNumRows);
  const VectorXd x = VectorXd::LinSpaced(kNumCols, 0.0, 1.0);
  A_operator_->Multiply(x, &y);
  VectorXd y_expected(kNumRows);
  Ablock_.Multiply(x, &y_expected);
  EXPECT_EQ(y, y_expected);
}

TEST_F(BlockSparseLinearOperatorTest, MultiplyByTransposeDense) {
  VectorXd y(kNumCols);
  const VectorXd x = VectorXd::LinSpaced(kNumRows, 0.0, 1.0);
  A_operator_->MultiplyByTranspose(x, &y);
  VectorXd y_expected(kNumCols);
  Ablock_.MultiplyByTranspose(x, &y_expected);
  EXPECT_EQ(y, y_expected);
}

TEST_F(BlockSparseLinearOperatorTest, MultiplySparse) {
  SparseVectord y(kNumRows);
  const SparseVectord x = VectorXd::LinSpaced(kNumCols, 0.0, 1.0).sparseView();
  A_operator_->Multiply(x, &y);
  VectorXd y_expected(kNumRows);
  Ablock_.Multiply(VectorXd(x), &y_expected);
  EXPECT_EQ(VectorXd(y), y_expected);
}

TEST_F(BlockSparseLinearOperatorTest, MultiplyByTransposeSparse) {
  SparseVectord y(kNumCols);
  const SparseVectord x = VectorXd::LinSpaced(kNumRows, 0.0, 1.0).sparseView();
  A_operator_->MultiplyByTranspose(x, &y);
  VectorXd y_expected(kNumCols);
  Ablock_.MultiplyByTranspose(VectorXd(x), &y_expected);
  EXPECT_EQ(VectorXd(y), y_expected);
}

TEST_F(BlockSparseLinearOperatorTest, AssembleMatrix) {
  SparseMatrixd A_sparse(A_operator_->rows(), A_operator_->cols());
  A_operator_->AssembleMatrix(&A_sparse);
  // Copy into a dense matrix for comparison.
  const MatrixX<double> A_dense = A_sparse;
  EXPECT_EQ(A_dense, A_);
}

TEST_F(BlockSparseLinearOperatorTest, AssembleMatrixBlockSparse) {
  BlockSparseMatrix<double> A_block;
  A_operator_->AssembleMatrix(&A_block);
  // Copy into a dense matrix for comparison.
  const MatrixX<double> A_dense = A_block.MakeDenseMatrix();
  EXPECT_EQ(A_dense, A_);
}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
