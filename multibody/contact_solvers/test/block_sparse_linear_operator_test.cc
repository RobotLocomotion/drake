#include "drake/multibody/contact_solvers/block_sparse_linear_operator.h"

#include <memory>

#include <Eigen/SparseCore>
#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace {

using SparseMatrixd = Eigen::SparseMatrix<double>;
using SparseVectord = Eigen::SparseVector<double>;
using Eigen::VectorXd;
using Triplet = Eigen::Triplet<double>;

class BlockSparseLinearOperatorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Build dense A.
    A_.resize(kNumRows_, kNumCols_);
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
    Aop_ = std::make_unique<BlockSparseLinearOperator<double>>("A", &Ablock_);
  }

  const int kNumRows_{10};
  const int kNumCols_{9};
  MatrixX<double> A_;
  BlockSparseMatrix<double> Ablock_;
  std::unique_ptr<BlockSparseLinearOperator<double>> Aop_;
};

TEST_F(BlockSparseLinearOperatorTest, Construction) {
  EXPECT_EQ(Aop_->name(), "A");
  EXPECT_EQ(Aop_->rows(), kNumRows_);
  EXPECT_EQ(Aop_->cols(), kNumCols_);
}

TEST_F(BlockSparseLinearOperatorTest, MultiplyDense) {
  VectorXd y(kNumRows_);
  const VectorXd x = VectorXd::LinSpaced(kNumCols_, 0.0, 1.0);
  Aop_->Multiply(x, &y);
  VectorXd y_expected(kNumRows_);
  Ablock_.Multiply(x, &y_expected);
  EXPECT_EQ(y, y_expected);
}

TEST_F(BlockSparseLinearOperatorTest, MultiplyByTransposeDense) {
  VectorXd y(kNumCols_);
  const VectorXd x = VectorXd::LinSpaced(kNumRows_, 0.0, 1.0);
  Aop_->MultiplyByTranspose(x, &y);
  VectorXd y_expected(kNumCols_);
  Ablock_.MultiplyByTranspose(x, &y_expected);
  EXPECT_EQ(y, y_expected);
}

TEST_F(BlockSparseLinearOperatorTest, MultiplySparse) {
  SparseVectord y(kNumRows_);
  const SparseVectord x = VectorXd::LinSpaced(kNumCols_, 0.0, 1.0).sparseView();
  Aop_->Multiply(x, &y);
  VectorXd y_expected(kNumRows_);
  Ablock_.Multiply(VectorXd(x), &y_expected);
  EXPECT_EQ(VectorXd(y), y_expected);
}

TEST_F(BlockSparseLinearOperatorTest, MultiplyByTransposeSparse) {
  SparseVectord y(kNumCols_);
  const SparseVectord x = VectorXd::LinSpaced(kNumRows_, 0.0, 1.0).sparseView();
  Aop_->MultiplyByTranspose(x, &y);
  VectorXd y_expected(kNumCols_);
  Ablock_.MultiplyByTranspose(VectorXd(x), &y_expected);
  EXPECT_EQ(VectorXd(y), y_expected);
}

TEST_F(BlockSparseLinearOperatorTest, AssembleMatrix) {
  SparseMatrixd Asparse(Aop_->rows(), Aop_->cols());
  Aop_->AssembleMatrix(&Asparse);
  // Copy into a dense matrix for comparison.
  const MatrixX<double> Acopy = Asparse;
  EXPECT_EQ(Acopy, A_);
}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
