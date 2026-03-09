#include "drake/multibody/contact_solvers/matrix_block.h"

#include <limits>
#include <utility>
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

/* Returns an arbitrary Block3x3SparseMatrix with size 12-by-9. */
Block3x3SparseMatrix<double> MakeBlockSparseMatrix() {
  Block3x3SparseMatrix<double> sparse_matrix(4, 3);
  std::vector<Block3x3SparseMatrix<double>::Triplet> triplets;
  triplets.emplace_back(0, 0, Matrix3d::Constant(1.0));
  triplets.emplace_back(2, 1, Matrix3d::Constant(2.0));
  triplets.emplace_back(3, 2, Matrix3d::Constant(3.0));
  sparse_matrix.SetFromTriplets(triplets);
  EXPECT_EQ(sparse_matrix.num_blocks(), 3);
  return sparse_matrix;
}

GTEST_TEST(MatrixBlockTest, Constructors) {
  Block3x3SparseMatrix<double> sparse = MakeBlockSparseMatrix();
  MatrixXd dense = sparse.MakeDenseMatrix();

  const MatrixBlock<double> sparse_block(std::move(sparse));
  const MatrixBlock<double> dense_block(std::move(dense));

  EXPECT_EQ(sparse_block.rows(), 12);
  EXPECT_EQ(dense_block.rows(), 12);
  EXPECT_EQ(sparse_block.cols(), 9);
  EXPECT_EQ(dense_block.cols(), 9);
  EXPECT_EQ(sparse_block.size(), 12 * 9);
  EXPECT_EQ(dense_block.size(), 12 * 9);
  EXPECT_FALSE(sparse_block.is_dense());
  EXPECT_TRUE(dense_block.is_dense());

  EXPECT_TRUE(CompareMatrices(sparse_block.MakeDenseMatrix(),
                              dense_block.MakeDenseMatrix()));
}

GTEST_TEST(MatrixBlockTest, MultiplyAndAddTo) {
  const MatrixXd x = MakeArbitraryMatrix(9, 7);
  Block3x3SparseMatrix<double> sparse_matrix = MakeBlockSparseMatrix();
  const MatrixXd dense_matrix = sparse_matrix.MakeDenseMatrix();

  const MatrixBlock<double> sparse_block(std::move(sparse_matrix));
  const MatrixBlock<double> dense_block(dense_matrix);

  /* Set the destinations to compatible-sized non-zero vectors. */
  MatrixXd y1 = MakeArbitraryMatrix(12, 7);
  MatrixXd y2 = y1;
  MatrixXd expected_y = y1;

  sparse_block.MultiplyAndAddTo(x, &y1);
  dense_block.MultiplyAndAddTo(x, &y2);
  expected_y += dense_matrix * x;

  EXPECT_TRUE(CompareMatrices(y1, expected_y));
  EXPECT_TRUE(CompareMatrices(y2, expected_y));
}

GTEST_TEST(MatrixBlockTest, TransposeAndMultiplyAndAddTo) {
  Block3x3SparseMatrix<double> sparse_matrix = MakeBlockSparseMatrix();
  const MatrixXd dense_matrix = sparse_matrix.MakeDenseMatrix();
  const MatrixBlock<double> sparse_block(std::move(sparse_matrix));
  const MatrixBlock<double> dense_block(dense_matrix);

  /* Set destination to be compatible-sized non-zero matrices. */
  MatrixXd y1 = MakeArbitraryMatrix(9, 9);
  MatrixXd y2 = y1;
  MatrixXd y3 = y1;
  MatrixXd y4 = y1;
  MatrixXd expected_y = y1;

  expected_y += dense_matrix.transpose() * dense_matrix;
  sparse_block.TransposeAndMultiplyAndAddTo(dense_block, &y1);
  sparse_block.TransposeAndMultiplyAndAddTo(sparse_block, &y2);
  dense_block.TransposeAndMultiplyAndAddTo(dense_block, &y3);
  dense_block.TransposeAndMultiplyAndAddTo(sparse_block, &y4);

  EXPECT_TRUE(CompareMatrices(y1, expected_y));
  EXPECT_TRUE(CompareMatrices(y2, expected_y));
  EXPECT_TRUE(CompareMatrices(y3, expected_y));
  EXPECT_TRUE(CompareMatrices(y4, expected_y));
}

GTEST_TEST(MatrixBlockTest, LeftMultiplyByBlockDiagonal) {
  Block3x3SparseMatrix<double> sparse_matrix = MakeBlockSparseMatrix();
  const MatrixXd dense_matrix = sparse_matrix.MakeDenseMatrix();
  const MatrixBlock<double> sparse_block(std::move(sparse_matrix));
  const MatrixBlock<double> dense_block(dense_matrix);

  const int num_Gs = 7;
  std::vector<MatrixXd> Gs;
  for (int i = 0; i < num_Gs; ++i) {
    Gs.emplace_back(Matrix3d::Constant(3.14 * i));
  }
  const int start = 2;
  const int end = 5;
  const MatrixBlock<double> sparse_result =
      sparse_block.LeftMultiplyByBlockDiagonal(Gs, start, end);
  const MatrixBlock<double> dense_result =
      dense_block.LeftMultiplyByBlockDiagonal(Gs, start, end);

  /* Compute the expected results using dense matrices. */
  MatrixXd dense_G = MatrixXd::Zero(12, 12);
  for (int i = 0; i < 4; ++i) {
    dense_G.block<3, 3>(3 * i, 3 * i) = Gs[start + i];
  }
  MatrixXd expected = dense_G * dense_matrix;

  EXPECT_TRUE(CompareMatrices(sparse_result.MakeDenseMatrix(), expected));
  EXPECT_TRUE(CompareMatrices(dense_result.MakeDenseMatrix(), expected));
}

GTEST_TEST(MatrixBlockTest, LeftMultiplyByBlockDiagonalWithNon3x3GBlocks) {
  Block3x3SparseMatrix<double> sparse_matrix = MakeBlockSparseMatrix();
  const MatrixBlock<double> sparse_block(std::move(sparse_matrix));

  /* Here we construct a block diagonal G with 7 square blocks of sizes
   3 3 3 [3 6 3] 3. We'll use only the three blocks marked with brackets to
   multiply with the sparse matrix which has 12 rows. */
  MatrixXd matrix6(6, 6);
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      matrix6(i, j) = 0.123 * i + 0.456 * j;
    }
  }
  const int num_Gs = 7;
  std::vector<MatrixXd> Gs;
  for (int i = 0; i < num_Gs; ++i) {
    if (i != 4) {
      Gs.emplace_back(Matrix3d::Constant(3.14 * i));
    } else {
      Gs.emplace_back(matrix6);
    }
  }
  const int start = 3;
  const int end = 5;
  const MatrixBlock<double> result =
      sparse_block.LeftMultiplyByBlockDiagonal(Gs, start, end);

  /* Compute the expected results using dense matrices. */
  MatrixXd dense_G = MatrixXd::Zero(12, 12);
  dense_G.topLeftCorner(3, 3) = Gs[3];
  dense_G.block<6, 6>(3, 3) = Gs[4];
  dense_G.bottomRightCorner(3, 3) = Gs[5];

  MatrixXd expected = dense_G * sparse_block.MakeDenseMatrix();
  EXPECT_TRUE(CompareMatrices(result.MakeDenseMatrix(), expected,
                              16 * std::numeric_limits<double>::epsilon()));
}

GTEST_TEST(MatrixBlockTest, MultiplyWithScaledTransposeAndAddTo) {
  Block3x3SparseMatrix<double> sparse_matrix = MakeBlockSparseMatrix();
  const MatrixXd dense_matrix = sparse_matrix.MakeDenseMatrix();
  const MatrixBlock<double> sparse_block(std::move(sparse_matrix));
  const MatrixBlock<double> dense_block(dense_matrix);

  /* Set scale and destination to be compatible-sized non-zero matrices. */
  const VectorXd scale = MakeArbitraryMatrix(9, 1);
  MatrixXd y1 = MakeArbitraryMatrix(12, 12);
  MatrixXd y2 = y1;
  MatrixXd expected = y1;

  sparse_block.MultiplyWithScaledTransposeAndAddTo(scale, &y1);
  dense_block.MultiplyWithScaledTransposeAndAddTo(scale, &y2);
  expected += dense_matrix * scale.asDiagonal() * dense_matrix.transpose();
  EXPECT_TRUE(CompareMatrices(y1, expected));
  EXPECT_TRUE(CompareMatrices(y2, expected));
}

GTEST_TEST(MatrixBlockTest, StackMatrixBlock) {
  Block3x3SparseMatrix<double> sparse_matrix1 = MakeBlockSparseMatrix();
  const MatrixXd dense_matrix1 = sparse_matrix1.MakeDenseMatrix();
  const MatrixBlock<double> sparse_block1(std::move(sparse_matrix1));
  const MatrixBlock<double> dense_block1(dense_matrix1);

  /* Make a second MatrixBlock that's different from the existing one. */
  Block3x3SparseMatrix<double> sparse_matrix2(2, 3);
  std::vector<Block3x3SparseMatrix<double>::Triplet> triplets;
  triplets.emplace_back(0, 0, Matrix3d::Constant(4.0));
  triplets.emplace_back(1, 1, Matrix3d::Constant(5.0));
  sparse_matrix2.SetFromTriplets(triplets);
  const MatrixXd dense_matrix2 = sparse_matrix2.MakeDenseMatrix();
  const MatrixBlock<double> sparse_block2(std::move(sparse_matrix2));
  const MatrixBlock<double> dense_block2(dense_matrix2);

  std::vector<MatrixBlock<double>> dense_blocks = {dense_block1, dense_block2};
  std::vector<MatrixBlock<double>> sparse_blocks = {sparse_block1,
                                                    sparse_block2};

  MatrixXd expected(dense_matrix1.rows() + dense_matrix2.rows(),
                    dense_matrix1.cols());
  expected.topRows(dense_matrix1.rows()) = dense_matrix1;
  expected.bottomRows(dense_matrix2.rows()) = dense_matrix2;

  const MatrixBlock<double> dense_stack = StackMatrixBlocks(dense_blocks);
  const MatrixBlock<double> sparse_stack = StackMatrixBlocks(sparse_blocks);
  EXPECT_TRUE(CompareMatrices(dense_stack.MakeDenseMatrix(), expected));
  EXPECT_TRUE(CompareMatrices(sparse_stack.MakeDenseMatrix(), expected));
}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
