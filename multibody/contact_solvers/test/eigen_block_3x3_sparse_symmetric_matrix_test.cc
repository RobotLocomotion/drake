#include "drake/multibody/contact_solvers/eigen_block_3x3_sparse_symmetric_matrix.h"

#include <limits>
#include <utility>
#include <vector>

#include <Eigen/IterativeLinearSolvers>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/contact_solvers/block_sparse_lower_triangular_or_symmetric_matrix.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace {

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* Makes an arbitrary SPD matrix with the following nonzero pattern.

  X X X | O O O | O O O | O O O
  X X X | O O O | O O O | O O O
  X X X | O O O | O O O | O O O
  ----- | ----- |-------| -----
  O O O | X X X | X X X | X X X
  O O O | X X X | X X X | X X X
  O O O | X X X | X X X | X X X
  ----- | ----- |-------| -----
  O O O | X X X | X X X | O O O
  O O O | X X X | X X X | O O O
  O O O | X X X | X X X | O O O
  ----- | ----- |-------| -----
  O O O | X X X | O O O | X X X
  O O O | X X X | O O O | X X X
  O O O | X X X | O O O | X X X

 The scaling factor can be used to control the values of each nonzero entry. */
Eigen::Matrix<double, 12, 12> MakeSpdMatrix() {
  Eigen::Matrix<double, 12, 12> A;
  for (int i = 0; i < 12; ++i) {
    for (int j = 0; j < 12; ++j) {
      A(i, j) = 0.03 * i + 0.02 * j;
    }
  }
  Eigen::Matrix<double, 12, 12> I = Eigen::Matrix<double, 12, 12>::Identity();
  /* Add a large diagonal entry to ensure diagonal dominance. */
  Eigen::Matrix<double, 12, 12> result = A * A.transpose() + 12 * I;
  /* Zeroing off-diagonal entries of a diagonally dominant matrix does not
   affect diagonal dominance, which in turns implies SPD-ness. */
  result.block<9, 3>(3, 0).setZero();
  result.block<3, 9>(0, 3).setZero();
  result.block<3, 3>(9, 6).setZero();
  result.block<3, 3>(6, 9).setZero();
  return result;
}

/* Makes a sparse version of the result from MakeSpdMatrix(). */
BlockSparseSymmetricMatrix3d MakeSparseSpdMatrix() {
  std::vector<std::vector<int>> sparsity;
  sparsity.emplace_back(std::vector<int>{0});
  sparsity.emplace_back(std::vector<int>{1, 2, 3});
  sparsity.emplace_back(std::vector<int>{2});
  sparsity.emplace_back(std::vector<int>{3});
  std::vector<int> block_sizes = {3, 3, 3, 3};
  BlockSparsityPattern block_pattern(block_sizes, sparsity);

  BlockSparseSymmetricMatrix3d A(std::move(block_pattern));
  const std::vector<int>& starting_cols = A.starting_cols();
  const Eigen::Matrix<double, 12, 12> dense_A = MakeSpdMatrix();
  std::vector<std::pair<int, int>> nonzero_lower_triangular_blocks{
      {0, 0}, {1, 1}, {2, 1}, {2, 2}, {3, 1}, {3, 3}};
  for (const auto& [a, b] : nonzero_lower_triangular_blocks) {
    A.AddToBlock(a, b,
                 dense_A.block(starting_cols[a], starting_cols[b],
                               block_sizes[a], block_sizes[b]));
  }
  return A;
}

/* Exercises the APIs of the wrapper and makes sure that it exposes what the
 underlying matrix reports. */
GTEST_TEST(EigenBlock3x3SparseSymmetricMatrixTest, Basic) {
  const double kTol = 4.0 * std::numeric_limits<double>::epsilon();

  const BlockSparseSymmetricMatrix3d A = MakeSparseSpdMatrix();
  const MatrixXd dense_A = A.MakeDenseMatrix();
  const EigenBlock3x3SparseSymmetricMatrix eigen_A(&A, Parallelism(true));

  EXPECT_EQ(eigen_A.rows(), A.rows());
  EXPECT_EQ(eigen_A.cols(), A.cols());
  EXPECT_TRUE(CompareMatrices(eigen_A.diagonal(), dense_A.diagonal(), kTol));
  EXPECT_EQ(eigen_A.outerSize(), A.cols());
  /* Test Multiply(). */
  const VectorXd x = VectorXd::LinSpaced(A.cols(), 0.0, 10.0);
  VectorXd y(A.rows());
  eigen_A.Multiply(x, &y);
  EXPECT_TRUE(
      CompareMatrices(y, dense_A * x, kTol, MatrixCompareType::relative));
  /* Test operator*() */
  EXPECT_TRUE(CompareMatrices(eigen_A * x, dense_A * x, kTol,
                              MatrixCompareType::relative));
}

/* Test the usage of the wrapper in Eigen::ConjugateGradient. */
GTEST_TEST(EigenBlock3x3SparseSymmetricMatrixTest, ScaledIdentityTwoBlock) {
  const double kTol = 1e-14;

  const BlockSparseSymmetricMatrix3d A = MakeSparseSpdMatrix();
  const MatrixXd dense_A = A.MakeDenseMatrix();
  const EigenBlock3x3SparseSymmetricMatrix eigen_A(&A, Parallelism(true));

  Eigen::ConjugateGradient<EigenBlock3x3SparseSymmetricMatrix,
                           Eigen::Lower | Eigen::Upper>
      cg;
  cg.setTolerance(kTol);
  cg.compute(eigen_A);
  EXPECT_EQ(cg.info(), Eigen::Success);

  const VectorXd b = VectorXd::LinSpaced(A.cols(), 0.0, 10.0);
  VectorXd x = cg.solve(b);
  EXPECT_EQ(cg.info(), Eigen::Success);
  EXPECT_TRUE(
      CompareMatrices(dense_A * x, b, kTol, MatrixCompareType::relative));
}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
