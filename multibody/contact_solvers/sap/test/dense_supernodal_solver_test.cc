#include "drake/multibody/contact_solvers/sap/dense_supernodal_solver.h"

#include <limits>
#include <tuple>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/unused.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::get;
using std::vector;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

constexpr double kEps = std::numeric_limits<double>::epsilon();

// Type used to represent a block diagonal matrix in two formats: 1) a dense
// matrix and 2) a std::vector of block diagonal entries.
typedef std::pair<MatrixXd, std::vector<MatrixXd>> DenseBlockDiagonalPair;

// Makes a block diagonal SPD matrix with three diagonal SPD blocks of size
// 2x2.
DenseBlockDiagonalPair Make6x6SpdBlockDiagonalMatrixOf2x2SpdMatrices() {
  MatrixXd A(6, 6);
  // clang-format off
  A << 1, 1, 0, 0, 0, 0,
       1, 5, 0, 0, 0, 0,
       0, 0, 4, 1, 0, 0,
       0, 0, 1, 4, 0, 0,
       0, 0, 0, 0, 4, 2,
       0, 0, 0, 0, 2, 5;
  // clang-format on
  std::vector<MatrixXd> blocks(3);
  blocks.at(0) = A.block<2, 2>(0, 0);
  blocks.at(1) = A.block<2, 2>(2, 2);
  blocks.at(2) = A.block<2, 2>(4, 4);
  return std::make_pair(A, blocks);
}

// Makes a block diagonal SPD matrix with three diagonal SPD blocks of size
// 3x3.
DenseBlockDiagonalPair Make9x9SpdBlockDiagonalMatrixOf3x3SpdMatrices() {
  MatrixXd A(9, 9);
  // clang-format off
  A << 1, 2, 2, 0, 0, 0, 0, 0, 0,
       2, 5, 3, 0, 0, 0, 0, 0, 0,
       2, 3, 8, 0, 0, 0, 0, 0, 0,
       0, 0, 0, 4, 1, 1, 0, 0, 0,
       0, 0, 0, 1, 4, 2, 0, 0, 0,
       0, 0, 0, 1, 2, 5, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 4, 1, 1,
       0, 0, 0, 0, 0, 0, 1, 4, 2,
       0, 0, 0, 0, 0, 0, 1, 2, 5;
  // clang-format on
  std::vector<MatrixXd> blocks(3);
  blocks.at(0) = A.block<3, 3>(0, 0);
  blocks.at(1) = A.block<3, 3>(3, 3);
  blocks.at(2) = A.block<3, 3>(6, 6);
  return std::make_pair(A, blocks);
}

// Helper to make block matrix triplets for an input dense matrix A.
// block_positions[b] corresponds to the (bi, bj) block indexes of the non-zero
// b-th block.
// dense_positions[b] corresponds to the (i, j) indexes of the top-left corner
// of the b-th block in the dense matrix A.
// block_sizes[b] corresponds to the number of rows (first) and columns (second)
// for the b-th block.
std::vector<BlockTriplet> MakeBlockTriplets(
    const MatrixXd& A, const std::vector<std::pair<int, int>>& block_positions,
    const std::vector<std::pair<int, int>>& dense_positions,
    const std::vector<std::pair<int, int>>& block_sizes) {
  DRAKE_DEMAND(block_positions.size() == dense_positions.size());
  DRAKE_DEMAND(block_positions.size() == block_sizes.size());
  const int num_blocks = ssize(block_positions);
  std::vector<BlockTriplet> triplets;
  for (int b = 0; b < num_blocks; ++b) {
    triplets.emplace_back(
        block_positions[b].first, block_positions[b].second,
        MatrixBlock<double>(
            A.block(dense_positions[b].first, dense_positions[b].second,
                    block_sizes[b].first, block_sizes[b].second)));
  }
  return triplets;
}

// Verifies the correct behavior of the NVIs provided by SuperNodalSolver,
// implemented by DenseSuperNodalSolver.
GTEST_TEST(DenseSuperNodalSolver, InterfaceTest) {
  const auto [A, blocks_of_A] = Make6x6SpdBlockDiagonalMatrixOf2x2SpdMatrices();

  // Build Jacobian matrix J.
  MatrixXd Jdense(9, 6);
  const int num_row_blocks_of_J = 3;
  const int num_col_blocks_of_J = 3;
  // clang-format off
  Jdense <<
       1, 2, 1, 2, 0, 0,
       0, 1, 0, 1, 0, 0,
       1, 3, 1, 3, 0, 0,
       0, 0, 0, 0, 1, 0,
       0, 0, 0, 0, 2, 0,
       0, 0, 0, 0, 2, 0,
       0, 0, 0, 0, 0, 1,
       0, 0, 0, 0, 0, 1,
       0, 0, 0, 0, 0, 3;
  const std::vector<BlockTriplet> Jtriplets = MakeBlockTriplets(Jdense,
      {{0, 0}, {1, 1}, {2, 2}},
      {{0, 0}, {3, 4}, {6, 5}},
      {{3, 4}, {3, 1}, {3, 1}});
  // clang-format on

  BlockSparseMatrixBuilder<double> builder(num_row_blocks_of_J,
                                           num_col_blocks_of_J, 3);
  for (const auto& block : Jtriplets) {
    builder.PushBlock(block.row, block.col, block.value);
  }
  BlockSparseMatrix<double> J = builder.Build();

  auto [G, blocks_of_G] = Make9x9SpdBlockDiagonalMatrixOf3x3SpdMatrices();

  DenseSuperNodalSolver solver(&blocks_of_A, &J);
  EXPECT_EQ(solver.GetSize(), 6);

  ASSERT_NO_THROW(solver.SetWeightMatrix(blocks_of_G));

  const MatrixXd H = solver.MakeFullMatrix();
  const MatrixXd H_expected = A + Jdense.transpose() * G * Jdense;
  EXPECT_TRUE(
      CompareMatrices(H, H_expected, kEps, MatrixCompareType::relative));

  EXPECT_TRUE(solver.Factor());
  VectorXd b = VectorXd::LinSpaced(A.rows(), -3.0, 12.0);
  VectorXd x = solver.Solve(b);
  VectorXd x_expected = H_expected.ldlt().solve(b);
  EXPECT_TRUE(
      CompareMatrices(x, x_expected, kEps, MatrixCompareType::relative));
}

GTEST_TEST(DenseSuperNodalSolver, BasicChecks) {
  const auto [A, blocks_of_A] = Make6x6SpdBlockDiagonalMatrixOf2x2SpdMatrices();

  // Build Jacobian matrix J.
  MatrixXd Jdense(9, 6);
  const int num_row_blocks_of_J = 3;
  const int num_col_blocks_of_J = 3;
  // clang-format off
  Jdense <<
       1, 2, 1, 2, 0, 0,
       0, 1, 0, 1, 0, 0,
       1, 3, 1, 3, 0, 0,
       0, 0, 0, 0, 1, 0,
       0, 0, 0, 0, 2, 0,
       0, 0, 0, 0, 2, 0,
       0, 0, 0, 0, 0, 1,
       0, 0, 0, 0, 0, 1,
       0, 0, 0, 0, 0, 3;
  const std::vector<BlockTriplet> Jtriplets = MakeBlockTriplets(Jdense,
      {{0, 0}, {1, 1}, {2, 2}},
      {{0, 0}, {3, 4}, {6, 5}},
      {{3, 4}, {3, 1}, {3, 1}});
  // clang-format on

  BlockSparseMatrixBuilder<double> builder(num_row_blocks_of_J,
                                           num_col_blocks_of_J, 3);
  for (auto& block : Jtriplets) {
    builder.PushBlock(block.row, block.col, block.value);
  }
  BlockSparseMatrix<double> J = builder.Build();

  // A is nullptr.
  {
    auto bad_constructor_call = [&J]() {
      DenseSuperNodalSolver solver(nullptr, &J);
    };
    DRAKE_EXPECT_THROWS_MESSAGE(bad_constructor_call(),
                                ".*condition 'A != nullptr' failed.");
  }

  // J is nullptr.
  {
    // N.B. Clang doesn't like to capture blocks_of_A because its a binding.
    auto bad_constructor_call = [&A_not_a_binding = blocks_of_A]() {
      DenseSuperNodalSolver solver(&A_not_a_binding, nullptr);
    };
    DRAKE_EXPECT_THROWS_MESSAGE(bad_constructor_call(),
                                ".*condition 'J != nullptr' failed.");
  }

  DenseSuperNodalSolver solver(&blocks_of_A, &J);
  EXPECT_EQ(solver.GetSize(), 6);

  // Calling Factor() before SetWeightMatrix() throws.
  {
    DRAKE_EXPECT_THROWS_MESSAGE(
        solver.Factor(), "Call to Factor\\(\\) failed: weight matrix not set.");
  }

  auto [G, blocks_of_G] = Make9x9SpdBlockDiagonalMatrixOf3x3SpdMatrices();
  ASSERT_NO_THROW(solver.SetWeightMatrix(blocks_of_G));

  // G is incompatible with Jacobian.
  {
    MatrixXd bad_block(2, 2);
    // clang-format off
    bad_block << 1, 1,
                 1, 5;
    // clang-format on
    std::vector<MatrixXd> bad_G = blocks_of_G;
    bad_G[1] = bad_block;
    DRAKE_EXPECT_THROWS_MESSAGE(solver.SetWeightMatrix(bad_G),
                                "Weight matrix incompatible with Jacobian.");
  }

  // Calling Solve() before Factor() throws.
  VectorXd b = VectorXd::LinSpaced(A.rows(), -3.0, 12.0);
  {
    DRAKE_EXPECT_THROWS_MESSAGE(
        solver.Solve(b),
        "Call to Solve\\(\\) failed: factorization not ready.");
  }

  // Calling MakeFullMatrix() after Factor() throws because the in-place
  // factorization overwrites the matrix.
  {
    solver.Factor();
    DRAKE_EXPECT_THROWS_MESSAGE(
        solver.MakeFullMatrix(),
        "Call to MakeFullMatrix\\(\\) failed: weight matrix not set or matrix "
        "has been factored in place.");
  }
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
