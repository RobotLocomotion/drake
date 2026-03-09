#include "drake/multibody/contact_solvers/block_sparse_cholesky_solver.h"

#include <memory>
#include <numeric>
#include <unordered_set>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/unused.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace {

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::MatrixXi;
using Eigen::VectorXd;

/* Makes an arbitrary SPD matrix with the following nonzero pattern.

  X X | O O O | O O O O | O O O
  X X | O O O | O O O O | O O O
  --- | ----- |---------| -----
  O O | X X X | X X X X | X X X
  O O | X X X | X X X X | X X X
  O O | X X X | X X X X | X X X
  --- | ----- |---------| -----
  O O | X X X | X X X X | O O O
  O O | X X X | X X X X | O O O
  O O | X X X | X X X X | O O O
  O O | X X X | X X X X | O O O
  --- | ----- |---------| -----
  O O | X X X | O O O O | X X X
  O O | X X X | O O O O | X X X
  O O | X X X | O O O O | X X X

 The scaling factor can be used to control the values of each nonzero entry. */
Eigen::Matrix<double, 12, 12> MakeSpdMatrix(double scale) {
  Eigen::Matrix<double, 12, 12> A;
  for (int i = 0; i < 12; ++i) {
    for (int j = 0; j < 12; ++j) {
      A(i, j) = 0.03 * i + 0.02 * j;
    }
  }
  Eigen::Matrix<double, 12, 12> I = Eigen::Matrix<double, 12, 12>::Identity();
  /* Add a large diagonal entry to ensure diagonal dominance. */
  Eigen::Matrix<double, 12, 12> result = scale * (A * A.transpose() + 12 * I);
  /* Zeroing off-diagonal entries of a diagonally dominant matrix does not
   affect diagonal dominance, which in turns implies SPDness. */
  result.block<10, 2>(2, 0).setZero();
  result.block<2, 10>(0, 2).setZero();
  result.block<3, 4>(9, 5).setZero();
  result.block<4, 3>(5, 9).setZero();
  return result;
}

/* Makes an arbitrary SPD sparse matrix. */
BlockSparseSymmetricMatrixXd MakeSparseSpdMatrix(double scale = 1.0) {
  std::vector<std::vector<int>> sparsity;
  sparsity.emplace_back(std::vector<int>{0});
  sparsity.emplace_back(std::vector<int>{1, 2, 3});
  sparsity.emplace_back(std::vector<int>{2});
  sparsity.emplace_back(std::vector<int>{3});
  std::vector<int> block_sizes = {2, 3, 4, 3};
  BlockSparsityPattern block_pattern(block_sizes, sparsity);

  BlockSparseSymmetricMatrixXd A(std::move(block_pattern));
  const std::vector<int>& starting_cols = A.starting_cols();
  const Eigen::Matrix<double, 12, 12> dense_A = MakeSpdMatrix(scale);
  std::vector<std::pair<int, int>> nonzero_lower_triangular_blocks{
      {0, 0}, {1, 1}, {2, 1}, {2, 2}, {3, 1}, {3, 3}};
  for (const auto& [a, b] : nonzero_lower_triangular_blocks) {
    A.AddToBlock(a, b,
                 dense_A.block(starting_cols[a], starting_cols[b],
                               block_sizes[a], block_sizes[b]));
  }
  return A;
}

GTEST_TEST(BlockSparseCholeskySolverTest, Solve) {
  BlockSparseCholeskySolver<MatrixXd> solver;
  BlockSparseSymmetricMatrixXd A = MakeSparseSpdMatrix();
  MatrixX<double> dense_A = A.MakeDenseMatrix();
  EXPECT_EQ(solver.solver_mode(),
            BlockSparseCholeskySolver<MatrixXd>::SolverMode::kEmpty);
  solver.SetMatrix(A);
  EXPECT_EQ(solver.solver_mode(),
            BlockSparseCholeskySolver<MatrixXd>::SolverMode::kAnalyzed);
  bool success = solver.Factor();
  EXPECT_TRUE(success);
  EXPECT_EQ(solver.solver_mode(),
            BlockSparseCholeskySolver<MatrixXd>::SolverMode::kFactored);

  const VectorXd b1 = VectorXd::LinSpaced(A.cols(), 0.0, 1.0);
  const VectorXd x1 = solver.Solve(b1);
  const VectorXd expected_x1 = dense_A.llt().solve(b1);
  EXPECT_TRUE(CompareMatrices(x1, expected_x1, 1e-13));

  /* Solve for a different right hand side without refactoring. */
  const VectorXd b2 = VectorXd::LinSpaced(A.cols(), 0.0, 10.0);
  const VectorXd x2 = solver.Solve(b2);
  const VectorXd expected_x2 = dense_A.llt().solve(b2);
  EXPECT_TRUE(CompareMatrices(x2, expected_x2, 1e-13));

  /* SolveInPlace variant. */
  const VectorXd b3 = VectorXd::LinSpaced(A.cols(), 7.0, 8.0);
  const VectorXd expected_x3 = dense_A.llt().solve(b3);
  VectorXd x3 = b3;
  solver.SolveInPlace(&x3);
  EXPECT_TRUE(CompareMatrices(x3, expected_x3, 1e-13));

  /* Update the matrix with different numeric values but the same sparsity
   pattern. */
  BlockSparseSymmetricMatrixXd A2 = MakeSparseSpdMatrix(10);
  MatrixX<double> dense_A2 = A2.MakeDenseMatrix();
  solver.UpdateMatrix(A2);
  success = solver.Factor();
  EXPECT_TRUE(success);
  const VectorXd b4 = VectorXd::LinSpaced(A2.cols(), 0.0, 10.0);
  const VectorXd x4 = solver.Solve(b4);
  const VectorXd expected_x4 = dense_A2.llt().solve(b4);
  EXPECT_TRUE(CompareMatrices(x4, expected_x4, 1e-13));
}

GTEST_TEST(BlockSparseCholeskySolverTest, FailureDueToNonSpdness) {
  std::vector<std::vector<int>> sparsity;
  sparsity.emplace_back(std::vector<int>{0, 1});
  sparsity.emplace_back(std::vector<int>{1});
  std::vector<int> block_sizes = {4, 3};
  BlockSparsityPattern block_pattern(block_sizes, sparsity);
  BlockSparseSymmetricMatrixXd A(std::move(block_pattern));
  A.AddToBlock(0, 0, -Matrix4d::Identity());
  A.AddToBlock(1, 1, -Matrix3d::Identity());

  BlockSparseCholeskySolver<MatrixXd> solver;
  solver.SetMatrix(A);
  EXPECT_EQ(solver.solver_mode(),
            BlockSparseCholeskySolver<MatrixXd>::SolverMode::kAnalyzed);
  /* Calling Factor() fails. */
  {
    const bool success = solver.Factor();
    EXPECT_FALSE(success);
    EXPECT_EQ(solver.solver_mode(),
              BlockSparseCholeskySolver<MatrixXd>::SolverMode::kEmpty);
  }
  /* Calling FactorAndCalcSchurComplement() also fails. */
  {
    for (const std::unordered_set<int>& eliminated_blocks :
         std::vector<std::unordered_set<int>>{std::unordered_set<int>{},
                                              std::unordered_set<int>{0},
                                              std::unordered_set<int>{0, 1}}) {
      const std::optional<MatrixXd> schur_complement =
          solver.FactorAndCalcSchurComplement(A, eliminated_blocks);
      EXPECT_FALSE(schur_complement.has_value());
      EXPECT_EQ(solver.solver_mode(),
                BlockSparseCholeskySolver<MatrixXd>::SolverMode::kEmpty);
    }
  }
}

GTEST_TEST(BlockSparseCholeskySolverTest, FactorBeforeSetMatrixThrows) {
  BlockSparseCholeskySolver<MatrixXd> solver;
  EXPECT_THROW(unused(solver.Factor()), std::exception);
}

GTEST_TEST(BlockSparseCholeskySolverTest, SolveBeforeFactorThrows) {
  BlockSparseCholeskySolver<MatrixXd> solver;
  BlockSparseSymmetricMatrixXd A = MakeSparseSpdMatrix();
  solver.SetMatrix(A);
  VectorXd b = VectorXd::LinSpaced(A.cols(), 0.0, 10.0);
  EXPECT_THROW(solver.Solve(b), std::exception);
  EXPECT_THROW(solver.SolveInPlace(&b), std::exception);
}

GTEST_TEST(BlockSparseCholeskySolverTest, PermutationMatrix) {
  BlockSparseCholeskySolver<MatrixXd> solver;
  BlockSparseSymmetricMatrixXd A = MakeSparseSpdMatrix();
  const MatrixXd A_dense = A.MakeDenseMatrix();
  solver.SetMatrix(A);
  /* Trying to get L before factorization is an exception. */
  EXPECT_THROW(solver.L(), std::exception);
  const bool success = solver.Factor();
  EXPECT_TRUE(success);
  const MatrixXd L = solver.L().MakeDenseMatrix();
  const Eigen::PermutationMatrix<Eigen::Dynamic> P =
      solver.CalcPermutationMatrix();
  const MatrixXd lhs = L * L.transpose();
  const MatrixXd rhs = P * A_dense * P.transpose();
  EXPECT_TRUE(CompareMatrices(lhs, rhs, 1e-14));
}

GTEST_TEST(BlockSparseCholeskySolverTest, PermutationMatrixPrecondition) {
  BlockSparseCholeskySolver<MatrixXd> solver;
  BlockSparseSymmetricMatrixXd A = MakeSparseSpdMatrix();
  /* CalcPermutationMatrix() before setting the matrix throws. */
  EXPECT_THROW(solver.CalcPermutationMatrix(), std::exception);
  /* After setting the matrix, CalcPermutatoinMatrix() returns the same result
   before and after factorization. */
  solver.SetMatrix(A);
  const Eigen::PermutationMatrix<Eigen::Dynamic> P0 =
      solver.CalcPermutationMatrix();
  const bool success = solver.Factor();
  EXPECT_TRUE(success);
  const Eigen::PermutationMatrix<Eigen::Dynamic> P1 =
      solver.CalcPermutationMatrix();
  EXPECT_EQ(MatrixXi(P0), MatrixXi(P1));
}

GTEST_TEST(BlockSparseCholeskySolverTest, SolverModeAfterMove) {
  BlockSparseCholeskySolver<MatrixXd> solver;
  BlockSparseSymmetricMatrixXd A = MakeSparseSpdMatrix();
  solver.SetMatrix(A);
  EXPECT_EQ(solver.solver_mode(),
            BlockSparseCholeskySolver<MatrixXd>::SolverMode::kAnalyzed);
  BlockSparseCholeskySolver<MatrixXd> new_solver(std::move(solver));
  EXPECT_EQ(new_solver.solver_mode(),
            BlockSparseCholeskySolver<MatrixXd>::SolverMode::kAnalyzed);
  /* The mode of the old solver resets to kEmpty. */
  EXPECT_EQ(solver.solver_mode(),
            BlockSparseCholeskySolver<MatrixXd>::SolverMode::kEmpty);
}

GTEST_TEST(BlockSparseCholeskySolverTest, FactorAndCalcSchurComplement) {
  BlockSparseCholeskySolver<MatrixXd> solver;
  BlockSparseSymmetricMatrixXd M = MakeSparseSpdMatrix();
  const int kNumBlocks = 4;
  /* All blocks are eliminated. */
  {
    std::vector<int> eliminated_blocks(kNumBlocks);
    std::iota(eliminated_blocks.begin(), eliminated_blocks.end(), 0);
    const std::optional<MatrixXd> schur_complement =
        solver.FactorAndCalcSchurComplement(
            M, std::unordered_set<int>(eliminated_blocks.begin(),
                                       eliminated_blocks.end()));
    EXPECT_TRUE(schur_complement.has_value());
    EXPECT_EQ(schur_complement.value(), MatrixXd::Zero(0, 0));
    EXPECT_EQ(solver.solver_mode(),
              BlockSparseCholeskySolver<MatrixXd>::SolverMode::kFactored);
  }
  /* None of the blocks is eliminated. */
  {
    const std::optional<MatrixXd> schur_complement =
        solver.FactorAndCalcSchurComplement(M, std::unordered_set<int>());
    EXPECT_TRUE(schur_complement.has_value());
    EXPECT_TRUE(CompareMatrices(schur_complement.value(), M.MakeDenseMatrix()));
    EXPECT_EQ(solver.solver_mode(),
              BlockSparseCholeskySolver<MatrixXd>::SolverMode::kFactored);
  }
  /* Some of the blocks are eliminated. */
  {
    std::unordered_set<int> eliminated_blocks = {0, 1, 3};
    const std::optional<MatrixXd> schur_complement =
        solver.FactorAndCalcSchurComplement(M, eliminated_blocks);
    EXPECT_TRUE(schur_complement.has_value());
    const MatrixXd dense = M.MakeDenseMatrix();
    MatrixXd A = dense.block<4, 4>(5, 5);
    MatrixXd D = MatrixXd::Zero(8, 8);
    D.topLeftCorner(2, 2) = dense.topLeftCorner(2, 2);
    D.block<3, 3>(2, 2) = dense.block<3, 3>(2, 2);
    D.block<3, 3>(2, 5) = dense.block<3, 3>(2, 9);
    D.block<3, 3>(5, 2) = dense.block<3, 3>(9, 2);
    D.bottomRightCorner(3, 3) = dense.bottomRightCorner(3, 3);
    MatrixXd B = MatrixXd::Zero(8, 4);
    B.block<3, 4>(2, 0) = dense.block<3, 4>(2, 5);
    MatrixXd expected_schur_complement = A - B.transpose() * D.llt().solve(B);
    EXPECT_TRUE(CompareMatrices(schur_complement.value(),
                                expected_schur_complement, 1e-14));
    EXPECT_EQ(solver.solver_mode(),
              BlockSparseCholeskySolver<MatrixXd>::SolverMode::kFactored);
  }
}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
