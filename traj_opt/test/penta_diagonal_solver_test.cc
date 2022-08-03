#include "drake/traj_opt/penta_diagonal_solver.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/traj_opt/penta_diagonal_matrix.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace traj_opt {
namespace internal {

GTEST_TEST(PentaDiagonalMatrixTest, SymmetricMatrixEmpty) {
  const std::vector<MatrixXd> empty_diagonal;
  PentaDiagonalMatrix M(empty_diagonal, empty_diagonal, empty_diagonal);
  EXPECT_EQ(M.rows(), 0);
}

GTEST_TEST(PentaDiagonalMatrixTest, MutateMatrix) {
  const int k = 3;
  PentaDiagonalMatrix M(5, k);
  EXPECT_TRUE(M.is_symmetric());
  EXPECT_EQ(M.block_rows(), 5);
  EXPECT_EQ(M.block_cols(), 5);
  EXPECT_EQ(M.block_size(), 3);
  EXPECT_EQ(M.rows(), 15);
  EXPECT_EQ(M.cols(), 15);

  const MatrixXd B1 = 1.5 * MatrixXd::Ones(k, k);
  const MatrixXd B2 = 2.1 * MatrixXd::Ones(k, k);
  const MatrixXd B3 = -12.8 * MatrixXd::Ones(k, k);
  const MatrixXd B4 = 1.8 * MatrixXd::Ones(k, k);
  const MatrixXd B5 = 15.3 * MatrixXd::Ones(k, k);
  const std::vector<MatrixXd> some_diagonal = {B1, B2, B3, B3, B5};

  // Mutate diagonals.
  EXPECT_NE(M.A(), some_diagonal);
  M.mutable_A() = some_diagonal;
  EXPECT_EQ(M.A(), some_diagonal);

  EXPECT_NE(M.B(), some_diagonal);
  M.mutable_B() = some_diagonal;
  EXPECT_EQ(M.B(), some_diagonal);

  EXPECT_NE(M.C(), some_diagonal);
  M.mutable_C() = some_diagonal;
  EXPECT_EQ(M.C(), some_diagonal);

  // These throw since M is diagonal and it only allows mutating the lower
  // diagonals.
  EXPECT_THROW(M.mutable_D(), std::exception);
  EXPECT_THROW(M.mutable_E(), std::exception);
}

GTEST_TEST(PentaDiagonalMatrixTest, SymmetricMatrix) {
  const int k = 5;
  const MatrixXd Z = 1.5 * MatrixXd::Zero(k, k);
  const MatrixXd B1 = 1.5 * MatrixXd::Ones(k, k);
  const MatrixXd B2 = 2.1 * MatrixXd::Ones(k, k);
  const MatrixXd B3 = -12.8 * MatrixXd::Ones(k, k);
  const MatrixXd B4 = 1.8 * MatrixXd::Ones(k, k);
  const MatrixXd B5 = 15.3 * MatrixXd::Ones(k, k);
  const MatrixXd B6 = 7.1 * MatrixXd::Ones(k, k);
  PentaDiagonalMatrix M({Z, Z, B1}, {Z, B2, B3}, {B4, B5, B6});
  EXPECT_EQ(M.rows(), k * 3);
  EXPECT_EQ(M.block_rows(), 3);
  // Verify M is symmetric and is properly zero padded.
  EXPECT_EQ(M.D()[0], M.B()[1]);
  EXPECT_EQ(M.D()[1], M.B()[2]);
  EXPECT_EQ(M.D()[2], Z);
  EXPECT_EQ(M.E()[0], M.A()[2]);
  EXPECT_EQ(M.E()[1], Z);
  EXPECT_EQ(M.E()[2], Z);
}

GTEST_TEST(PentaDiagonalMatrixTest, SolveIdentity) {
  const int block_size = 3;
  const int num_blocks = 5;
  const int size = num_blocks * block_size;
  const PentaDiagonalMatrix H =
      PentaDiagonalMatrix::MakeIdentity(num_blocks, block_size);
  PentaDiagonalFactorization Hchol(H);
  EXPECT_EQ(Hchol.status(), PentaDiagonalFactorizationStatus::kSuccess);

  const VectorXd b = VectorXd::LinSpaced(size, -3, 12.4);
  VectorXd x = b;
  Hchol.SolveInPlace(&x);

  EXPECT_EQ(x, b);
}

GTEST_TEST(PentaDiagonalMatrixTest, SolveBlockDiagonal) {
  const int block_size = 3;
  const int num_blocks = 5;
  const int size = num_blocks * block_size;
  const MatrixXd I = MatrixXd::Identity(block_size, block_size);
  const MatrixXd Z = MatrixXd::Zero(block_size, block_size);
  const MatrixXd random_block = MatrixXd::Random(block_size, block_size);
  const MatrixXd B1 = 2.1 * I + random_block * random_block.transpose();
  const MatrixXd B2 = 3.5 * I + random_block * random_block.transpose();
  const MatrixXd B3 = 0.2 * I + random_block * random_block.transpose();

  std::vector<MatrixXd> A(num_blocks, Z);
  std::vector<MatrixXd> B(num_blocks, Z);
  std::vector<MatrixXd> C{B1, B2, B3, B1, B3};
  const PentaDiagonalMatrix H(std::move(A), std::move(B), std::move(C));
  const MatrixXd Hdense = H.MakeDense();

  PentaDiagonalFactorization Hchol(H);
  EXPECT_EQ(Hchol.status(), PentaDiagonalFactorizationStatus::kSuccess);
  const VectorXd b = VectorXd::LinSpaced(size, -3, 12.4);
  VectorXd x = b;
  Hchol.SolveInPlace(&x);

  // Reference solution computed with Eigen, dense.
  const VectorXd x_expected = Hdense.ldlt().solve(b);

  const double kTolerance = std::numeric_limits<double>::epsilon() * size;
  EXPECT_TRUE(
      CompareMatrices(x, x_expected, kTolerance, MatrixCompareType::relative));
}

GTEST_TEST(PentaDiagonalMatrixTest, SolveTriDiagonal) {
  const int block_size = 3;
  const int num_blocks = 5;
  const int size = num_blocks * block_size;
  const MatrixXd I = MatrixXd::Identity(block_size, block_size);
  const MatrixXd Z = MatrixXd::Zero(block_size, block_size);
  const MatrixXd random_block = MatrixXd::Random(block_size, block_size);
  const MatrixXd B1 = 2.1 * I + random_block * random_block.transpose();
  const MatrixXd B2 = 3.5 * I + random_block * random_block.transpose();
  const MatrixXd B3 = 0.2 * I + random_block * random_block.transpose();
  const MatrixXd B4 = 1.3 * I + random_block * random_block.transpose();

  std::vector<MatrixXd> A(num_blocks, Z);
  std::vector<MatrixXd> B{Z, B1, B2, B3, B4};
  std::vector<MatrixXd> C{B1, B2, B3, B1, B3};
  const PentaDiagonalMatrix H(std::move(A), std::move(B), std::move(C));
  const MatrixXd Hdense = H.MakeDense();

  PentaDiagonalFactorization Hchol(H);
  EXPECT_EQ(Hchol.status(), PentaDiagonalFactorizationStatus::kSuccess);
  const VectorXd b = VectorXd::LinSpaced(size, -3, 12.4);
  VectorXd x = b;
  Hchol.SolveInPlace(&x);

  // Reference solution computed with Eigen, dense.
  const VectorXd x_expected = Hdense.ldlt().solve(b);

  const double kTolerance = std::numeric_limits<double>::epsilon() * size;
  EXPECT_TRUE(
      CompareMatrices(x, x_expected, kTolerance, MatrixCompareType::relative));
}

GTEST_TEST(PentaDiagonalMatrixTest, SolvePentaDiagonal) {
  const int block_size = 2;
  const int num_blocks = 5;
  const int size = num_blocks * block_size;
  // Generate and SPD matrix.
  const MatrixXd A = MatrixXd::Random(size, size);
  const MatrixXd P = MatrixXd::Identity(size, size) + A * A.transpose();

  // Generate a penta-diagonal SPD matrix. Ignore off-diagonal elements of
  // Hdense outside the 5-diagonal band.
  const PentaDiagonalMatrix H =
      PentaDiagonalMatrix::MakeSymmetricFromLowerDense(P, num_blocks,
                                                       block_size);
  const MatrixXd Hdense = H.MakeDense();

  PentaDiagonalFactorization Hchol(H);
  EXPECT_EQ(Hchol.status(), PentaDiagonalFactorizationStatus::kSuccess);
  const VectorXd b = VectorXd::LinSpaced(size, -3, 12.4);
  VectorXd x = b;
  Hchol.SolveInPlace(&x);

  // Reference solution computed with Eigen, dense.
  const VectorXd x_expected = Hdense.ldlt().solve(b);

  const double kTolerance = std::numeric_limits<double>::epsilon() * size;
  EXPECT_TRUE(
      CompareMatrices(x, x_expected, kTolerance, MatrixCompareType::relative));
}

}  // namespace internal
}  // namespace traj_opt
}  // namespace drake
