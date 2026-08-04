#include "drake/multibody/contact_solvers/schur_complement.h"

#include <limits>
#include <unordered_set>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace {

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::VectorXd;

constexpr double kTolerance = 4.0 * std::numeric_limits<double>::epsilon();

// clang-format off
Matrix3d A00() {
  return (Eigen::Matrix3d() << 11, 2, 3,
                               2, 50, 6,
                               3, 6, 190).finished();
}
Matrix3d A10() {
  return (Eigen::Matrix3d() << 1.1, 1.0, 0.9,
                               1.2, 1.5, 1.6,
                               1.3, 1.8, 1.9).finished();
}
Matrix3d A11() {
    return (Eigen::Matrix3d() << 110, 12, 13,
                                 12, 150, 16,
                                 13, 16, 190).finished();
}
Matrix3d A20() {
    return (Eigen::Matrix3d() << 11, 12, 13,
                                 14, 21, 16,
                                 17, 18, 29).finished();
}
Matrix3d A22() {
    return (Eigen::Matrix3d() << 210, 22, 23,
                                 22, 205, 26,
                                 23, 26, 290).finished();
}
// clang-format on

/* Makes a block sparse symmetric matrix with 3x3 dense blocks that looks like
   A =   A00 | A01 | A02
        -----------------
         A10 | A11 |  0
        -----------------
         A20 |  0  | A22
 where A02 = A20.transpose(). We choose values so that A is diagonally dominant
 and thus SPD. */
BlockSparseSymmetricMatrix3d MakeBlockSparseMatrix() {
  std::vector<std::vector<int>> sparsity_pattern;
  sparsity_pattern.emplace_back(std::vector<int>{0, 1, 2});
  sparsity_pattern.emplace_back(std::vector<int>{1});
  sparsity_pattern.emplace_back(std::vector<int>{2});
  BlockSparsityPattern block_pattern({{3, 3, 3}}, std::move(sparsity_pattern));
  BlockSparseSymmetricMatrix3d A(std::move(block_pattern));
  A.SetBlock(0, 0, A00());
  A.SetBlock(1, 0, A10());
  A.SetBlock(2, 0, A20());
  A.SetBlock(1, 1, A11());
  A.SetBlock(2, 2, A22());
  return A;
}

/* Constructs an arbitrary block sparse matrix (see MakeBlockSparseMatrix())

   A =   A00 | A01 | A02
        -----------------
         A10 | A11 |  0
        -----------------
         A20 |  0  | A22

and returns the Schur complement of the A11 block. */
SchurComplement MakeSchurComplement() {
  BlockSparseSymmetricMatrix3d block_sparse_matrix = MakeBlockSparseMatrix();
  const std::unordered_set<int> eliminated_blocks = {1};
  return SchurComplement(block_sparse_matrix, eliminated_blocks);
}

GTEST_TEST(SchurComplementTest, GetDComplement) {
  const SchurComplement schur_complement = MakeSchurComplement();
  const MatrixXd S = schur_complement.get_D_complement();

  MatrixXd C = MatrixXd(6, 6);
  C.topLeftCorner<3, 3>() = A00();
  C.bottomLeftCorner<3, 3>() = A20();
  C.topRightCorner<3, 3>() = A20().transpose();
  C.bottomRightCorner<3, 3>() = A22();
  MatrixXd D = A11();
  MatrixXd B = MatrixXd::Zero(3, 6);
  B.topLeftCorner<3, 3>() = A10();

  const MatrixXd expected = C - B.transpose() * D.llt().solve(B);
  EXPECT_TRUE(CompareMatrices(S, expected, kTolerance));
}

GTEST_TEST(SchurComplementTest, SolveForX) {
  const SchurComplement schur_complement = MakeSchurComplement();
  const VectorXd y = VectorXd::LinSpaced(6, 0.0, 12.0);
  const VectorXd x = schur_complement.SolveForX(y);

  MatrixXd D = A11();
  MatrixXd B = MatrixXd::Zero(3, 6);
  B.topLeftCorner<3, 3>() = A10();
  /* The system of equation reads
       Dx  + By = 0 (1)
       Bᵀx + Ay = a (2)
     Using equation (1), we get
       x = -D⁻¹By */
  const VectorXd expected_x = D.llt().solve(-B * y);
  EXPECT_TRUE(CompareMatrices(x, expected_x, kTolerance));
}

GTEST_TEST(SchurComplementTest, Solve) {
  const SchurComplement schur_complement = MakeSchurComplement();
  const VectorXd b = VectorXd::LinSpaced(9, 0.0, 12.0);
  const VectorXd z = schur_complement.Solve(b);

  const MatrixXd A = MakeBlockSparseMatrix().MakeDenseMatrix();
  const VectorXd expected_z = A.llt().solve(b);
  EXPECT_TRUE(CompareMatrices(z, expected_z, kTolerance));
}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
