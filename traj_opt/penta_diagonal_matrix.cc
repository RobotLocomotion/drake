#include "drake/traj_opt/penta_diagonal_matrix.h"

#include <utility>

#include "drake/common/drake_assert.h"

using Eigen::MatrixXd;

namespace drake {
namespace multibody {
namespace trajopt {
namespace internal {

PentaDiagonalMatrix::PentaDiagonalMatrix(std::vector<Eigen::MatrixXd> A,
                                         std::vector<Eigen::MatrixXd> B,
                                         std::vector<Eigen::MatrixXd> C,
                                         std::vector<Eigen::MatrixXd> D,
                                         std::vector<Eigen::MatrixXd> E)
    : A_(std::move(A)),
      B_(std::move(B)),
      C_(std::move(C)),
      D_(std::move(D)),
      E_(std::move(E)) {
  // Minimum sanity check.
  DRAKE_DEMAND(A_.size() == B_.size());
  DRAKE_DEMAND(A_.size() == C_.size());
  DRAKE_DEMAND(A_.size() == D_.size());
  DRAKE_DEMAND(A_.size() == E_.size());
  // Thorough sanity check in debug builds only.
  DRAKE_ASSERT(VerifySizes());
}

PentaDiagonalMatrix::PentaDiagonalMatrix(std::vector<Eigen::MatrixXd> A,
                                         std::vector<Eigen::MatrixXd> B,
                                         std::vector<Eigen::MatrixXd> C)
    : A_(std::move(A)), B_(std::move(B)), C_(std::move(C)) {
  // Minimum sanity check.
  DRAKE_DEMAND(A_.size() == B_.size());
  DRAKE_DEMAND(A_.size() == C_.size());

  // Alocate space for D and E.
  // TODO(amcastro-tri): Consider not allocating space for D/E in the symmetric
  // case.
  const int size = A_.size();
  D_ = B_;
  E_ = A_;

  // We overwrite the (strictly) upper triangular part of C with its lower
  // triangular part.
  for (int i = 0; i < size; ++i) {
    C_[i].triangularView<Eigen::StrictlyUpper>() = C_[i].transpose();
  }

  // D = B.
  // N.B. The first entry in B is zero and we skip its copy.
  // The last entry in D_ is defined to be zero.
  if (size >= 2) {
    for (int i = 0; i < size - 1; ++i) {
      D_[i] = B_[i + 1].transpose();
    }
    D_[size - 1].setZero();
  }

  // E = A.
  // N.B. The first two entries in A are zero and we skip their copy.
  // The last two entries in E are defined to be zero.
  if (size >= 3) {
    for (int i = 0; i < size - 2; ++i) {
      E_[i] = A_[i + 2].transpose();
    }
    E_[size - 1].setZero();
    E_[size - 2].setZero();
  }

  is_symmetric_ = true;

  // Thorough sanity check in debug builds only.
  // N.B. We place it here at the bottom since we need all five bands to be
  // properly initialized.
  DRAKE_ASSERT(VerifySizes());
}

PentaDiagonalMatrix PentaDiagonalMatrix::MakeIdentity(int num_blocks,
                                                      int block_size) {
  const MatrixXd Z = MatrixXd::Zero(block_size, block_size);
  const MatrixXd Id = MatrixXd::Identity(block_size, block_size);
  std::vector<MatrixXd> A(num_blocks, Z);
  std::vector<MatrixXd> B(num_blocks, Z);
  std::vector<MatrixXd> C(num_blocks, Id);
  return PentaDiagonalMatrix(std::move(A), std::move(B), std::move(C));
}

bool PentaDiagonalMatrix::VerifySizes() const {
  const int k = block_size();
  if (!VerifyAllBlocksOfSameSize(A_, k)) return false;
  if (!VerifyAllBlocksOfSameSize(B_, k)) return false;
  if (!VerifyAllBlocksOfSameSize(C_, k)) return false;
  if (!VerifyAllBlocksOfSameSize(D_, k)) return false;
  if (!VerifyAllBlocksOfSameSize(E_, k)) return false;
  return true;
}

PentaDiagonalMatrix PentaDiagonalMatrix::MakeSymmetricFromLowerDense(
    const Eigen::MatrixXd& M, int num_blocks, int block_size) {
  const MatrixXd Z = MatrixXd::Zero(block_size, block_size);
  std::vector<MatrixXd> A(num_blocks, Z);
  std::vector<MatrixXd> B(num_blocks, Z);
  std::vector<MatrixXd> C(num_blocks, Z);
  for (int i = 0; i < num_blocks; ++i) {
    if (i >= 2)
      A[i] =
          M.block(i * block_size, (i - 2) * block_size, block_size, block_size);
    if (i >= 1)
      B[i] =
          M.block(i * block_size, (i - 1) * block_size, block_size, block_size);
    C[i] = M.block(i * block_size, i * block_size, block_size, block_size);
  }
  return PentaDiagonalMatrix(std::move(A), std::move(B), std::move(C));
}

MatrixXd PentaDiagonalMatrix::MakeDense() const {
  const int num_blocks = block_cols();
  MatrixXd M = MatrixXd::Zero(rows(), cols());
  for (int i = 0; i < num_blocks; ++i) {
    if (i >= 2)
      M.block(i * block_size(), (i - 2) * block_size(), block_size(),
              block_size()) = A_[i];
    if (i >= 1)
      M.block(i * block_size(), (i - 1) * block_size(), block_size(),
              block_size()) = B_[i];
    M.block(i * block_size(), i * block_size(), block_size(), block_size()) =
        C_[i];
    if (i < num_blocks - 1)
      M.block(i * block_size(), (i + 1) * block_size(), block_size(),
              block_size()) = D_[i];
    if (i < num_blocks - 2)
      M.block(i * block_size(), (i + 2) * block_size(), block_size(),
              block_size()) = E_[i];
  }
  return M;
}

bool PentaDiagonalMatrix::VerifyAllBlocksOfSameSize(
    const std::vector<Eigen::MatrixXd>& X, int size) {
  for (const Eigen::MatrixXd& Xblock : X) {
    if (Xblock.rows() != size || Xblock.cols() != size) return false;
  }
  return true;
}

}  // namespace internal
}  // namespace trajopt
}  // namespace multibody
}  // namespace drake
