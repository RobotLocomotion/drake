#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/traj_opt/penta_diagonal_matrix.h"

namespace drake {
namespace multibody {
namespace trajopt {
namespace internal {

/// Status reported by PentaDiagonalFactorization::status().
enum class PentaDiagonalFactorizationStatus {
  // Successful computation.
  kSuccess = 0,

  // Factorization failed. Typically the block diagonal factorization fails.
  kFailure = 1,
};

/// Implements the Thomas algorithm for the solution of block structured
/// penta-diagonal system of equations, [Benkert and Fischer, 2007].
///
/// [Benkert and Fischer, 2007] Benkert, K. and Fischer, R., 2007, May. An
/// efficient implementation of the Thomas-algorithm for block penta-diagonal
/// systems on vector computers. In International Conference on Computational
/// Science (pp. 144-151). Springer, Berlin, Heidelberg.
///
/// @tparm kBlockSize The size of the diagonal blocks if known at compile time.
/// It defaults to dynamics block sizes by default.
template <int kBlockSize = Eigen::Dynamic>
class PentaDiagonalFactorization {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PentaDiagonalFactorization);

  using BlockType = Eigen::Matrix<double, kBlockSize, kBlockSize>;

  /// Peforms the factorization of M to solve the system of equations M⋅x = b.
  /// This constructor only throws if the matrix is not symmetric. Call status()
  /// to check whether the factorization succeeded or not, typically because M
  /// is not SPD.
  ///
  /// @pre Matrix M is symmetric and positive definite.
  /// @throws if M is not symmetric.
  explicit PentaDiagonalFactorization(const PentaDiagonalMatrix& M);

  /// Reports whether the factorization succeeded or not.
  PentaDiagonalFactorizationStatus status() const { return status_; }

  int size() const { return factorization_.size; }

  /// Solves M⋅x = b into b. The factorization of M is reused on multiple calls
  /// to this method.
  void SolveInPlace(EigenPtr<Eigen::VectorXd> b);

 private:
  // Struct to store the factorization of matrix M.
  struct Factorization {
    void Resize(int num_blocks_in, int block_size_in) {
      size = num_blocks_in * block_size_in;
      num_blocks = num_blocks_in;
      block_size = block_size_in;

      A.resize(num_blocks, BlockType(block_size, block_size));
      G.resize(num_blocks, BlockType(block_size, block_size));
      Ginv.resize(num_blocks);
      K.resize(num_blocks, BlockType(block_size, block_size));
      // In math notation, we need access to Y from i=-1 to i=n with Y[-1] and
      // Y[0] defined to be zero. Therefore its size is n+2. In C++ we store
      // i=-1 in the very first entry of Y. It turns out that the final block
      // Y[n] is always zero.
      Ym2.resize(num_blocks + 2, BlockType(block_size, block_size));
      // In math notation, we need access to Z from i=-1 to i=n with Z[-1] and
      // Z[0] defined to be zero. Therefore its size is n+2. In C++ we store
      // i=-1 in the very first entry of Z. The last two blocks of Z are always
      // zero.
      Zm2.resize(num_blocks + 2, BlockType(block_size, block_size));
    }

    int size{0};
    int block_size{0};
    int num_blocks{0};
    std::vector<BlockType> A;
    std::vector<BlockType> G;
    std::vector<Eigen::LDLT<BlockType>> Ginv;
    std::vector<BlockType> K;
    std::vector<BlockType> Ym2;  // The very first entry is i=-1 (one-based).
    std::vector<BlockType> Zm2;  // The very first entry is i=-1 (one-based).
  };

  void Factorize(const PentaDiagonalMatrix& M);

  Factorization factorization_;
  Eigen::VectorXd r_;  // Convenient storage for the Solve() stage.
  PentaDiagonalFactorizationStatus status_{
      PentaDiagonalFactorizationStatus::kFailure};
};

template <int kBlockSize>
PentaDiagonalFactorization<kBlockSize>::PentaDiagonalFactorization(
    const PentaDiagonalMatrix& M) {
  Factorize(M);
  r_.resize(M.block_size() * (M.block_rows() + 2));
}

template <int kBlockSize>
void PentaDiagonalFactorization<kBlockSize>::Factorize(
    const PentaDiagonalMatrix& M) {
  DRAKE_DEMAND(M.is_symmetric());

  const int k = M.block_size();
  const int num_blocks = M.block_rows();
  factorization_.Resize(num_blocks, k);
  factorization_.A = M.A();

  // Convenient aliases.
  auto& A = factorization_.A;
  auto& G = factorization_.G;
  auto& Ginv = factorization_.Ginv;
  auto& K = factorization_.K;
  auto& Ym2 = factorization_.Ym2;
  auto& Zm2 = factorization_.Zm2;

  // Initialize boundaries.
  Ym2[0].setZero();  // i=-1 in 1-based math index.
  Ym2[1].setZero();  // i= 0 in 1-based math index.
  Zm2[0].setZero();  // i=-1 in 1-based math index.
  Zm2[1].setZero();  // i= 0 in 1-based math index.

  // Initializations.
  A = M.A();  // Final result.

  // Index i spans the block rows of M.
  for (int i = 0; i < num_blocks; ++i) {
    // TODO(amcastro-tri): Consider placing these in a 2k x k matrix for better
    // memory access. That way Ai and Ki get resused for two multiplicaitons
    // once loaded in memory.

    // Bring Ai into memory and reuse twice below.
    const BlockType& Ai = M.A()[i];
    const BlockType& Bi = M.B()[i];
    const BlockType& Ci = M.C()[i];
    const BlockType& Yim2 = Ym2[i];
    const BlockType& Zim2 = Zm2[i];
    BlockType& Ki = K[i];
    BlockType& Gi = G[i];
    Ki.noalias() = Bi - Ai * Yim2;
    Gi.noalias() = Ci - Ai * Zim2;  // = Gi'

    // Bring Ki into memory and reuse twice below.
    const BlockType& Yim1 = Ym2[i + 1];
    const BlockType& Zim1 = Zm2[i + 1];
    BlockType& Yi = Ym2[i + 2];
    Gi.noalias() -= Ki * Yim1;
    // N.B. The last block is always zero given that the last two blocks of Z
    // are also always zero, see note below for Z.
    const BlockType& Di = M.D()[i];
    Yi.noalias() = Di - Ki * Zim1;  // = Hi

    // Factorize Gi and reuse it.
    Eigen::LDLT<BlockType>& Ginv_i = Ginv[i];
    Ginv_i.compute(Gi);
    if (Ginv_i.info() != Eigen::Success) {
      status_ = PentaDiagonalFactorizationStatus::kFailure;
      return;
    }

    // TODO(amcastro-tri): can I call .noalias() here?
    Yi = Ginv_i.solve(Yi);
    // N.B. The last two blocks of Z are always zero since the last blocks of E
    // are always zero by convention.
    const BlockType& Ei = M.E()[i];
    BlockType& Zi = Zm2[i + 2];
    Zi = Ginv_i.solve(Ei);
  }

  status_ = PentaDiagonalFactorizationStatus::kSuccess;
}

template <int kBlockSize>
void PentaDiagonalFactorization<kBlockSize>::SolveInPlace(
    EigenPtr<Eigen::VectorXd> b) {
  DRAKE_DEMAND(b->size() == size());

  const int block_size = factorization_.block_size;
  const int num_blocks = factorization_.num_blocks;

  // Copy b into r s.t. the first two blocks in r are zero.
  Eigen::VectorXd& rm2 = r_;  // rm2[i] stores r_{i-2}.
  rm2.head(2 * block_size).setZero();
  rm2.segment(2 * block_size, b->size()) = *b;

  // Second pass to the Gaussian ellimination (first pass happened in
  // Factorize()).
  for (int i = 0; i < num_blocks; ++i) {
    auto rim2 = rm2.segment<kBlockSize>(i * block_size, block_size);
    auto rim1 = rm2.segment<kBlockSize>((i + 1) * block_size, block_size);
    auto ri = rm2.segment<kBlockSize>((i + 2) * block_size, block_size);

    const BlockType& Ai = factorization_.A[i];
    const BlockType& Ki = factorization_.K[i];
    const Eigen::LDLT<BlockType>& Ginv = factorization_.Ginv[i];

    ri -= Ai * rim2;
    ri -= Ki * rim1;
    ri = Ginv.solve(ri);  // heap allocation?
  }

  // Backward substitution.
  *b = rm2.segment(2 * block_size, b->size());
  int i = num_blocks - 1;  // x[n] = r[n].
  const auto xn = b->segment<kBlockSize>(i * block_size, block_size);

  i = num_blocks - 2;
  const BlockType& Ynm1 = factorization_.Ym2[i + 2];
  b->segment<kBlockSize>(i * block_size, block_size) -= Ynm1 * xn;

  for (i = num_blocks - 3; i >= 0; --i) {
    // N.B. Ym2[0] = Ym2[1] = Ym2[num_blocks + 1] = 0.
    const BlockType& Yi = factorization_.Ym2[i + 2];
    const BlockType& Zi = factorization_.Zm2[i + 2];
    auto xi = b->segment<kBlockSize>(i * block_size, block_size);
    const auto xip1 = b->segment<kBlockSize>((i + 1) * block_size, block_size);
    const auto xip2 = b->segment<kBlockSize>((i + 2) * block_size, block_size);
    xi -= Yi * xip1;
    xi -= Zi * xip2;
  }
}

}  // namespace internal
}  // namespace trajopt
}  // namespace multibody
}  // namespace drake
