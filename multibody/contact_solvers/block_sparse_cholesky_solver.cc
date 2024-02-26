#include "drake/multibody/contact_solvers/block_sparse_cholesky_solver.h"

#include <algorithm>
#include <utility>
#include <vector>

#include "drake/multibody/contact_solvers/minimum_degree_ordering.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename BlockType>
BlockSparseCholeskySolver<BlockType>::~BlockSparseCholeskySolver() = default;

template <typename BlockType>
void BlockSparseCholeskySolver<BlockType>::SetMatrix(const SymmetricMatrix& A) {
  const BlockSparsityPattern& A_block_pattern = A.sparsity_pattern();
  /* Compute the elimination ordering using Minimum Degree algorithm. */
  const std::vector<int> elimination_ordering =
      ComputeMinimumDegreeOrdering(A_block_pattern);
  BlockSparsityPattern L_block_pattern =
      SymbolicFactor(A, elimination_ordering);
  SetMatrixImpl(A, elimination_ordering, std::move(L_block_pattern));
}

template <typename BlockType>
void BlockSparseCholeskySolver<BlockType>::UpdateMatrix(
    const SymmetricMatrix& A) {
  PermuteAndCopyToL(A);
  solver_mode_ = SolverMode::kAnalyzed;
}

template <typename BlockType>
bool BlockSparseCholeskySolver<BlockType>::Factor() {
  DRAKE_THROW_UNLESS(solver_mode_ == SolverMode::kAnalyzed);
  const bool success = CalcPartialFactorization(0, L_->block_cols());
  solver_mode_ = success ? SolverMode::kFactored : SolverMode::kEmpty;
  return success;
}

template <typename BlockType>
std::optional<MatrixX<double>>
BlockSparseCholeskySolver<BlockType>::FactorAndCalcSchurComplement(
    const SymmetricMatrix& A,
    const std::unordered_set<int>& eliminated_blocks) {
  const int num_eliminated_blocks = ssize(eliminated_blocks);
  const int num_total_blocks = A.block_cols();
  const int num_remaining_blocks = num_total_blocks - num_eliminated_blocks;
  DRAKE_DEMAND(num_remaining_blocks >= 0);
  if (num_remaining_blocks == 0) {
    SetMatrix(A);
    const bool success = Factor();
    return success ? std::optional<MatrixX<double>>(MatrixX<double>::Zero(0, 0))
                   : std::nullopt;
  }

  /* The Schur complement of the `eliminated_blocks` appear on the bottom
   right corner of the matrix under Cholesky factorization when the
   `eliminated_blocks` have been factorized first with a right-looking
   Cholesky. So we need an elimination ordering that ensures that the blocks
   associated with `eliminated_blocks` are eliminated first. There are
   many ordering that satisfy this requirement, and we look for one that reduces
   fill-in. */
  const std::vector<int> elimination_ordering =
      ComputeMinimumDegreeOrdering(A.sparsity_pattern(), eliminated_blocks);
  SetMatrixImpl(A, elimination_ordering,
                SymbolicFactor(A, elimination_ordering));

  /* Reset solver mode and exit if factorization of the eliminated blocks fails.
   */
  if (!CalcPartialFactorization(0, num_eliminated_blocks)) {
    solver_mode_ = SolverMode::kEmpty;
    return std::nullopt;
  }

  const MatrixX<double> permuted_S =
      L_->MakeDenseBottomRightCorner(num_remaining_blocks)
          .template selfadjointView<Eigen::Lower>();

  /* Finish the factorization. If the factorization fails, set the solver mode
   and exit. */
  bool success =
      CalcPartialFactorization(num_eliminated_blocks, L_->block_cols());
  solver_mode_ = success ? SolverMode::kFactored : SolverMode::kEmpty;
  if (!success) {
    return std::nullopt;
  }

  /* The Schur complement computed here is permuted according to the elimination
   ordering. We need to invert this permutation to obtain the original Schur
   complement. To do that, we need to find the _scalar_ permutation for the
   remaining blocks, and for that we need the _block_ permutation of the
   remaining blocks as well as the sizes of those blocks. */

  /* The block elimination ordering implies a permutation of the remaining
   blocks. For example, suppose the elimination ordering is
   [0, 5, 2, 4, 6, 3, 1], eliminated blocks are {0, 2, 5}, and the remaining
   blocks are {1, 3, 4, 6} (notice how the eliminated blocks come before
   remaining blocks). Then the induced elimination ordering on the remaining
   blocks is [2, 3, 1, 0]. To see that, note that 1, 3, 4, 6 are the 0th, 1st,
   2nd, 3rd elements in the remaining group; the order that they are eliminated
   is

     4 (2nd element in remaining blocks).
     6 (3rd element in remaining blocks).
     3 (1st element in remaining blocks).
     1 (0th element in remaining blocks).

   We name this elimination ordering on the remaining blocks
   `remaining_ordering`. We also store the mapping from global indices to
   remaining indices in `global_to_remaining` where a value of -1 indicates that
   the element is eliminated.

   In particular, in the above example, `remaining ordering` is [2, 3, 1, 0] and
   `global_to_remaining` is [-1, 0, -1, 1, 2, -1, 3]. Note that
   `global_to_remaining` assigns the set {1, 3, 4, 6} indices in ascending
   order. */
  std::vector<int> global_to_remaining(num_total_blocks, -1);
  const std::vector<int>& block_sizes = A.sparsity_pattern().block_sizes();
  std::vector<int> remaining_block_sizes;
  remaining_block_sizes.reserve(num_remaining_blocks);
  int remaining_index = 0;
  for (int i = 0; i < num_total_blocks; ++i) {
    if (!eliminated_blocks.contains(i)) {
      global_to_remaining[i] = remaining_index++;
      remaining_block_sizes.emplace_back(block_sizes[i]);
    }
  }
  /* The scalar index of the first scalar in each remaining block. */
  std::vector<int> remaining_starting_indices(num_remaining_blocks);
  remaining_starting_indices[0] = 0;
  for (int i = 1; i < num_remaining_blocks; ++i) {
    remaining_starting_indices[i] =
        remaining_starting_indices[i - 1] + remaining_block_sizes[i - 1];
  }
  /* The block elimination ordering within remaining blocks. */
  std::vector<int> remaining_ordering;
  remaining_ordering.reserve(num_remaining_blocks);
  /* We start with permuted_i = ssize(eliminated_blocks) to skip all
   eliminated blocks. */
  for (int permuted_i = ssize(eliminated_blocks);
       permuted_i < ssize(elimination_ordering); ++permuted_i) {
    const int i = elimination_ordering[permuted_i];
    DRAKE_DEMAND(global_to_remaining[i] >= 0);
    remaining_ordering.emplace_back(global_to_remaining[i]);
  }
  const int num_remaining_scalars =
      remaining_starting_indices.back() + remaining_block_sizes.back();
  /* Build the scalar elimination ordering (for scalars in the remaining blocks)
   from the block elimination ordering. */
  VectorX<int> scalar_remaining_ordering(num_remaining_scalars);
  int new_scalar_index = 0;
  for (int i = 0; i < ssize(remaining_ordering); ++i) {
    const int block = remaining_ordering[i];
    const int start = remaining_starting_indices[block];
    const int size = remaining_block_sizes[block];
    for (int s = start; s < start + size; ++s) {
      scalar_remaining_ordering(new_scalar_index++) = s;
    }
  }
  /* The elimination ordering is the inverse of the permutation. */
  const auto& inverse_scalar_permutation = scalar_remaining_ordering;
  Eigen::PermutationMatrix<Eigen::Dynamic> P_inv(inverse_scalar_permutation);
  const auto P = P_inv.transpose();
  return P_inv * permuted_S * P;
}

template <typename BlockType>
VectorX<double> BlockSparseCholeskySolver<BlockType>::Solve(
    const Eigen::Ref<const VectorX<double>>& b) const {
  VectorX<double> x(b);
  SolveInPlace(&x);
  return x;
}

template <typename BlockType>
void BlockSparseCholeskySolver<BlockType>::SolveInPlace(
    VectorX<double>* b) const {
  DRAKE_THROW_UNLESS(solver_mode() == SolverMode::kFactored);
  DRAKE_THROW_UNLESS(b != nullptr);
  DRAKE_THROW_UNLESS(b->size() == L_->cols());
  VectorX<double> permuted_b(*b);
  scalar_permutation_.Apply(*b, &permuted_b);

  const BlockSparsityPattern& block_sparsity_pattern = L_->sparsity_pattern();
  const std::vector<int>& block_sizes = block_sparsity_pattern.block_sizes();
  const std::vector<int>& starting_cols = L_->starting_cols();

  /* Solve Lz = b in place. */
  for (int j = 0; j < L_->block_cols(); ++j) {
    const int block_size = block_sizes[j];
    const int offset = starting_cols[j];
    /* Solve for the j-th block entry. */
    const VectorX<double> bj =
        L_diag_[j].matrixL().solve(permuted_b.segment(offset, block_size));
    permuted_b.segment(offset, block_size) = bj;
    /* Eliminate for the j-th block entry from the system. */
    const auto& blocks_in_col_j = L_->block_row_indices(j);
    for (int flat = 1; flat < ssize(blocks_in_col_j); ++flat) {
      const int i = blocks_in_col_j[flat];
      permuted_b.segment(starting_cols[i], block_sizes[i]).noalias() -=
          L_->block_flat(flat, j) * bj;
    }
  }

  VectorX<double>& permuted_z = permuted_b;
  /* Solve Lᵀx = z in place. */
  for (int j = L_->block_cols() - 1; j >= 0; --j) {
    /* Eliminate all solved variables. */
    const auto& blocks_in_col_j = L_->block_row_indices(j);
    for (int flat = 1; flat < ssize(blocks_in_col_j); ++flat) {
      const int i = blocks_in_col_j[flat];
      permuted_z.segment(starting_cols[j], block_sizes[j]).noalias() -=
          L_->block_flat(flat, j).transpose() *
          permuted_z.segment(starting_cols[i], block_sizes[i]);
    }
    /* Solve for the j-th block entry. */
    const VectorX<double> zj = L_diag_[j].matrixU().solve(
        permuted_z.segment(starting_cols[j], block_sizes[j]));
    permuted_z.segment(starting_cols[j], block_sizes[j]) = zj;
  }
  scalar_permutation_.ApplyInverse(permuted_z, b);
}

template <typename BlockType>
typename BlockSparseCholeskySolver<BlockType>::LowerTriangularMatrix
BlockSparseCholeskySolver<BlockType>::L() const {
  DRAKE_THROW_UNLESS(solver_mode() == SolverMode::kFactored);
  return *L_;
}

template <typename BlockType>
Eigen::PermutationMatrix<Eigen::Dynamic>
BlockSparseCholeskySolver<BlockType>::CalcPermutationMatrix() const {
  DRAKE_THROW_UNLESS(solver_mode() != SolverMode::kEmpty);
  const std::vector<int>& p = scalar_permutation_.permutation();
  return Eigen::PermutationMatrix<Eigen::Dynamic>(
      Eigen::Map<const VectorX<int>>(p.data(), p.size()));
}

template <typename BlockType>
void BlockSparseCholeskySolver<BlockType>::SetMatrixImpl(
    const SymmetricMatrix& A, const std::vector<int>& elimination_ordering,
    BlockSparsityPattern&& L_pattern) {
  /* First documented responsibility: set `block_permutation_`. */
  /* Construct the inverse of the elimination ordering, which permutes the
   original indices to new indices. */
  std::vector<int> permutation(elimination_ordering.size());
  for (int i = 0; i < ssize(permutation); ++i) {
    permutation[elimination_ordering[i]] = i;
  }
  block_permutation_ = PartialPermutation(std::move(permutation));
  /* Second documented responsibility: set `scalar_permutation_`. */
  SetScalarPermutation(A, elimination_ordering);
  /* Third documented responsibility: allocate for `L_` and `L_diag_`. */
  L_ = std::make_unique<LowerTriangularMatrix>(std::move(L_pattern));
  L_diag_.resize(A.block_cols());
  /* Fourth documented responsibility: UpdateMatrix. */
  UpdateMatrix(A);
}

template <typename BlockType>
void BlockSparseCholeskySolver<BlockType>::SetScalarPermutation(
    const SymmetricMatrix& A, const std::vector<int>& elimination_ordering) {
  /* It's easier to build the scalar elimination ordering first from block
   elimination ordering and then convert it to the scalar permutation (the
   inverse of the scalar elimination ordering) that induces the permutation P
   such that L⋅Lᵀ = P⋅A⋅Pᵀ.
   More specificially, Pᵢⱼ = 1 for j = scalar_elimination_ordering[i] (or
   equivalently i = scalar_permutation_[j]) and Pᵢⱼ = 0 otherwise. See
   CalcPermutationMatrix(). */
  std::vector<int> scalar_elimination_ordering(A.cols());
  {
    const BlockSparsityPattern& A_block_pattern = A.sparsity_pattern();
    const std::vector<int>& A_block_sizes = A_block_pattern.block_sizes();
    const std::vector<int>& starting_indices = A.starting_cols();
    int i_permuted = 0;
    for (int block_permuted = 0; block_permuted < ssize(elimination_ordering);
         ++block_permuted) {
      const int block = elimination_ordering[block_permuted];
      const int start = starting_indices[block];
      const int size = A_block_sizes[block];
      for (int i = start; i < start + size; ++i) {
        scalar_elimination_ordering[i_permuted++] = i;
      }
    }
  }
  /* Invert the elimination ordering to get the permutation. */
  std::vector<int> scalar_permutation(scalar_elimination_ordering.size());
  for (int i_permuted = 0; i_permuted < ssize(scalar_permutation);
       ++i_permuted) {
    scalar_permutation[scalar_elimination_ordering[i_permuted]] = i_permuted;
  }
  scalar_permutation_ = PartialPermutation(std::move(scalar_permutation));
}

template <typename BlockType>
BlockSparsityPattern BlockSparseCholeskySolver<BlockType>::SymbolicFactor(
    const SymmetricMatrix& A, const std::vector<int>& elimination_ordering) {
  /* 1. Compute the block permutation as well as the scalar permutation. */
  const int n = elimination_ordering.size();
  /* Construct the inverse of the elimination ordering, which permutes the
   original indices to new indices. */
  std::vector<int> permutation(n);
  for (int i = 0; i < n; ++i) {
    permutation[elimination_ordering[i]] = i;
  }
  const PartialPermutation block_permutation(std::move(permutation));

  /* Find the sparsity pattern of the permuted A (under the permutation induced
   by the elimination ordering). */
  const BlockSparsityPattern& A_block_pattern = A.sparsity_pattern();
  const std::vector<int>& A_block_sizes = A_block_pattern.block_sizes();
  const std::vector<std::vector<int>>& sparsity_pattern =
      A_block_pattern.neighbors();
  std::vector<std::vector<int>> permuted_sparsity_pattern(
      sparsity_pattern.size());
  for (int i = 0; i < ssize(sparsity_pattern); ++i) {
    const int pi = block_permutation.permuted_index(i);
    for (int j : sparsity_pattern[i]) {
      const int pj = block_permutation.permuted_index(j);
      permuted_sparsity_pattern[std::min(pi, pj)].emplace_back(
          std::max(pi, pj));
    }
  }
  std::vector<int> permuted_block_sizes(A.block_cols());
  block_permutation.Apply(A_block_sizes, &permuted_block_sizes);

  /* Compute the sparsity pattern of L given the sparsity pattern of A in the
   new ordering. */
  return contact_solvers::internal::SymbolicCholeskyFactor(
      BlockSparsityPattern(permuted_block_sizes, permuted_sparsity_pattern));
}

template <typename BlockType>
bool BlockSparseCholeskySolver<BlockType>::CalcPartialFactorization(
    int starting_col_block, int ending_col_block) {
  DRAKE_THROW_UNLESS(solver_mode() == SolverMode::kAnalyzed);
  DRAKE_DEMAND(starting_col_block >= 0 &&
               starting_col_block <= L_->block_cols());
  DRAKE_DEMAND(ending_col_block >= 0 && ending_col_block <= L_->block_cols());
  for (int j = starting_col_block; j < ending_col_block; ++j) {
    /* Update diagonal. */
    const BlockType& Ajj = L_->diagonal_block(j);
    L_diag_[j].compute(Ajj);
    if (L_diag_[j].info() != Eigen::Success) {
      return false;
    }
    L_->SetBlockFlat(0, j, L_diag_[j].matrixL());
    /* Update L₂₁ column.
     | a₁₁  *  | = | λ₁₁  0 | * | λ₁₁ᵀ L₂₁ᵀ |
     | a₂₁ a₂₂ |   | L₂₁ L₂₂|   |  0   L₂₂ᵀ |
     So we have
      L₂₁λ₁₁ᵀ = a₂₁, and thus
      λ₁₁L₂₁ᵀ = a₂₁ᵀ */
    const std::vector<int>& row_blocks = L_->block_row_indices(j);
    const auto Ljj = L_diag_[j].matrixL();
    /* We start from flat = 1 here to skip the j,j diagonal entry. */
    for (int flat = 1; flat < ssize(row_blocks); ++flat) {
      const BlockType& Aij = L_->block_flat(flat, j);
      BlockType Lij = Ljj.solve(Aij.transpose()).transpose();
      L_->SetBlockFlat(flat, j, std::move(Lij));
    }
    /* Update L₂₂ according to L₂₂ = a₂₂ - L₂₁⋅L₂₁ᵀ. */
    RightLookingSymmetricRank1Update(j);
  }
  return true;
}

template <typename BlockType>
void BlockSparseCholeskySolver<BlockType>::RightLookingSymmetricRank1Update(
    int j) {
  const std::vector<int>& blocks_in_col_j = L_->block_row_indices(j);
  const int n = blocks_in_col_j.size();
  /* We start from k = 1 here to skip the j,j diagonal entry. */
  for (int k = 1; k < n; ++k) {
    const int col = blocks_in_col_j[k];
    const BlockType& B = L_->block_flat(k, j);
    for (int l = k; l < n; ++l) {
      const int row = blocks_in_col_j[l];
      const BlockType& A = L_->block_flat(l, j);
      L_->AddToBlock(row, col, -A * B.transpose());
    }
  }
}

template <typename BlockType>
void BlockSparseCholeskySolver<BlockType>::PermuteAndCopyToL(
    const SymmetricMatrix& A) {
  const int n = A.block_cols();
  DRAKE_DEMAND(n == block_permutation_.domain_size());
  DRAKE_DEMAND(n == block_permutation_.permuted_domain_size());
  L_->SetZero();
  for (int j = 0; j < n; ++j) {
    const std::vector<int>& row_indices = A.block_row_indices(j);
    for (int i : row_indices) {
      const BlockType& block = A.block(i, j);
      const int pi = block_permutation_.permuted_index(i);
      const int pj = block_permutation_.permuted_index(j);
      if (pi >= pj) {
        L_->SetBlock(pi, pj, block);
      } else {
        L_->SetBlock(pj, pi, block.transpose());
      }
    }
  }
}

template class BlockSparseCholeskySolver<MatrixX<double>>;
template class BlockSparseCholeskySolver<Matrix3<double>>;

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
