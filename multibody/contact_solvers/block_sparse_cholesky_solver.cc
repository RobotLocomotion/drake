#include "drake/multibody/contact_solvers/block_sparse_cholesky_solver.h"

#include <algorithm>
#include <numeric>
#include <utility>
#include <vector>

#include "drake/multibody/contact_solvers/minimum_degree_ordering.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

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
  matrix_set_ = true;
  is_factored_ = false;
}

template <typename BlockType>
bool BlockSparseCholeskySolver<BlockType>::Factor() {
  DRAKE_THROW_UNLESS(matrix_set_);
  matrix_set_ = false;
  for (int j = 0; j < L_->block_cols(); ++j) {
    /* Update diagonal. */
    const BlockType& Ajj = L_->diagonal_block(j);
    L_diag_[j].compute(Ajj);
    if (L_diag_[j].info() != Eigen::Success) {
      is_factored_ = false;
      return false;
    }
    /* Technically, we should have
      L_->SetBlockFlat(0, j, L_diag_[j].matrixL());
    for a complete L_ matrix, but we omit it here since we don't ever touch the
    diagonal block of L_. Instead, we directly invoke L_diag_. */

    /* Update column.
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
    RightLookingSymmetricRank1Update(j);
  }
  is_factored_ = true;
  return true;
}

template <typename BlockType>
VectorX<double> BlockSparseCholeskySolver<BlockType>::Solve(
    const Eigen::Ref<const VectorX<double>>& y) const {
  VectorX<double> x(y);
  SolveInPlace(&x);
  return x;
}

template <typename BlockType>
void BlockSparseCholeskySolver<BlockType>::SolveInPlace(
    VectorX<double>* y) const {
  DRAKE_THROW_UNLESS(is_factored_);
  DRAKE_DEMAND(y != nullptr);
  DRAKE_DEMAND(y->size() == L_->cols());
  VectorX<double> permuted_y(*y);
  scalar_permutation_.Apply(*y, &permuted_y);

  const BlockSparsityPattern& block_sparsity_pattern = L_->sparsity_pattern();
  const std::vector<int>& block_sizes = block_sparsity_pattern.block_sizes();
  const std::vector<int>& starting_cols = L_->starting_cols();

  /* Solve Lz = y in place. */
  for (int j = 0; j < L_->block_cols(); ++j) {
    const int block_size = block_sizes[j];
    const int offset = starting_cols[j];
    /* Solve for the j-th block entry. */
    const VectorX<double> yj =
        L_diag_[j].matrixL().solve(permuted_y.segment(offset, block_size));
    permuted_y.segment(offset, block_size) = yj;
    /* Eliminate for the j-th block entry from the system. */
    const auto& blocks_in_col_j = L_->block_row_indices(j);
    for (int flat = 1; flat < ssize(blocks_in_col_j); ++flat) {
      const int i = blocks_in_col_j[flat];
      permuted_y.segment(starting_cols[i], block_sizes[i]).noalias() -=
          L_->block_flat(flat, j) * yj;
    }
  }

  VectorX<double>& permuted_z = permuted_y;
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
  scalar_permutation_.ApplyInverse(permuted_z, y);
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
   inverse of the scalar elimination ordering). The scalar permutation maps the
   right hand side of A * x = b to the right handside of
   L * L^T * x_permuted = b_permuted. */
  std::vector<int> scalar_elimination_ordering(A.cols());
  const BlockSparsityPattern& A_block_pattern = A.sparsity_pattern();
  const std::vector<int>& A_block_sizes = A_block_pattern.block_sizes();
  const std::vector<int>& starting_indices = A.starting_cols();
  int scalar_order = 0;
  for (int i = 0; i < ssize(elimination_ordering); ++i) {
    const int block = elimination_ordering[i];
    const int start = starting_indices[block];
    const int size = A_block_sizes[block];
    for (int s = start; s < start + size; ++s) {
      scalar_elimination_ordering[scalar_order++] = s;
    }
  }
  /* Invert the elimination ordering to get the permutation. */
  std::vector<int> scalar_permutation(scalar_elimination_ordering.size());
  for (int i = 0; i < ssize(scalar_permutation); ++i) {
    scalar_permutation[scalar_elimination_ordering[i]] = i;
  }
  scalar_permutation_ = PartialPermutation(std::move(scalar_permutation));
}

template <typename BlockType>
BlockSparsityPattern BlockSparseCholeskySolver<BlockType>::SymbolicFactor(
    const SymmetricMatrix& A, const std::vector<int>& elimination_ordering) {
  /* 1. Compute the block permutation as well as the scalar permutation. */
  const int N = elimination_ordering.size();
  /* Construct the inverse of the elimination ordering, which permutes the
   original indices to new indices. */
  std::vector<int> permutation(N);
  for (int i = 0; i < N; ++i) {
    permutation[elimination_ordering[i]] = i;
  }
  PartialPermutation block_permutation(std::move(permutation));

  /* Find the sparsity pattern of the permuted A (under the permutation induced
   by the elimination ordering). */
  const BlockSparsityPattern& A_block_pattern = A.sparsity_pattern();
  const std::vector<int>& A_block_sizes = A_block_pattern.block_sizes();
  const std::vector<std::vector<int>>& sparsity_pattern =
      A_block_pattern.neighbors();
  std::vector<std::vector<int>> permuted_sparsity_pattern(
      sparsity_pattern.size());
  for (int i = 0; i < ssize(sparsity_pattern); ++i) {
    const int pi = block_permutation.push(i);
    for (int j : sparsity_pattern[i]) {
      const int pj = block_permutation.push(j);
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
void BlockSparseCholeskySolver<BlockType>::RightLookingSymmetricRank1Update(
    int j) {
  const std::vector<int>& blocks_in_col_j = L_->block_row_indices(j);
  const int N = blocks_in_col_j.size();
  /* We start from k = 1 here to skip the j,j diagonal entry. */
  for (int k = 1; k < N; ++k) {
    const int col = blocks_in_col_j[k];
    const BlockType& B = L_->block_flat(k, j);
    for (int l = k; l < N; ++l) {
      const int row = blocks_in_col_j[l];
      const BlockType& A = L_->block_flat(l, j);
      L_->AddToBlock(row, col, -A * B.transpose());
    }
  }
}

template <typename BlockType>
void BlockSparseCholeskySolver<BlockType>::PermuteAndCopyToL(
    const SymmetricMatrix& A) {
  const int N = A.block_cols();
  DRAKE_DEMAND(N == block_permutation_.domain_size());
  DRAKE_DEMAND(N == block_permutation_.permuted_domain_size());
  L_->SetZero();
  for (int j = 0; j < N; ++j) {
    const std::vector<int>& row_indices = A.block_row_indices(j);
    for (int i : row_indices) {
      const BlockType& block = A.block(i, j);
      const int pi = block_permutation_.push(i);
      const int pj = block_permutation_.push(j);
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
