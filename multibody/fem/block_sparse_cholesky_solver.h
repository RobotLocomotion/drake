#pragma once

#include <set>
#include <unordered_set>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/sap/partial_permutation.h"
#include "drake/multibody/fem/symmetric_block_sparse_matrix.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* result[j] is the set of i>=j such that the i,j block of the matrix is
 nonzero.*/
std::vector<std::set<int>> BuildAdjacencyGraph(
    int num_verts, const std::vector<Vector4<int>>& elements);

/* Computes the elimination of the matrix with the given `adjacency_graph` that
 CHOLMOD thinks is the best. For example if the result is [1, 3, 0, 2], it means
 that we should first eliminate vertex 1, then 3, 0, and 2. In other words, this
 is a permutation mapping from new vertex indices to old indices. */
std::vector<int> CalcEliminationOrdering(
    const std::vector<std::set<int>>& adjacency_graph);

/* Computes the elimination ordering for the subgraph of `adjacency_graph`
 involving vertices in `D_indices` only as well as the subgraph that includes
 the complement of `D_indices` only and then returns a concatenation of those
 two orderings. */
std::vector<int> CalcEliminationOrdering(
    const std::vector<std::set<int>>& adjacency_graph,
    const std::vector<int>& D_indices);

/* Computes an elimination ordering consistent with the given `ordering` that
 puts vertices in the set `D` first. More specifically, let P: V->V be the given
 `ordering` and D ⊂ V. This function computes a new elimination ordering Q:V->V
 such that
 1. Q(d) < Q(a) if d ∈ D and a ∉ D,
 2. if d₁, d₂ ∈ D, Q(d₁) < Q(d₂) iff P(d₁) < P(d₂), and
 3. if a₁, a₂ ∉ D, Q(a₁) < Q(a₂) iff P(a₁) < P(a₂). */
std::vector<int> RestrictOrdering(const std::vector<int>& ordering,
                                  const std::vector<int>& D);

/* Returns the column-wise sparsity pattern of L given the adjacency graph of A
 and the elimination ordering.
 Note that the elimination ordering is a mapping from new index to old index. */
std::vector<std::vector<int>> CalcSparsityPattern(
    const std::vector<std::set<int>>& adjacency_graph,
    const std::vector<int>& elimination_ordering);

std::vector<std::vector<int>> GetFillInGraph(
    int num_verts, const std::vector<Vector4<int>>& cliques);

/* Given an input matrix M, and a permutation mapping e, sets the resulting
 matrix M̃ such that M̃(i, j) = M(e(i), e(j)). */
void PermuteSymmetricBlockSparseMatrix(
    const SymmetricBlockSparseMatrix<double>& input,
    const std::vector<int>& permutation,
    SymmetricBlockSparseMatrix<double>* result);

/* Sparse cholesky solver where the blocks are of size 3x3. */
class BlockSparseCholeskySolver {
 public:
  /* Constructs a solver. */
  BlockSparseCholeskySolver() = default;

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BlockSparseCholeskySolver);

  /* Sets the matrix to be factored and uses the best-effort elimination
   ordering that minimizes fill-ins. */
  void SetMatrix(const SymmetricBlockSparseMatrix<double>& A);

  /* Sets the matrix to be factored and uses the given elimination ordering. */
  void SetMatrix(const SymmetricBlockSparseMatrix<double>& A,
                 const std::vector<int>& elimination_ordering);

  /* Updates the matrix to be factored. This is useful for solving a series of
   matrices with the same sparsity pattern using the same elimination ordering.
   For example, with matrices A, B, and C with the same sparisty pattern. It's
   more efficient to call
     solver.SetMatrix(A);
     solver.UpdateMatrix(B);
     solver.UpdateMatrix(C);
   than to call
     solver.SetMatrix(A);
     solver.SetMatrix(B);
     solver.SetMatrix(C); */
  void UpdateMatrix(const SymmetricBlockSparseMatrix<double>& A);

  /* (Advanced) Returns the Schur complement matrix S = A - BᵀD⁻¹B of the matrix
   M. Upon row and column permutation, the matrix takes the form
     M = [D, B; Bᵀ, A]
   The input `eliminated_block_indices` specifies the block row (and column due
   to symmetry) block indices of the 3x3 blocks that belong to D. In other
   words, the i,j-th block of D is the eliminated_block_indices[i],
   eliminated_block_indices[j]-th block of the original matrix. The ordering for
   rows and columns of A is induced from the rows and columns of the original
   matrix.
   Resets the matrix set up in `SetMatrix()` and `UpdateMatrix()`.
   @pre all entries in `eliminated_block_indices` are unique and lie in
   [0, M.cols()/3). */
  MatrixX<double> CalcSchurComplementAndFactor(
      const SymmetricBlockSparseMatrix<double>& M,
      const std::vector<int>& eliminated_block_indices);

  void Factor() {
    DRAKE_DEMAND(!is_factored_);
    FactorImpl(0, block_cols_);
  }

  int size() const { return block_cols_ * 3; }

  int num_block_columns() const { return block_cols_; }

  void SolveInPlace(VectorX<double>* y) const;

  VectorX<double> Solve(const VectorX<double>& y) const;

 private:
  void SetMatrixImpl(const SymmetricBlockSparseMatrix<double>& A,
                     const std::vector<std::set<int>>& adjacency_graph,
                     const std::vector<int>& elimination_ordering);

  void FactorImpl(int starting_col_block, int ending_col_block);

  /* Performs L(j+1:, j+1:) -= L(j+1:,j) * L(j+1:,j).transpose().
   @pre 0 <= j < block_cols_. */
  void RightLookingSymmetricRank1Update(int j);

  int block_cols_{0};
  SymmetricBlockSparseMatrix<double> L_{{}};
  std::vector<Matrix3<double>> L_diag_;
  /* The mapping from the internal block indices (i.e, the indices for L_) to
   the block indices of the matrix supplied in SetMatrix(). */
  contact_solvers::internal::PartialPermutation internal_to_original_;
  contact_solvers::internal::PartialPermutation internal_to_original_dof_;
  bool is_factored_{false};
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
