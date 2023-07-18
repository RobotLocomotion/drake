#pragma once

#include <memory>
#include <optional>
#include <unordered_set>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/reset_after_move.h"
#include "drake/multibody/contact_solvers/block_sparse_lower_triangular_or_symmetric_matrix.h"
#include "drake/multibody/contact_solvers/sap/partial_permutation.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

/* A Cholesky solver for solving the symmetric positive definite
 system
   A⋅x = b
 where A is block sparse.
 Example use case:
 ```
  BlockSparseCholeskySolver<MatrixX<double>> solver;
  // Sets the matrix A.
  solver.SetMatrix(A);
  // Factorize the matrix.
  solver.Factor();
  // Solve A⋅x1 = b1.
  x1 = solver.Solve(b1);
  // Solve A⋅x2 = b2. This reuses the factorization (important for speed!).
  x2 = solver.Solve(b2);
  // Update the numerical values in the matrix but the sparsity pattern doesn't
  // change.
  solver.UpdateMatrix(A2);
  // Need to refactor after the matrix has changed.
  solver.Factor();
  // Solve A2⋅x = b using updated factorization.
  x = solver.Solve(b);
 ```

 @tparam BlockType The matrix type for individual block matrices;
 MatrixX<double> or Matrix3<double>. The fixed-size matrix version is preferred
 if you know the sizes of blocks are uniform and fixed. */
template <typename BlockType>
class BlockSparseCholeskySolver {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BlockSparseCholeskySolver);

  /* The state of the solver. */
  enum class SolverMode {
    kEmpty,     // Matrix A is yet to be set.
    kAnalyzed,  // Matrix A is symbolically analyzed and ready to be factored,
                // but not yet numerically factored.
    kFactored,  // Matrix A is factored in place into L, its lower triangular
                // Cholesky factorization.
  };

  using SymmetricMatrix =
      BlockSparseLowerTriangularOrSymmetricMatrix<BlockType, true>;
  using LowerTriangularMatrix =
      BlockSparseLowerTriangularOrSymmetricMatrix<BlockType, false>;

  /* Constructs a BlockSparseCholeskySolver. */
  BlockSparseCholeskySolver() = default;

  ~BlockSparseCholeskySolver();

  /* Sets the matrix to be factored and analyzes its sparsity pattern to find an
   efficient elimination ordering. Factor() may be called after call to
   SetMatrix(). SetMatrix() can be called repeatedly in a row, but prefer the
   following pattern when the sparsity patterns don't change.

     solver.SetMatrix(A0);
     ...
     solver.UpdateMatrix(A1);
     ...
     solver.UpdateMatrix(A2);
     ...
     solver.UpdateMatrix(A3);
     ...
   See UpdateMatrix().

   @pre A is positive definite.
   @post solver_mode() == SolverMode::kAnalyzed. */
  void SetMatrix(const SymmetricMatrix& A);

  /* Updates the matrix to be factored. This is useful for solving a series of
   matrices with the same sparsity pattern using the same elimination ordering.
   For example, with matrices A and B with the same sparisty pattern. It's more
   efficient to call
     solver.SetMatrix(A);
     solver.Factor();
     solver.Solve(...)
     solver.UpdateMatrix(B);
     solver.Factor();
     solver.Solve(...)
   than to call
     solver.SetMatrix(A);
     solver.Factor();
     solver.Solve(...)
     solver.SetMatrix(B);
     solver.Factor();
     solver.Solve(...)
   Factor() may be called after call to UpdateMatrix().
   @pre SetMatrix() has been invoked and the argument to the last call of
   SetMatrix() has the same sparsity pattern of A.
   @pre A is positive definite.
   @post solver_mode() == SolverMode::kAnalyzed. */
  void UpdateMatrix(const SymmetricMatrix& A);

  /* Computes the block sparse Cholesty factorization. Returns true if
   factorization succeeds, otherwise returns false. Failure is triggered by an
   internal failure of Eigen::LLT.  This can fail if, for instance, the input
   matrix set in SetMatrix() or UpdateMatrix() is not positive definite. If
   failure is encountered, the user should verify that the specified matrix is
   positive definite and not poorly conditioned.
   @throws std::exception if solver_mode() is not SolverMode::kAnalyzed.
   @post solver_mode() is SolverMode::kFactored if factorization is successful
   and is SolverMode::kEmpty otherwise. */
  [[nodiscard]] bool Factor();

  /* Computes the block sparse Cholesky factorization of the given matrix `A`.
   Returns std::nullopt if the factorization fails. Failure is triggered by an
   internal failure of Eigen::LLT. This can fail if, for instance, the input
   matrix `A` is not positive definite. If failure is encountered, the user
   should verify that the specified matrix is positive definite and not poorly
   conditioned.

   In addition, this function computes the Schur complement matrix of the input
   matrix A in the following sense:

   Let E be the set of `eliminated_blocks`. We define permutation p on the block
   indices of `A` such that p(i) < p(j) iff

    (1) i ∈ E and j ∉ E or
    (2) i ∈ E and j ∈ E and i < j or
    (3) i ∉ E and j ∉ E and i < j.

   For example, if A has 7 block rows/columns and E = {2, 5, 6}, then

    [p(0), p(1), p(2), p(3), p(4), p(5), p(6)] =
    [3,    4,    0,    5,    6,    1,    2]

   We then define Â = P⋅A⋅Pᵀ, where P is the permutation matrix representation
   of p (i.e. Pᵢⱼ = 1 if i = p(j) and Pᵢⱼ = 0 otherwise). In other words, Â is
   permuted from A so that blocks with indices in `eliminated_blocks` appear on
   the top left corner of the matrix Â and all other blocks appear on the bottom
   right corner of the matrix. The matrix Â can be written in block form as Â =
   [D, B; Bᵀ, C] where D corresponds to the blocks with indices in E, C
   corresponds to the blocks with indices outside of E, and B is the resulting
   off-diagonal block from the permutation.

   If the fatorization of A is successful, returns the Schur complement
   S = C - BᵀD⁻¹B.

   @pre `eliminated_blocks` has all its entries in [0, A.block_cols()).
   @post solver_mode() is SolverMode::kFactored if factorization is successful
   and is SolverMode::kEmpty otherwise. */
  [[nodiscard]] std::optional<MatrixX<double>> FactorAndCalcSchurComplement(
      const SymmetricMatrix& A,
      const std::unordered_set<int>& eliminated_blocks);

  /* Solves the system A⋅x = b and returns x.
   @throws std::exception if b.size() is incompatible with the size of the
   matrix set by SetMatrix().
   @throws std::exception unless solver_mode() == SolverMode::kFactored. */
  VectorX<double> Solve(const Eigen::Ref<const VectorX<double>>& b) const;

  /* Solves the system A⋅x = b and writes the result in b.
   @throws std::exception if b == nullptr or b->size() is incompatible with the
   size of the matrix set by SetMatrix().
   @throws std::exception unless solver_mode() == SolverMode::kFactored. */
  void SolveInPlace(VectorX<double>* b) const;

  /* Returns the current mode of the solver. See SolverMode. */
  SolverMode solver_mode() const { return solver_mode_; }

  /* Returns (the lower triangular) Cholesky factorization matrix L as a
   dense matrix. L is defined by L⋅Lᵀ = P⋅A⋅Pᵀ, where A is the matrix set via
   SetMatrix() or UpdateMatrix() and P is the permutation matrix induced by the
   elimination ordering (see CalcPermutationMatrix()).
   @throws std::exception unless solver_mode() == SolverMode::kFactored. */
  LowerTriangularMatrix L() const;

  /* Returns the permutation matrix induced by the elimination ordering.
   The permutation matrix is defined through L⋅Lᵀ = P⋅A⋅Pᵀ, where A is the
   matrix set via SetMatrix() or UpdateMatrix() and L is the lower triangular
   Cholesky factorization of the permuted A.
   @throws std::exception if solver_mode() == SolverMode::kEmpty. */
  Eigen::PermutationMatrix<Eigen::Dynamic> CalcPermutationMatrix() const;

 private:
  /* Helper for SetMatrix() to set the matrix given its elimination ordering and
   the sparsity pattern of its Cholesky factorization. It performs the
   following:
    1. sets `block_permutation_`;
    2. sets `scalar_permutation_`;
    3. allocates for `L_` and `L_diag_`;
    4. calls UpdateMatrix(A) to copy the numeric values of A to L_.
   @param[in] A                     The matrix to be factored.
   @param[in] elimination_ordering  Elimination ordering of the blocks of A.
                                    Must be a permutation of {0, 1, ...,
                                    A.block_cols() - 1}.
   @param[in] L_pattern             The block sparsity pattern of the lower
                                    triangular matrix L if A is to be factored
                                    with the given elimination ordering.  */
  void SetMatrixImpl(const SymmetricMatrix& A,
                     const std::vector<int>& elimination_ordering,
                     BlockSparsityPattern&& L_pattern);

  /* Sets `scalar_permutation_` given the matrix A and the prescribed
   elimination ordering.
   @param[in] A                     The matrix to be factored.
   @param[in] elimination_ordering  Elimination ordering of the blocks of A.
                                    Must be a permutation of {0, 1, ...,
                                    A.block_cols() - 1}. */
  void SetScalarPermutation(const SymmetricMatrix& A,
                            const std::vector<int>& elimination_ordering);

  /* Returns the block sparsity pattern of the L matrix from a block sparse
   Cholesky factorization following the prescribed elimination ordering.
   @param[in] A                     The matrix to be factored.
   @param[in] elimination_ordering  Elimination ordering of the blocks of A.
                                    Must be a permutation of {0, 1, ...,
                                    A.block_cols() - 1}. */
  BlockSparsityPattern SymbolicFactor(
      const SymmetricMatrix& A, const std::vector<int>& elimination_ordering);

  /* Factorizes matrix A but in particular only processes a range of columns.
   If the range of columns is [0, L_.block_cols()), then this method performs a
   full factorization of A. Otherwise, this leaves the underlying factorization
   in an intermediate state and therefore successive calls to this method
   must be performed with care.
   @note this function does not modify solver mode.
   @pre solver_mode() == kAnalyzed.
   @pre 0 <= starting_col_block <= ending_col_block <= L.block_cols(). */
  bool CalcPartialFactorization(int starting_col_block, int ending_col_block);

  /* Performs L(j+1:, j+1:) -= L(j+1:, j) * L(j+1:, j).transpose().
   @pre 0 <= j < L.block_cols(). */
  void RightLookingSymmetricRank1Update(int j);

  /* Permutes the given matrix A with `block_permutation_` p and set L such that
   the lower triangular part of L satisfies L(p(i), p(j)) = A(i, j).
   @pre SetMarix() has been called. */
  void PermuteAndCopyToL(const SymmetricMatrix& A);

  /* The cholesky factorization of the permuted matrix, i.e. L⋅Lᵀ = P⋅A⋅Pᵀ,
   where P is the permutation matrix induced by the `scalar_permutation_`. */
  copyable_unique_ptr<LowerTriangularMatrix> L_;
  std::vector<Eigen::LLT<BlockType>> L_diag_;

  /* Block and scalar representations of the permutation matrix P (see
   CalcPermutationMatrix()). */
  /* Permutation for block indices, same size as A.block_cols(). For a given
   block index i into A, `block_permutation_[i]` gives the permuted block index
   into L_. */
  PartialPermutation block_permutation_;
  /* Permutation for scarlar indices, same size as A.cols(). For a given
   scalar index i into A, `scalar_permutation_[i]` gives the permuted scalar
   index into L_. */
  PartialPermutation scalar_permutation_;

  reset_after_move<SolverMode> solver_mode_{SolverMode::kEmpty};
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
