#pragma once

#include <memory>
#include <unordered_set>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
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

 @tparam BlockType The matrix type for individual block matrices, usually
                   MatrixX<double>, but fixed size matrices are preferred if you
                   know the sizes of blocks are uniform and fixed. */
template <typename BlockType>
class BlockSparseCholeskySolver {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BlockSparseCholeskySolver);

  using SymmetricMatrix =
      BlockSparseLowerTriangularOrSymmetricMatrix<BlockType, true>;

  /* Constructs a BlockSparseCholeskySovler. */
  BlockSparseCholeskySolver() = default;

  /* Sets the matrix to be factored and analyzes its sparsity pattern to find an
   efficient elimination ordering. Factor() may be called after call to
   SetMatrix().
   @pre A is positive definite.
   @post matrix_set() == true and is_factored() == false. */
  void SetMatrix(const SymmetricMatrix& A);

  /* Updates the matrix to be factored. This is useful for solving a series of
   matrices with the same sparsity pattern using the same elimination ordering.
   For example, with matrices A, B, and C with the same sparisty pattern. It's
   more efficient to call
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
   @pre A has the same elimination ordering as the input of the last invocation
   of SetMatrix(). UpdateMatrix() must not be called before any invocation of
   SetMatrix().
   @pre A is positive definite.
   @post matrix_set() == true and is_factored() == false. */
  void UpdateMatrix(const SymmetricMatrix& A);

  /* Computes the block sparse Cholesty factorization. Returns true if
   factorization succeeds, otherwise returns false. Failure is triggered by an
   internal failure of Eigen::LLT.  This can fail if, for instance, the input
   matrix set in SetMatrix() or UpdateMatrix() is not positive definite. If
   failure is encountered, the user should verify that the specified matrix is
   positive definite and not poorly conditioned.
   @throws std::exception if matrix_set() returns false.
   @post matrix_set() == false and is_factored() == true. */
  bool Factor();

  /* Solves the system A⋅x = b and returns x.
   @pre b.size() is compatible with the size of the matrix set by SetMatrix().
   @throws std::exception if is_factored() returns false. */
  VectorX<double> Solve(const Eigen::Ref<const VectorX<double>>& b) const;

  /* Solves the system A⋅x = b and writes the result in b.
   @pre y != nullptr and y->size() is compatible with the size of the matrix set
   by SetMatrix().
   @throws std::exception if is_factored() returns false. */
  void SolveInPlace(VectorX<double>* y) const;

  /* Returns true iff the matrix is set via SetMatrix() or UpdateMatrix() but
   the matrix hasn't been factorized yet (via Factor() or
   CalcSchurComplementAndFactor()) since the matrix is set. */
  bool matrix_set() const { return matrix_set_; }

  /* Returns true iff the matrix has been factorized (via Factor() or
   CalcSchurComplementAndFactor()) and is ready for back solve. */
  bool is_factored() const { return is_factored_; }

 private:
  using LowerTriangularMatrix =
      BlockSparseLowerTriangularOrSymmetricMatrix<BlockType, false>;

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

  /* Performs L(j+1:, j+1:) -= L(j+1:, j) * L(j+1:, j).transpose().
   @pre 0 <= j < L.block_cols(). */
  void RightLookingSymmetricRank1Update(int j);

  /* Permutes the given matrix A with `block_permutation_` p and set L such that
   the lower triangular part of L satisfies L(p(i), p(j)) = A(i, j).
   @pre SetMarix() has been called. */
  void PermuteAndCopyToL(const SymmetricMatrix& A);

  copyable_unique_ptr<LowerTriangularMatrix> L_;
  std::vector<Eigen::LLT<BlockType>> L_diag_;
  /* The mapping from the internal indices (i.e, the indices for L_) to
   the indices of the original matrix supplied in SetMatrix(). */
  PartialPermutation block_permutation_;  // permutation for block indices, same
                                          // size as A.block_cols().
  PartialPermutation scalar_permutation_;  // permutation for scalar indices,
                                           // same size as A.cols().
  bool is_factored_{false};
  bool matrix_set_{false};
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
