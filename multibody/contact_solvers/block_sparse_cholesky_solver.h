#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/block_sparse_lower_triangular_or_symmetric_matrix.h"
#include "drake/multibody/contact_solvers/sap/partial_permutation.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

/* A supernodal Cholesky solver for solving the symmetric positive definite
 system
   A⋅x = b
 Example use case:

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

 @tparam BlockType The matrix type for individual block matrices, usually
                   MatrixX<double>, but fixed size matrices are preferred if you
                   know the sizes of blocks are uniform and fixed. */
template <typename BlockType>
class BlockSparseCholeskySolver {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BlockSparseCholeskySolver);

  using SymmetricMatrix =
      BlockSparseLowerTriangularOrSymmetricMatrix<BlockType, true>;

  /* Constructs a solver. */
  BlockSparseCholeskySolver() = default;

  /* Sets the matrix to be factored and computes the elimination ordering using
   minimum degree algorithm to prepare for the factorization. */
  void SetMatrix(const SymmetricMatrix& A);

  /* Updates the matrix to be factored. This is useful for solving a series of
   matrices with the same sparsity pattern using the same elimination ordering.
   For example, with matrices A, B, and C with the same sparisty pattern. It's
   more efficient to call
     solver.SetMatrix(A);
     ...
     solver.UpdateMatrix(B);
     ...
   than to call
     solver.SetMatrix(A);
     ...
     solver.SetMatrix(B);
     ...
   If called before any calls to SetMatrix(), the identity elimination ordering
   is used. */
  void UpdateMatrix(const SymmetricMatrix& A);

  /* Computes the supernodal LLT factorization. Returns true if factorization
   succeeds, otherwise returns false. Failure is triggered by an internal
   failure of Eigen::LLT.  This can fail if, for instance, the input matrix set
   in SetMatrix() or UpdateMatrix() is not positive definite. If failure is
   encountered, the user should verify that the specified matrix is positive
   definite and not poorly conditioned.
   @throws std::exception if SetMatrix() or UpdateMatrix() has not been called
   since construction or the last time Factor is called(). */
  bool Factor();

  /* Solves the system A⋅x = b and returns x.
   @throws std::exception if Factor() has not been called. */
  VectorX<double> Solve(const VectorX<double>& b) const;

  /* Solves the system A⋅x = b and writes the result in b.
   @throws std::exception if Factor() has not been called. */
  void SolveInPlace(VectorX<double>* y) const;

 private:
  using LowerTriangularMatrix =
      BlockSparseLowerTriangularOrSymmetricMatrix<BlockType, false>;

  /* Performs L(j+1:, j+1:) -= L(j+1:, j) * L(j+1:, j).transpose().
   @pre 0 <= j < L.block_cols(). */
  void RightLookingSymmetricRank1Update(int j);

  /* Permutes the given matrix A with `block_permutation_` p and set L such that
   the lower triangular part of L satisfies L(p(i), p(j)) = A(i, j).
   @pre SetMarix() has been called. */
  void PermuteAndCopyToL(const SymmetricMatrix& A);

  std::unique_ptr<LowerTriangularMatrix> L_;
  std::vector<Eigen::LLT<BlockType>> L_diag_;
  /* The mapping from the internal indices (i.e, the indices for L_) to
   the indices of the original matrix supplied in SetMatrix(). */
  PartialPermutation block_permutation_;
  PartialPermutation scalar_permutation_;
  bool is_factored_{false};
  bool matrix_set_{false};
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
