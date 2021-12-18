#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* A symmetric block sparse matrix data structure that uses PETSc for storage
 and linear algebra operations. It provides supports for the inverse operator,
 i.e. one can solve for A*x = b where A is this matrix. It also supports
 calculating the Schur complement of the matrix. It only supports double as
 the scalar type. */
class PetscSymmetricBlockSparseMatrix {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PetscSymmetricBlockSparseMatrix);

  enum class SolverType {
    /* For positive definite matrix. */
    kConjugateGradient,
    /* For positive definite matrix. Can only be paired with Cholesky
     preconditioner. */
    kDirect,
    /* For generic symmetric matrix. */
    kMINRES
  };

  enum class PreconditionerType {
    /* For generic matrix. N.B. the BlockJacobi preconditioner from PETSc by
     default seems to assume positive definite blocks.*/
    kJacobi,
    /* For positive definite matrix. */
    kCholesky,
    kIncompleteCholesky,
  };

  /* Constructs a symmetric block sparse matrix.
   @param size            The number of rows and columns of the symmetric
                          matrix.
   @param block_size      The number rows and columns within a single non-zero
                          block. Each non-zero block in the sparse matrix is of
                          the same size.
   @param num_upper_triangular_blocks_per_row
                          `num_upper_triangular_blocks_per_row[i]` contains the
                          number of non-zero upper triangular and diagonal
                          blocks in the i-th row block.

   @pre `size` >=  0, and `size` is a integer multiple of `block_size`.
   @pre num_upper_triangular_blocks_per_row.size() == size/block_size.
   @pre num_upper_triangular_blocks_per_row[i] <= size/block_size. */
  PetscSymmetricBlockSparseMatrix(
      int size, int block_size,
      const std::vector<int>& num_upper_triangular_blocks_per_row);

  ~PetscSymmetricBlockSparseMatrix();

  /* Creates a deep identical copy of this matrix.
   @pre Matrix must be assembled. See AssembleIfNecessary(). */
  std::unique_ptr<PetscSymmetricBlockSparseMatrix> Clone() const;

  /* Accumulate values in the block matrix. The Eigen analogy of this operation
   is
   ```
   for (int i = 0; i < block_indices.size(); ++i) {
     for (int j = 0; j < block_indices.size(); ++j){
       // `A` is `this` matrix. `b` is block size specified at construction.
       const int block_row = block_indices(i);
       const int block_col = block_indices(j);
       A.block(block_row * b, block_col * b, b, b) +=
           block.block(i * b, j * b, b, b);
     }
   }
   ```
   @pre block is symmetric.
   @pre block.rows() = block_indices.size() * block_size.
   @pre 0 <= block_indices[i] * block_size < size for each i =
   0, ..., block_indices.size()-1.
   @warn None of the prerequisite is checked since this is used in inner loop.
   @warn Over the lifespan of this object, for each row block i, the number of
   blocks accumulated in the i-th row block through `AddToBlock()` whose column
   block index is >= i must not exceed the number of nonzero upper triangular
   blocks in the i-th row block specified at construction.
   @note The full matrix is expected for `block` even though the matrix is
   symmetric. */
  void AddToBlock(const VectorX<int>& block_indices,
                  const MatrixX<double>& block);

  /* Solves for A*x = b using the given type of solver, where A is this matrix.
   @warn The compatibility of the solver and preconditioner type with the
   problem at hand is not checked. Callers need to be careful to choose the
   reasonable combination of solver and preconditioner given the type of matrix.
   @pre b.size() == A.rows()
   @note The matrix/preconditioner will be refactored upon successive call to
   this method even if the matrix hasn't changed.
   @pre Matrix must be assembled. See AssembleIfNecessary(). */
  VectorX<double> Solve(SolverType solver_type,
                        PreconditionerType preconditioner_type,
                        const VectorX<double>& b) const;

  /* Similar to Solve(), but writes the result in `b`.
   @pre Matrix must be assembled. See AssembleIfNecessary(). */
  void SolveInPlace(SolverType solver_type,
                    PreconditionerType preconditioner_type,
                    EigenPtr<VectorX<double>> b) const;

  /* Sets all blocks to zeros while maintaining the sparsity pattern. */
  void SetZero();

  /* Makes a dense matrix representation of this block-sparse matrix.
   @pre Matrix must be assembled. See AssembleIfNecessary(). */
  MatrixX<double> MakeDenseMatrix() const;

  /* Zeros out all rows and columns whose index is included in `indexes` and
   sets the diagonal entry of these rows and columns to `value`. In other words,
   if `i` is contained in `indexes`, then the i-th row and column of the matrix
   is set to zero and i-th diagonal entry is set to `value`. This operation
   doesn't change the sparsity pattern.
   @pre 0 <= indexes[i] < rows() for each i = 0, 1, ..., indexes.size()-1.  */
  void ZeroRowsAndColumns(const std::vector<int>& indexes, double value);

  /* Sets the relative tolerance of the linear solve A*x = b. Doesn't affect the
   result of Solve() when the direct solver is used.
   In particular, this sets the relative convergence tolerance in
   https://petsc.org/main/docs/manualpages/KSP/KSPSetTolerances.html. All the
   other tolerance parameters (e.g. absolute tolerance, maximum number of
   iterations, etc) are set to the default value specified in the PETSc
   documentaion. */
  void SetRelativeTolerance(double tolerance);

  // TODO(xuchenhan-tri): Support Schur complement.

  int rows() const;

  int cols() const;

  /* Turn the matrix into "assembled" state if it's not already "assembled".
   This method is cheap if the matrix is already "assembled", so invoke it if
   unsure. */
  void AssembleIfNecessary();

 private:
  class Impl;

  /* Default construct that facilites the Clone() method. */
  PetscSymmetricBlockSparseMatrix();

  std::unique_ptr<Impl> pimpl_;
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
