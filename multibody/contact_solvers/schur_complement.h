#pragma once

#include <unordered_set>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/block_sparse_cholesky_solver.h"
#include "drake/multibody/contact_solvers/block_sparse_lower_triangular_or_symmetric_matrix.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

/* Given a symmetric linear system of equations Az = b in block form as:

     Dx  + By = 0     (1)
     Bᵀx + Cy = a     (2)

 where A = [D B; Bᵀ C], z = [x; y], b = [0; a]. If C (size p-by-p),
 D (size q-by-q) and A (size p+q-by-p+q) are positive definite, one can solve
 the system using Schur complement. Specifically, using equation (1), we get

     x = -D⁻¹By       (3)

 Plugging (3) in (1), we get

    (C - BᵀD⁻¹B)y = a.

 After a solution for y is obtained, we can use (3) to recover the solution for
 x. The matrix S = C - BᵀD⁻¹B is the Schur complement of the block D of the
 matrix A. Since A is positive definite, so is S.

 The decomposition of A into [D, B; Bᵀ, C] is defined by the block indices of D.
 (See the two-argument constructor for more detail.) Given the symmetric matrix
 A and its decomposition [D, B; Bᵀ, C], this class computes the Schur complement
 of the block D of A. The matrix is factorized in the process and consequently,
 one can solve the system Az = b efficiently once the Schur complement is
 computed. */
class SchurComplement {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SchurComplement);

  /* Constructs an empty SchurComplement, i.e one that corresponds
   corresponds to a linear system with no equations.*/
  SchurComplement();

  ~SchurComplement();

  /* Constructs a SchurComplement for the block sparse matrix A of size
   3*N-by-3*N consisting of blocks of size 3x3. `D_indices` determine which
   block rows/columns make up the D diagonal blocks. The remaining block
   rows/columns make up the C diagonal blocks. For example, if matrix A is given
   by

     x x x | u u u | w w w
     x x x | u u u | w w w
     x x x | u u u | w w w
     ------+-------+------
     u u u | y y y | v v v
     u u u | y y y | v v v
     u u u | y y y | v v v
     ------+-------+------
     w w w | v v v | z z z
     w w w | v v v | z z z
     w w w | v v v | z z z

   and D_indices = {0, 2},  then submatrix D is given by

     x x x | w w w
     x x x | w w w
     x x x | w w w
     ------+------
     w w w | z z z
     w w w | z z z
     w w w | z z z

   submatrix C is given by

     y y y
     y y y
     y y y

   submatrix B is given by

     u u u
     u u u
     u u u
     -----
     v v v
     v v v
     v v v

   @pre D_indices is a subset of {0, ..., A.block_cols()-1}. */
  SchurComplement(const BlockSparseSymmetricMatrix3d& A,
                  const std::unordered_set<int>& D_indices);

  /* Returns the Schur complement for the block D of the matrix A,
   S = C - BᵀD⁻¹B. */
  const MatrixX<double>& get_D_complement() const { return S_; }

  /* Given a value of y, solves for x in the equation Dx + By = 0.
   @pre The size of y is equal to the number of columns of B */
  VectorX<double> SolveForX(const Eigen::Ref<const VectorX<double>>& y) const;

  // TODO(xuchenhan-tri): Consider adding a SolveForY() function for
  // completeness.

  /* Given a right-hand side vector b with the same dimension as the input
   matrix A provided at construction, solve solves for A*z = b.
   @pre The size of b is compatible with the input matrix A provided at
   construction. */
  VectorX<double> Solve(const Eigen::Ref<const VectorX<double>>& b) const;

 private:
  /* Sorted block row/column indices for 3x3 blocks that belong to submatrix D.
   */
  std::vector<int> D_indices_;
  /* Sorted block row/column indices for 3x3 blocks that belong to submatrix C.
   */
  std::vector<int> C_indices_;
  /* Cholesky solver that factorizes the matrix and stores the factorization. */
  BlockSparseCholeskySolver<Matrix3<double>> A_solver_;
  /* The Schur complement of block D: S = C - BᵀD⁻¹B. */
  MatrixX<double> S_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
