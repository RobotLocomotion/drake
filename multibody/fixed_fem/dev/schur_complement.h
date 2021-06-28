#pragma once

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
namespace internal {

/* Computes the Schur complement of a matrix given its block components.

Given a linear system of equations Mz = c that can be written in block form
as:
    Ax + By  =  a     (1)
    Bᵀx + Dy =  0     (2)
where M = [A B; Bᵀ D], zᵀ = [xᵀ yᵀ], cᵀ = [aᵀ 0ᵀ], and A(size p-by-p),
D(size q-by-q) and M(size p+q-by-p+q) are positive definite, we may choose to
solve the system using Schur complement. Specifically, using equation (2), we
get
    y = -D⁻¹Bᵀx       (3)
Plugging (3) in (1), we get
   (A - BD⁻¹Bᵀ)x = a.
After a solution for x is obtained, we can use (3) to recover the solution for
y. The matrix A - BD⁻¹Bᵀ is the Schur complement of the block D of the matrix M.

@tparam_nonsymbolic_scalar */
template <typename T>
class SchurComplement {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SchurComplement);

  /* Creates an empty Schur complement. This allows SchurComplement to be
   directly constructed in containers. */
  SchurComplement()
      : neg_Dinv_B_transpose_(MatrixX<T>::Zero(0, 0)),
        D_complement_(MatrixX<T>::Zero(0, 0)) {}

  // TODO(xuchenhan-tri): Consider supporting a constructor taking sparse
  //  matrices. To effectively utilize the sparsity, we need a sparse linear
  //  solver.
  /* Calculates the Schur complement of the positive matrix M = [A B; Bᵀ D]
  given its block components.
  @pre A, D, and M are symmetric positive definite. Note that this prerequisite
       is not checked at construction as the check is expensive. Caller to the
       constructor should take care to pass in valid arguments. One way of
       making sure that this prerequisite is satisfied is by passing in
       components of a matrix M that is known to be symmetric positive-definite.
  @pre A.rows() == B.rows().
  @pre B.cols() == D.cols(). */
  SchurComplement(const Eigen::Ref<const MatrixX<T>>& A,
                  const Eigen::Ref<const MatrixX<T>>& B,
                  const Eigen::Ref<const MatrixX<T>>& D);

  /* Returns the Schur complement for the block D of the matrix M, A - BD⁻¹Bᵀ.
   */
  const MatrixX<T>& get_D_complement() const { return D_complement_; }

  /* Solves for y given the solution for x assuming the right hand side takes
  the form  [aᵀ 0ᵀ]. See class documentation. */
  VectorX<T> SolveForY(const VectorX<T>& x) const;

 private:
  int p_{0};                         // Number of rows and columns for A.
  int q_{0};                         // Number of rows and columns for D.
  MatrixX<T> neg_Dinv_B_transpose_;  // -D⁻¹Bᵀ.
  MatrixX<T> D_complement_;          // A - BD⁻¹Bᵀ.
};

}  // namespace internal
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fixed_fem::internal::SchurComplement)
