#include "drake/multibody/contact_solvers/contact_solver.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
ContactSolver<T>::~ContactSolver() = default;

template <typename T>
void ContactSolver<T>::FormDelassusOperatorMatrix(
    const LinearOperator<T>& G, const LinearOperator<T>& Ainv,
    const LinearOperator<T>& J, Eigen::SparseMatrix<T>* W) const {
  const int num_velocities = Ainv.rows();
  const int num_impulses = J.rows();
  DRAKE_DEMAND(G.rows() == num_impulses);
  DRAKE_DEMAND(G.cols() == num_velocities);
  DRAKE_DEMAND(Ainv.rows() == num_velocities);
  DRAKE_DEMAND(Ainv.cols() == num_velocities);
  DRAKE_DEMAND(J.rows() == num_impulses);
  DRAKE_DEMAND(J.cols() == num_velocities);
  DRAKE_DEMAND(W->rows() == num_impulses);
  DRAKE_DEMAND(W->cols() == num_impulses);

  Eigen::SparseVector<T> ej(num_impulses);
  // N.B. ej.makeCompressed() is not available for SparseVector.
  ej.coeffRef(0) = 1.0;  // Effectively allocate one non-zero entry.

  Eigen::SparseVector<T> JTcolj(num_velocities);
  Eigen::SparseVector<T> AinvJTcolj(num_velocities);
  Eigen::SparseVector<T> Wcolj(num_impulses);
  // Reserve maximum number of non-zeros.
  JTcolj.reserve(num_velocities);
  AinvJTcolj.reserve(num_velocities);
  Wcolj.reserve(num_impulses);

  // Loop over the j-th column.
  for (int j = 0; j < W->cols(); ++j) {
    // By changing the inner index, we change what entry is the non-zero with
    // value 1.0.
    *ej.innerIndexPtr() = j;

    // Reset to nnz = 0. Memory is not freed.
    JTcolj.setZero();
    AinvJTcolj.setZero();
    Wcolj.setZero();

    // Apply each operator in sequence.
    J.MultiplyByTranspose(ej, &JTcolj);  // JTcolj = Jᵀ * ej
    Ainv.Multiply(JTcolj, &AinvJTcolj);
    G.Multiply(AinvJTcolj, &Wcolj);
    W->col(j) = Wcolj;
  }
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::ContactSolver);
