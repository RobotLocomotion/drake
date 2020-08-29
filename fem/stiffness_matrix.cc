#include "drake/fem/stiffness_matrix.h"

namespace drake {
namespace fem {

template <typename T>
void StiffnessMatrix<T>::Reinitialize() {
  if (!matrix_free_) {
    /* TODO(xuchenhan-tri): Use the objective to set the size of the matrix
     and set its sparsity pattern.*/
    /*
      int matrix_size = objective_.get_num_dofs();
      if (matrix_.cols() != matrix_size) {
        matrix_.resize(matrix_size, matrix_size);
        objective_.SetSparsityPattern(&matrix_);
      }
    */
  }
}

template <typename T>
VectorX<T> StiffnessMatrix<T>::Multiply(
    const Eigen::Ref<const VectorX<T>>& x) const {
  /* TODO(xuchenhan-tri): call the alternative signature for matrix free
   multiplications. */
  /*
  if (matrix_free_) {
    Matrix3X<T> b(3, x.size() / 3);
    Multiply(x, &b);
    return Eigen::Map<VectorX<T>>(b.data(), b.size());
  }
  */
  DRAKE_DEMAND(!matrix_free_);
  VectorX<T> b = matrix_ * x;
  // TODO(xuchenhan-tri): project away the Dirichlet dofs.
  /*
  Eigen::Ref<Matrix3X<T>> tmp_b =
      Eigen::Map<Matrix3X<T>>(b.data(), 3, b.size() / 3);
  objective_.Project(&tmp_b);
  return Eigen::Map<VectorX<T>>(tmp_b.data(), tmp_b.size());
   */
  return b;
}

// TODO(xuchenhan-tri): use the objective to perform matrix-free multiply.
/*
template <typename T>
void StiffnessMatrix<T>::Multiply(const Eigen::Ref<const VectorX<T>>& x,
                                  EigenPtr<Matrix3X<T>> b) const {
  DRAKE_DEMAND(matrix_free_);
  const Matrix3X<T>& tmp_x =
      Eigen::Map<const Matrix3X<T>>(x.data(), 3, x.size() / 3);
  return objective_.Multiply(tmp_x, b);
}
*/

}  // namespace fem
}  // namespace drake
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
        class ::drake::fem::StiffnessMatrix)
