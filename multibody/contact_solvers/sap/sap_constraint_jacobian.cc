#include "drake/multibody/contact_solvers/sap/sap_constraint_jacobian.h"

#include "drake/math/autodiff.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
SapConstraintJacobian<T>::SapConstraintJacobian(int clique, MatrixBlock<T> J) {
  DRAKE_THROW_UNLESS(clique >= 0);
  clique_jacobians_.emplace_back(clique, std::move(J));
}

template <typename T>
SapConstraintJacobian<T>::SapConstraintJacobian(int clique, MatrixX<T> J)
    : SapConstraintJacobian(clique, MatrixBlock<T>(std::move(J))) {}

template <typename T>
SapConstraintJacobian<T>::SapConstraintJacobian(
    int first_clique, MatrixBlock<T> J_first_clique, int second_clique,
    MatrixBlock<T> J_second_clique) {
  DRAKE_THROW_UNLESS(first_clique >= 0);
  DRAKE_THROW_UNLESS(second_clique >= 0);
  DRAKE_THROW_UNLESS(first_clique != second_clique);
  DRAKE_THROW_UNLESS(J_first_clique.rows() == J_second_clique.rows());
  clique_jacobians_.reserve(2);
  clique_jacobians_.emplace_back(first_clique, std::move(J_first_clique));
  clique_jacobians_.emplace_back(second_clique, std::move(J_second_clique));
}

template <typename T>
SapConstraintJacobian<T>::SapConstraintJacobian(int first_clique,
                                                MatrixX<T> J_first_clique,
                                                int second_clique,
                                                MatrixX<T> J_second_clique)
    : SapConstraintJacobian(
          first_clique, MatrixBlock<T>(std::move(J_first_clique)),
          second_clique, MatrixBlock<T>(std::move(J_second_clique))) {}

template <typename T>
bool SapConstraintJacobian<T>::blocks_are_dense() const {
  bool is_dense = clique_jacobian(0).is_dense();
  if (num_cliques() == 2) {
    is_dense = is_dense && clique_jacobian(1).is_dense();
  }
  return is_dense;
}

template <typename T>
SapConstraintJacobian<T> SapConstraintJacobian<T>::LeftMultiplyByTranspose(
    const Eigen::Ref<const MatrixX<T>>& A) const {
  DRAKE_THROW_UNLESS(blocks_are_dense());

  // Contribution from first the clique.
  const MatrixX<T> J_first_clique = clique_jacobian(0).MakeDenseMatrix();
  MatrixX<T> ATJ_first_clique = A.transpose() * J_first_clique;
  if (num_cliques() == 1) {
    return SapConstraintJacobian<T>(clique(0), std::move(ATJ_first_clique));
  }
  // Jacobian involving two cliques, second clique.
  const MatrixX<T> J_second_clique = clique_jacobian(1).MakeDenseMatrix();
  MatrixX<T> ATJ_second_clique = A.transpose() * J_second_clique;
  return SapConstraintJacobian<T>(clique(0), std::move(ATJ_first_clique),
                                  clique(1), std::move(ATJ_second_clique));
}

template <typename T>
SapConstraintJacobian<double> SapConstraintJacobian<T>::ToDouble() const {
  const MatrixBlock<T>& first_block = clique_jacobian(0);
  DRAKE_THROW_UNLESS(first_block.is_dense());
  MatrixX<double> J_first_clique =
      math::DiscardGradient(first_block.MakeDenseMatrix());
  if (num_cliques() == 1) {
    return SapConstraintJacobian<double>(clique(0), std::move(J_first_clique));
  }
  const MatrixBlock<T>& second_block = clique_jacobian(1);
  DRAKE_THROW_UNLESS(second_block.is_dense());
  MatrixX<double> J_second_clique =
      math::DiscardGradient(second_block.MakeDenseMatrix());
  return SapConstraintJacobian<double>(clique(0), std::move(J_first_clique),
                                       clique(1), std::move(J_second_clique));
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapConstraintJacobian);
