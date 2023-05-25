#include "drake/multibody/contact_solvers/sap/sap_constraint_jacobian.h"

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

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapConstraintJacobian)
