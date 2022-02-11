#include "drake/multibody/contact_solvers/sap/sap_constraint.h"

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
SapConstraint<T>::SapConstraint(int clique, const VectorX<T>& g,
                                const MatrixX<T>& J)
    : first_clique_(clique), g_(g), first_clique_jacobian_(J) {
  DRAKE_THROW_UNLESS(clique >= 0);
  DRAKE_THROW_UNLESS(g.size() >= 0);
  DRAKE_THROW_UNLESS(J.rows() == g.size());
}

template <typename T>
SapConstraint<T>::SapConstraint(int first_clique, int second_clique,
                                const VectorX<T>& g,
                                const MatrixX<T>& J_first_clique,
                                const MatrixX<T>& J_second_clique)
    : first_clique_(first_clique),
      second_clique_(second_clique),
      g_(g),
      first_clique_jacobian_(J_first_clique),
      second_clique_jacobian_(J_second_clique) {
  DRAKE_THROW_UNLESS(first_clique >= 0);
  DRAKE_THROW_UNLESS(second_clique >= 0);
  DRAKE_THROW_UNLESS(first_clique != second_clique);
  DRAKE_THROW_UNLESS(g.size() >= 0);
  DRAKE_THROW_UNLESS(J_first_clique.rows() == J_second_clique.rows());
  DRAKE_THROW_UNLESS(g.size() == J_first_clique.rows());
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapConstraint)
