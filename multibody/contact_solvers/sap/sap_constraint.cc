#include "drake/multibody/contact_solvers/sap/sap_constraint.h"

#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
SapConstraint<T>::SapConstraint(int clique, VectorX<T> g, MatrixBlock<T> J)
    : first_clique_(clique),
      g_(std::move(g)),
      first_clique_jacobian_(std::move(J)) {
  DRAKE_THROW_UNLESS(clique >= 0);
  DRAKE_THROW_UNLESS(constraint_function().size() >= 0);
  DRAKE_THROW_UNLESS(first_clique_jacobian().rows() ==
                     constraint_function().size());
}

template <typename T>
SapConstraint<T>::SapConstraint(int first_clique, int second_clique,
                                VectorX<T> g, MatrixBlock<T> J_first_clique,
                                MatrixBlock<T> J_second_clique)
    : first_clique_(first_clique),
      second_clique_(second_clique),
      g_(std::move(g)),
      first_clique_jacobian_(std::move(J_first_clique)),
      second_clique_jacobian_(std::move(J_second_clique)) {
  DRAKE_THROW_UNLESS(first_clique >= 0);
  DRAKE_THROW_UNLESS(second_clique >= 0);
  DRAKE_THROW_UNLESS(first_clique != second_clique);
  DRAKE_THROW_UNLESS(constraint_function().size() >= 0);
  DRAKE_THROW_UNLESS(first_clique_jacobian().rows() ==
                     second_clique_jacobian().rows());
  DRAKE_THROW_UNLESS(constraint_function().size() ==
                     first_clique_jacobian().rows());
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapConstraint)
