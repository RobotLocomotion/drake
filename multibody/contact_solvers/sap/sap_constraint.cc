#include "drake/multibody/contact_solvers/sap/sap_constraint.h"

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
SapConstraint<T>::SapConstraint(int clique, const MatrixX<T>& J)
    : num_constrained_dofs_(J.rows()), clique0_(clique), J0_(J) {
  DRAKE_DEMAND(clique >= 0);
}

template <typename T>
SapConstraint<T>::SapConstraint(int clique0, int clique1, const MatrixX<T>& J0,
                                const MatrixX<T>& J1)
    : num_constrained_dofs_(J0.rows()),
      clique0_(clique0),
      num_velocities0_(J0.cols()),
      clique1_(clique1),
      num_velocities1_(J1.cols()),
      J0_(J0),
      J1_(J1) {
  DRAKE_DEMAND(clique0 >= 0);
  DRAKE_DEMAND(clique1 >= 0);
  DRAKE_DEMAND(clique0 != clique1);
  DRAKE_DEMAND(J0.rows() == J1.rows());
}

template <typename T>
int SapConstraint<T>::num_cliques() const {
  return clique1_ < 0 ? 1 : 2;
}

template <typename T>
int SapConstraint<T>::num_constrained_dofs() const {
  return num_constrained_dofs_;
}

template <typename T>
int SapConstraint<T>::clique0() const {
  return clique0_;
}

template <typename T>
int SapConstraint<T>::clique1() const {
  return clique1_;
}

template <typename T>
const MatrixX<T>& SapConstraint<T>::clique0_jacobian() const {
  return J0_;
}

template <typename T>
const MatrixX<T>& SapConstraint<T>::clique1_jacobian() const {
  return J1_;
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapConstraint)
