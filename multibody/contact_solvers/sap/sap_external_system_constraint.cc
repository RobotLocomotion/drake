#include "drake/multibody/contact_solvers/sap/sap_external_system_constraint.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
SapExternalSystemConstraint<T>::SapExternalSystemConstraint(
    int clique, int nv, const MatrixX<T>& A_tilde, const VectorX<T>& tau0)
    : SapConstraint<T>(MakeConstraintJacobian(clique, nv), {}),
     A_tilde_{A_tilde}, 
     tau0_{tau0}
    {}

template <typename T>
std::unique_ptr<AbstractValue> SapExternalSystemConstraint<T>::DoMakeData(
    const T&, const Eigen::Ref<const VectorX<T>>&) const {
  // Placeholder for actual data
  int data = 0;
  return SapConstraint<T>::MoveAndMakeAbstractValue(std::move(data));
}

template <typename T>
void SapExternalSystemConstraint<T>::DoCalcData(
    const Eigen::Ref<const VectorX<T>>&, AbstractValue*) const {}

template <typename T>
T SapExternalSystemConstraint<T>::DoCalcCost(
    const AbstractValue&) const {
  return 0.0;
}

template <typename T>
void SapExternalSystemConstraint<T>::DoCalcImpulse(
    const AbstractValue&, EigenPtr<VectorX<T>> gamma) const {
  *gamma = Vector1<T>::Zero();
}

template <typename T>
void SapExternalSystemConstraint<T>::DoCalcCostHessian(
    const AbstractValue&, MatrixX<T>* G) const {
  G->setZero();
}

template <typename T>
void SapExternalSystemConstraint<T>::DoAccumulateGeneralizedImpulses(
    int, const Eigen::Ref<const VectorX<T>>&, EigenPtr<VectorX<T>>) const {}

template <typename T>
SapConstraintJacobian<T> SapExternalSystemConstraint<T>::MakeConstraintJacobian(
    int clique, int nv) {
  MatrixX<T> J = MatrixX<T>::Zero(1, nv);
  return SapConstraintJacobian<T>(clique, std::move(J));
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::
        SapExternalSystemConstraint);
