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
      tau0_{tau0} {}

template <typename T>
std::unique_ptr<AbstractValue> SapExternalSystemConstraint<T>::DoMakeData(
    const T& dt, const Eigen::Ref<const VectorX<T>>&) const {
  const int nv = this->num_velocities(0);

  // TODO(vincekurtz): add near-rigid regularization like other SAP constraints

  SapExternalSystemConstraintData<T> data;
  data.time_step = dt;
  data.v = VectorX<T>::Zero(nv);
  data.cost = 0.0;
  data.impulse = VectorX<T>::Zero(nv);
  data.hessian = MatrixX<T>::Zero(nv, nv);

  return SapConstraint<T>::MoveAndMakeAbstractValue(std::move(data));
}

template <typename T>
void SapExternalSystemConstraint<T>::DoCalcData(
    const Eigen::Ref<const VectorX<T>>& v, AbstractValue* abstract_data) const {
  auto& data =
      abstract_data->get_mutable_value<SapExternalSystemConstraintData<T>>();
  const T& h = data.time_step;

  // TODO(vincekurtz): add effort limits
  data.v = v;
  data.hessian = A_tilde_;
  data.impulse = h * tau0_ - A_tilde_ * v;
  data.cost = 0.5 * v.transpose() * A_tilde_ * v - h * tau0_.dot(v);
}

template <typename T>
T SapExternalSystemConstraint<T>::DoCalcCost(
    const AbstractValue& abstract_data) const {
  const auto& data =
      abstract_data.get_value<SapExternalSystemConstraintData<T>>();
  return data.cost;
}

template <typename T>
void SapExternalSystemConstraint<T>::DoCalcImpulse(
    const AbstractValue& abstract_data, EigenPtr<VectorX<T>> gamma) const {
  const auto& data =
      abstract_data.get_value<SapExternalSystemConstraintData<T>>();
  *gamma = data.impulse;
}

template <typename T>
void SapExternalSystemConstraint<T>::DoCalcCostHessian(
    const AbstractValue& abstract_data, MatrixX<T>* G) const {
  const auto& data =
      abstract_data.get_value<SapExternalSystemConstraintData<T>>();
  *G = data.hessian;
}

template <typename T>
void SapExternalSystemConstraint<T>::DoAccumulateGeneralizedImpulses(
    int c, const Eigen::Ref<const VectorX<T>>& gamma,
    EigenPtr<VectorX<T>> tau) const {
  if (c == 0) {
    *tau += gamma;
  } else {
    DRAKE_UNREACHABLE();
  }
}

template <typename T>
SapConstraintJacobian<T> SapExternalSystemConstraint<T>::MakeConstraintJacobian(
    int clique, int nv) {
  MatrixX<T> J = MatrixX<T>::Identity(nv, nv);
  return SapConstraintJacobian<T>(clique, std::move(J));
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::
        SapExternalSystemConstraint);
