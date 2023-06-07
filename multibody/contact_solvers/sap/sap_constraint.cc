#include "drake/multibody/contact_solvers/sap/sap_constraint.h"

#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
SapConstraint<T>::SapConstraint(SapConstraintJacobian<T> J) : J_(std::move(J)) {
  DRAKE_THROW_UNLESS(J_.rows() > 0);
}

template <typename T>
std::unique_ptr<AbstractValue> SapConstraint<T>::MakeData(
    const T& time_step,
    const Eigen::Ref<const VectorX<T>>& delassus_estimation) const {
  DRAKE_DEMAND(delassus_estimation.size() == num_constraint_equations());
  return DoMakeData(time_step, delassus_estimation);
}

template <typename T>
void SapConstraint<T>::CalcData(const Eigen::Ref<const VectorX<T>>& vc,
                                AbstractValue* data) const {
  DRAKE_DEMAND(vc.size() == num_constraint_equations());
  DRAKE_DEMAND(data != nullptr);
  DoCalcData(vc, data);
}

template <typename T>
T SapConstraint<T>::CalcCost(const AbstractValue& data) const {
  return DoCalcCost(data);
}

template <typename T>
void SapConstraint<T>::CalcImpulse(const AbstractValue& data,
                                   EigenPtr<VectorX<T>> gamma) const {
  DRAKE_DEMAND(gamma != nullptr);
  DoCalcImpulse(data, gamma);
}

template <typename T>
void SapConstraint<T>::CalcCostHessian(const AbstractValue& data,
                                       MatrixX<T>* G) const {
  DRAKE_DEMAND(G != nullptr);
  const int ne = num_constraint_equations();
  G->resize(ne, ne);
  DoCalcCostHessian(data, G);
}

template <typename T>
void SapConstraint<T>::AccumulateGeneralizedImpulses(
    int c, const Eigen::Ref<const VectorX<T>>& gamma,
    EigenPtr<VectorX<T>> tau) const {
  DRAKE_THROW_UNLESS(0 <= c && c < num_cliques());
  DRAKE_THROW_UNLESS(gamma.size() == num_constraint_equations());
  DRAKE_THROW_UNLESS(tau != nullptr);
  DRAKE_THROW_UNLESS(tau->size() == num_velocities(c));
  DoAccumulateGeneralizedImpulses(c, gamma, tau);
}

template <typename T>
void SapConstraint<T>::AccumulateSpatialImpulses(
    int o, const Eigen::Ref<const VectorX<T>>& gamma,
    SpatialForce<T>* F) const {
  DRAKE_THROW_UNLESS(o < num_objects());
  DRAKE_THROW_UNLESS(gamma.size() == num_constraint_equations());
  DRAKE_THROW_UNLESS(F != nullptr);
  DoAccumulateSpatialImpulses(o, gamma, F);
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapConstraint)
