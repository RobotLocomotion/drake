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
  DoCalcCostHessian(data, G);
}

template <typename T>
std::unique_ptr<AbstractValue> SapConstraint<T>::DoMakeData(
    const T& time_step,
    const Eigen::Ref<const VectorX<T>>& delassus_estimation) const {
  // TODO(amcastro-tri): make use of the full Delassus estimation, not just the
  // mean.
  const T wi = delassus_estimation.sum() / delassus_estimation.size();
  VectorX<T> v_hat = CalcBiasTerm(time_step, wi);
  VectorX<T> R = CalcDiagonalRegularization(time_step, wi);
  SapConstraintData<T> data(std::move(R), std::move(v_hat));
  return AbstractValue::Make(data);
}

template <typename T>
void SapConstraint<T>::DoCalcData(const Eigen::Ref<const VectorX<T>>& vc,
                                  AbstractValue* abstract_data) const {
  auto& data = abstract_data->get_mutable_value<SapConstraintData<T>>();
  const VectorX<T>& R_inv = data.R_inv();
  const VectorX<T>& v_hat = data.v_hat();
  data.mutable_vc() = vc;
  data.mutable_y() = R_inv.asDiagonal() * (v_hat - vc);
  Project(data.y(), data.R(), &data.mutable_gamma(), &data.mutable_dPdy());
}

template <typename T>
T SapConstraint<T>::DoCalcCost(const AbstractValue& abstract_data) const {
  const auto& data = abstract_data.get_value<SapConstraintData<T>>();
  const VectorX<T>& R = data.R();
  const VectorX<T>& gamma = data.gamma();
  const T cost = 0.5 * gamma.dot(R.asDiagonal() * gamma);
  return cost;
}

template <typename T>
void SapConstraint<T>::DoCalcImpulse(const AbstractValue& abstract_data,
                                     EigenPtr<VectorX<T>> gamma) const {
  const auto& data = abstract_data.get_value<SapConstraintData<T>>();
  *gamma = data.gamma();
}

template <typename T>
void SapConstraint<T>::DoCalcCostHessian(const AbstractValue& abstract_data,
                                         MatrixX<T>* G) const {
  const auto& data = abstract_data.get_value<SapConstraintData<T>>();
  *G = data.dPdy() * data.R_inv().asDiagonal();
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapConstraint)
