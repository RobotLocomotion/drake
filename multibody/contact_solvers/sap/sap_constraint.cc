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
  return DoMakeData(time_step, delassus_estimation);
}

template <typename T>
void SapConstraint<T>::CalcData(const Eigen::Ref<const VectorX<T>>& vc,
                                AbstractValue* data) const {
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
  DoCalcImpulse(data, gamma);
}

template <typename T>
void SapConstraint<T>::CalcCostHessian(const AbstractValue& data,
                                       MatrixX<T>* G) const {
  DoCalcCostHessian(data, G);
}

template <typename T>
std::unique_ptr<AbstractValue> SapConstraint<T>::DoMakeData(
    const T&, const Eigen::Ref<const VectorX<T>>&) const {
  return AbstractValue::Make(VectorX<T>(this->num_constraint_equations()));
}

template <typename T>
void SapConstraint<T>::DoCalcData(const Eigen::Ref<const VectorX<T>>&,
                                  AbstractValue*) const {
  throw std::runtime_error("Implement me!!!");
}

template <typename T>
T SapConstraint<T>::DoCalcCost(const AbstractValue&) const {
  throw std::runtime_error("Implement me!!!");
}

template <typename T>
void SapConstraint<T>::DoCalcImpulse(const AbstractValue&,
                                     EigenPtr<VectorX<T>>) const {
  throw std::runtime_error("Implement me!!!");
}

template <typename T>
void SapConstraint<T>::DoCalcCostHessian(const AbstractValue&,
                                         MatrixX<T>*) const {
  throw std::runtime_error("Implement me!!!");
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapConstraint)
