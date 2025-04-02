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
    Configuration configuration, const T& k, const T& tau0,
    const T& effort_limits)
    : SapConstraint<T>(MakeConstraintJacobian(configuration), {}),
      configuration_{std::move(configuration)},
      k_{k},
      tau0_{tau0},
      e_{effort_limits} {
  using std::isfinite;
  using std::isnan;

  DRAKE_DEMAND(isfinite(k_));
  DRAKE_DEMAND(isfinite(tau0_));
  DRAKE_DEMAND(!isnan(e_));
  DRAKE_DEMAND(k_ >= 0.0);
  DRAKE_DEMAND(e_ >= 0.0);
}

template <typename T>
std::unique_ptr<AbstractValue> SapExternalSystemConstraint<T>::DoMakeData(
    const T& dt, const Eigen::Ref<const VectorX<T>>&) const {
  // TODO(vincekurtz): add near-rigid regularization like other SAP constraints

  SapExternalSystemConstraintData<T> data;
  data.time_step = dt;
  return SapConstraint<T>::MoveAndMakeAbstractValue(std::move(data));
}

template <typename T>
void SapExternalSystemConstraint<T>::DoCalcData(
    const Eigen::Ref<const VectorX<T>>& vc,
    AbstractValue* abstract_data) const {
  auto& data =
      abstract_data->get_mutable_value<SapExternalSystemConstraintData<T>>();
  const T& h = data.time_step;
  const T& v = vc[0];

  // This uses essentially the same method as SapPdControllerConstraint
  // to enforce effort limits.
  const T y = -k_ * v + tau0_;

  data.v = v;

  data.cost = h * ClampAntiderivative(y, e_) / k_;
  data.impulse = h * Clamp(y, e_);
  data.hessian = h * k_ * ClampDerivative(y, e_);
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
  *gamma = Vector1<T>::Constant(data.impulse);
}

template <typename T>
void SapExternalSystemConstraint<T>::DoCalcCostHessian(
    const AbstractValue& abstract_data, MatrixX<T>* G) const {
  const auto& data =
      abstract_data.get_value<SapExternalSystemConstraintData<T>>();
  (*G)(0, 0) = data.hessian;
}

template <typename T>
void SapExternalSystemConstraint<T>::DoAccumulateGeneralizedImpulses(
    int, const Eigen::Ref<const VectorX<T>>& gamma,
    EigenPtr<VectorX<T>> tau) const {
  (*tau)(configuration_.clique_dof) += gamma(0);
}

template <typename T>
SapConstraintJacobian<T> SapExternalSystemConstraint<T>::MakeConstraintJacobian(
    Configuration c) {
  MatrixX<T> J = RowVectorX<T>::Unit(c.clique_nv, c.clique_dof);
  return SapConstraintJacobian<T>(c.clique, std::move(J));
}

template <typename T>
std::unique_ptr<SapConstraint<double>>
SapExternalSystemConstraint<T>::DoToDouble() const {
  const typename SapExternalSystemConstraint<T>::Configuration& c =
      configuration_;
  SapExternalSystemConstraint<double>::Configuration c_to_double{
      c.clique,
      c.clique_nv,
      c.clique_dof,
  };
  return std::make_unique<SapExternalSystemConstraint<double>>(
      std::move(c_to_double), ExtractDoubleOrThrow(k_),
      ExtractDoubleOrThrow(tau0_), ExtractDoubleOrThrow(e_));
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::
        SapExternalSystemConstraint);
