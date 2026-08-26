#include "drake/multibody/contact_solvers/sap/sap_joint_friction_constraint.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/common/extract_double.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
SapJointFrictionConstraint<T>::SapJointFrictionConstraint(int clique,
                                                          int clique_dof,
                                                          int clique_nv,
                                                          Parameters parameters)
    : SapConstraint<T>(CalcConstraintJacobian(clique, clique_dof, clique_nv),
                       {}),
      parameters_(std::move(parameters)),
      clique_dof_(clique_dof) {
  DRAKE_DEMAND(parameters_.friction > 0.0);
  DRAKE_DEMAND(parameters_.sigma > 0.0);
}

template <typename T>
SapConstraintJacobian<T> SapJointFrictionConstraint<T>::CalcConstraintJacobian(
    int clique, int clique_dof, int clique_nv) {
  DRAKE_DEMAND(clique >= 0);
  DRAKE_DEMAND(0 <= clique_dof && clique_dof < clique_nv);
  MatrixX<T> J = MatrixX<T>::Zero(1, clique_nv);
  J(0, clique_dof) = 1.0;
  return SapConstraintJacobian<T>(clique, std::move(J));
}

template <typename T>
std::unique_ptr<AbstractValue> SapJointFrictionConstraint<T>::DoMakeData(
    const T& time_step,
    const Eigen::Ref<const VectorX<T>>& delassus_estimation) const {
  // Regularization R = σ⋅w, with w the diagonal approximation of the Delassus
  // operator for this constraint, which estimates the inverse effective inertia
  // of the constrained DOF. See class documentation.
  const T R = parameters_.sigma * delassus_estimation(0);
  // The Delassus estimation w = 1/Aᵢᵢ is positive and finite for any DOF with
  // a positive diagonal entry in the (positive definite) dynamics matrix A.
  // Both are needed for R⁻¹ and the cost to be well defined.
  constexpr double kInf = std::numeric_limits<double>::infinity();
  DRAKE_DEMAND(R > 0.0 && R < kInf);
  const T gamma_max = time_step * parameters_.friction;
  SapJointFrictionConstraintData<T> data(R, gamma_max);
  return SapConstraint<T>::MoveAndMakeAbstractValue(std::move(data));
}

template <typename T>
void SapJointFrictionConstraint<T>::DoCalcData(
    const Eigen::Ref<const VectorX<T>>& vc,
    AbstractValue* abstract_data) const {
  using std::max;
  using std::min;
  auto& data =
      abstract_data->get_mutable_value<SapJointFrictionConstraintData<T>>();
  const T& gamma_max = data.gamma_max();
  data.mutable_vc() = vc(0);
  data.mutable_y() = -vc(0) * data.R_inv();
  data.mutable_gamma() = max(-gamma_max, min(gamma_max, data.y()));
}

template <typename T>
T SapJointFrictionConstraint<T>::DoCalcCost(
    const AbstractValue& abstract_data) const {
  using std::abs;
  const auto& data =
      abstract_data.get_value<SapJointFrictionConstraintData<T>>();
  const T& R = data.R();
  const T& gamma_max = data.gamma_max();
  const T& vc = data.vc();
  const T& y = data.y();
  // The stiction condition |vc| ≤ R⋅γₘₐₓ is equivalent to |y| ≤ γₘₐₓ.
  if (abs(y) <= gamma_max) {
    return 0.5 * R * y * y;  // Equals vc²/(2R).
  }
  return gamma_max * abs(vc) - 0.5 * R * gamma_max * gamma_max;
}

template <typename T>
void SapJointFrictionConstraint<T>::DoCalcImpulse(
    const AbstractValue& abstract_data, EigenPtr<VectorX<T>> gamma) const {
  const auto& data =
      abstract_data.get_value<SapJointFrictionConstraintData<T>>();
  (*gamma)(0) = data.gamma();
}

template <typename T>
void SapJointFrictionConstraint<T>::DoCalcCostHessian(
    const AbstractValue& abstract_data, MatrixX<T>* G) const {
  using std::abs;
  const auto& data =
      abstract_data.get_value<SapJointFrictionConstraintData<T>>();
  // The Hessian is R⁻¹ in stiction and zero in sliding. At the transition we
  // pick the stiction value, consistently with the branch used in DoCalcCost().
  (*G)(0, 0) = abs(data.y()) <= data.gamma_max() ? data.R_inv() : T(0.0);
}

template <typename T>
void SapJointFrictionConstraint<T>::DoAccumulateGeneralizedImpulses(
    int, const Eigen::Ref<const VectorX<T>>& gamma,
    EigenPtr<VectorX<T>> tau) const {
  // N.B. The NVI guarantees the clique index is correct, and since there is
  // only one, we don't need to use it.
  // For this constraint the generalized impulses are τ = Jᵀ⋅γ = eᵢ⋅γ.
  (*tau)(clique_dof_) += gamma(0);
}

template <typename T>
std::unique_ptr<SapConstraint<double>>
SapJointFrictionConstraint<T>::DoToDouble() const {
  SapJointFrictionConstraint<double>::Parameters p_to_double{
      ExtractDoubleOrThrow(parameters_.friction), parameters_.sigma};
  // N.B. Joint friction constraints always act on a single clique.
  const int clique = this->first_clique();
  const int clique_nv = this->num_velocities(0);
  return std::make_unique<SapJointFrictionConstraint<double>>(
      clique, clique_dof(), clique_nv, std::move(p_to_double));
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::
        SapJointFrictionConstraint);
