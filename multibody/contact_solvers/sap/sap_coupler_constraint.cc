#include "drake/multibody/contact_solvers/sap/sap_coupler_constraint.h"

#include <limits>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
SapCouplerConstraint<T>::SapCouplerConstraint(Kinematics kinematics)
    : SapHolonomicConstraint<T>(
          MakeSapHolonomicConstraintKinematics(kinematics),
          MakeSapHolonomicConstraintParameters(), {}),
      kinematics_(std::move(kinematics)) {}

template <typename T>
typename SapHolonomicConstraint<T>::Parameters
SapCouplerConstraint<T>::MakeSapHolonomicConstraintParameters() {
  // "Near-rigid" regime parameter, see [Castro et al., 2022].
  // TODO(amcastro-tri): consider exposing this parameter.
  constexpr double kBeta = 0.1;

  // Coupler constraints do not have impulse limits, they are bi-lateral
  // constraints. Each coupler constraint introduces a single constraint
  // equation.
  //
  // N.B. `stiffness` is set to infinity to model this constraint in the
  // "near-rigid" regime. `relaxation_time` is set arbitrarily to 0.0.
  // SapHolonomicConstraint will use the time step as the relaxation time when
  // it detects this constraint is "near-rigid". See
  // SapHolonomicConstraint::DoMakeData().
  constexpr double kInf = std::numeric_limits<double>::infinity();
  return typename SapHolonomicConstraint<T>::Parameters{
      Vector1<T>(-kInf), Vector1<T>(kInf), Vector1<T>(kInf), Vector1<T>(0.0),
      kBeta};
}

template <typename T>
typename SapHolonomicConstraint<T>::Kinematics
SapCouplerConstraint<T>::MakeSapHolonomicConstraintKinematics(
    const Kinematics& kinematics) {
  Vector1<T> g(kinematics.q0 - kinematics.gear_ratio * kinematics.q1 -
               kinematics.offset);    // Constraint function.
  Vector1<T> b = Vector1<T>::Zero();  // Bias term.

  // Determine the single clique or two clique case.
  if (kinematics.clique0 == kinematics.clique1) {
    MatrixX<T> J0 = MatrixX<T>::Zero(1, kinematics.clique_nv0);
    J0(0, kinematics.clique_dof0) = 1.0;
    J0(0, kinematics.clique_dof1) = -kinematics.gear_ratio;

    return typename SapHolonomicConstraint<T>::Kinematics(
        std::move(g),
        SapConstraintJacobian<T>(kinematics.clique0, std::move(J0)),
        std::move(b));
  } else {
    MatrixX<T> J0 = MatrixX<T>::Zero(1, kinematics.clique_nv0);
    MatrixX<T> J1 = MatrixX<T>::Zero(1, kinematics.clique_nv1);
    J0(0, kinematics.clique_dof0) = 1.0;
    J1(0, kinematics.clique_dof1) = -kinematics.gear_ratio;

    return typename SapHolonomicConstraint<T>::Kinematics(
        std::move(g),
        SapConstraintJacobian<T>(kinematics.clique0, std::move(J0),
                                 kinematics.clique1, std::move(J1)),
        std::move(b));
  }
}

template <typename T>
void SapCouplerConstraint<T>::DoAccumulateGeneralizedImpulses(
    int c, const Eigen::Ref<const VectorX<T>>& gamma,
    EigenPtr<VectorX<T>> tau) const {
  // τ = Jᵀ⋅γ
  if (c == 0) {
    (*tau)(kinematics().clique_dof0) += gamma(0);
  }

  if (this->num_cliques() == 1 || c == 1) {
    (*tau)(kinematics().clique_dof1) -= kinematics().gear_ratio * gamma(0);
  }
}

template <typename T>
std::unique_ptr<SapConstraint<double>> SapCouplerConstraint<T>::DoToDouble()
    const {
  SapCouplerConstraint<double>::Kinematics k{
      kinematics_.clique0,
      kinematics_.clique_dof0,
      kinematics_.clique_nv0,
      ExtractDoubleOrThrow(kinematics_.q0),
      kinematics_.clique1,
      kinematics_.clique_dof1,
      kinematics_.clique_nv1,
      ExtractDoubleOrThrow(kinematics_.q1),
      ExtractDoubleOrThrow(kinematics_.gear_ratio),
      ExtractDoubleOrThrow(kinematics_.offset)};
  return std::make_unique<SapCouplerConstraint<double>>(std::move(k));
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapCouplerConstraint);
