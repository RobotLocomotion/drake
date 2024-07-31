#include "drake/multibody/contact_solvers/sap/sap_distance_constraint.h"

#include <limits>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/math/autodiff.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
SapDistanceConstraint<T>::ComplianceParameters::ComplianceParameters(
    T stiffness, T damping)
    : stiffness_(std::move(stiffness)), damping_(std::move(damping)) {
  DRAKE_DEMAND(stiffness_ > 0.0);
  DRAKE_DEMAND(damping_ >= 0.0);
}

template <typename T>
SapDistanceConstraint<T>::SapDistanceConstraint(Kinematics kinematics,
                                                ComplianceParameters parameters)
    : SapHolonomicConstraint<T>(
          MakeSapHolonomicConstraintKinematics(kinematics),
          MakeSapHolonomicConstraintParameters(parameters),
          {kinematics.objectA(), kinematics.objectB()}),
      kinematics_(std::move(kinematics)),
      parameters_(std::move(parameters)) {}

template <typename T>
typename SapHolonomicConstraint<T>::Parameters
SapDistanceConstraint<T>::MakeSapHolonomicConstraintParameters(
    const ComplianceParameters& p) {
  // "Near-rigid" regime parameter, see [Castro et al., 2022].
  // TODO(amcastro-tri): consider exposing this parameter.
  constexpr double kBeta = 0.1;

  // Distance constraints do not have impulse limits, they are bi-lateral
  // constraints. Each distance constraint introduces a single constraint
  // equation.
  constexpr double kInf = std::numeric_limits<double>::infinity();
  return typename SapHolonomicConstraint<T>::Parameters{
      Vector1<T>(-kInf), Vector1<T>(kInf), Vector1<T>(p.stiffness()),
      Vector1<T>(p.damping() / p.stiffness()), kBeta};
}

template <typename T>
typename SapHolonomicConstraint<T>::Kinematics
SapDistanceConstraint<T>::MakeSapHolonomicConstraintKinematics(
    const Kinematics& kinematics) {
  Vector1<T> g(kinematics.distance() -
               kinematics.length());  // Constraint function.
  Vector1<T> b = Vector1<T>::Zero();  // Bias term.

  // The constraint Jacobian is the projection of the relative velocity between
  // P and Q on p_hat. That is ḋ = p̂⋅v_PQ_W = p̂⋅(J_WBq - J_WAp)⋅v.
  // Therefore the distance constraint Jacobian is Jdist = p̂⋅J.
  SapConstraintJacobian<T> Jdist =
      kinematics.jacobian().LeftMultiplyByTranspose(kinematics.p_hat_W());

  return typename SapHolonomicConstraint<T>::Kinematics(
      std::move(g), std::move(Jdist), std::move(b));
}

template <typename T>
void SapDistanceConstraint<T>::DoAccumulateSpatialImpulses(
    int i, const Eigen::Ref<const VectorX<T>>& gamma,
    SpatialForce<T>* F) const {
  if (i == 0) {
    // Object A.
    const Vector3<T> f_Ap_W = -gamma(0) * kinematics_.p_hat_W();
    const Vector3<T> t_A_W = kinematics_.p_AP_W().cross(f_Ap_W);
    const SpatialForce<T> F_Ao_W(t_A_W, f_Ap_W);
    *F += F_Ao_W;
  } else {
    // Object B.
    const Vector3<T> f_Bq_W = gamma(0) * kinematics_.p_hat_W();
    const Vector3<T> t_B_W = kinematics_.p_BQ_W().cross(f_Bq_W);
    const SpatialForce<T> F_Bo_W(t_B_W, f_Bq_W);
    *F += F_Bo_W;
    return;
  }
}

template <typename T>
std::unique_ptr<SapConstraint<double>> SapDistanceConstraint<T>::DoToDouble()
    const {
  const typename SapDistanceConstraint<T>::Kinematics& k = kinematics_;
  const typename SapDistanceConstraint<T>::ComplianceParameters& p =
      parameters_;

  SapDistanceConstraint<double>::Kinematics k_to_double(
      k.objectA(), math::DiscardGradient(k.p_WP()),
      math::DiscardGradient(k.p_AP_W()), k.objectB(),
      math::DiscardGradient(k.p_WQ()), math::DiscardGradient(k.p_BQ_W()),
      ExtractDoubleOrThrow(k.length()), k.jacobian().ToDouble());

  SapDistanceConstraint<double>::ComplianceParameters p_to_double(
      ExtractDoubleOrThrow(p.stiffness()), ExtractDoubleOrThrow(p.damping()));

  return std::make_unique<SapDistanceConstraint<double>>(
      std::move(k_to_double), std::move(p_to_double));
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapDistanceConstraint);
